#include <boost/program_options.hpp>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <iostream>

#include "../lib/sst_core.hpp"

using namespace dbcbs_sst;

static ProblemSpec loadProblem(const std::string& path) {
    YAML::Node env = YAML::LoadFile(path);
    ProblemSpec P{};

    auto mn = env["environment"]["min"];
    auto mx = env["environment"]["max"];
    P.min_x = mn[0].as<double>(); P.min_y = mn[1].as<double>();
    P.max_x = mx[0].as<double>(); P.max_y = mx[1].as<double>();

    for (const auto& obs : env["environment"]["obstacles"]) {
        if (obs["type"].as<std::string>() != "box") continue;
        BoxObs b{};
        b.cx = obs["center"][0].as<double>();
        b.cy = obs["center"][1].as<double>();
        b.sx = obs["size"][0].as<double>();
        b.sy = obs["size"][1].as<double>();
        P.obstacles.push_back(b);
    }

    for (const auto& rn : env["robots"]) {
        ProblemSpec::Robot R{};
        R.meta.type = rn["type"].as<std::string>();

        if (auto fp = rn["footprint"]; fp && fp.IsMap()) {
            Footprint F{};
            F.shape = fp["shape"] ? fp["shape"].as<std::string>() : "";
            if (F.shape == "circle") {
                F.radius = fp["radius"].as<double>();
                F.valid = true;
            } else if (F.shape == "box") {
                F.size_x = fp["size"][0].as<double>();
                F.size_y = fp["size"][1].as<double>();
                F.valid = true;
            } else if (F.shape == "multibox") {
                for (const auto& b : fp["boxes"]) {
                    Footprint::Part p{};
                    p.cx = b["center"][0].as<double>();
                    p.cy = b["center"][1].as<double>();
                    p.sx = b["size"][0].as<double>();
                    p.sy = b["size"][1].as<double>();
                    p.angle = b["angle"] ? b["angle"].as<double>() : 0.0;
                    F.boxes.push_back(p);
                }
                F.valid = !F.boxes.empty();
            }
            R.meta.fp = F;
        }

        for (const auto& v : rn["start"]) R.start.push_back(v.as<double>());
        for (const auto& v : rn["goal"])  R.goal.push_back(v.as<double>());
        P.robots.push_back(std::move(R));
    }
    return P;
}

static SSTConfig loadCfg(const std::string& path, const std::string& planner, int timelimit) {
    YAML::Node C = YAML::LoadFile(path);
    SSTConfig cfg;
    cfg.planner              = planner;
    cfg.timelimit_s          = timelimit;
    cfg.propagation_step_size= C["propagation_step_size"].as<double>();
    cfg.control_duration_min = C["control_duration"][0].as<int>();
    cfg.control_duration_max = C["control_duration"][1].as<int>();
    cfg.goal_epsilon         = C["goal_epsilon"].as<double>();
    cfg.goal_bias            = C["goal_bias"].as<double>();
    if (planner == "sst") {
        cfg.selection_radius = C["selection_radius"].as<double>();
        cfg.pruning_radius   = C["pruning_radius"].as<double>();
    }
    return cfg;
}

int main(int argc, char** argv) {
    namespace po = boost::program_options;
    std::string inputFile, outputFile, statsFile, plannerDesc, cfgFile;
    int timelimit = 60;

    po::options_description desc("Allowed options");
    desc.add_options()
        ("help", "produce help message")
        ("input,i",   po::value<std::string>(&inputFile)->required(),  "input problem.yaml")
        ("output,o",  po::value<std::string>(&outputFile)->required(), "output solution.yaml")
        ("cfg,c",     po::value<std::string>(&cfgFile)->required(),    "config yaml")
        ("planner,p", po::value<std::string>(&plannerDesc)->default_value("sst"), "planner: sst|rrt")
        ("timelimit", po::value<int>(&timelimit)->default_value(60), "time limit (s)")
        ("stats",     po::value<std::string>(&statsFile)->default_value("ompl_stats.yaml"), "stats yaml");

    try {
        po::variables_map vm;
        po::store(po::parse_command_line(argc, argv, desc), vm);
        if (vm.count("help")) { std::cout << desc << "\n"; return 0; }
        po::notify(vm);
    } catch (std::exception& e) {
        std::cerr << e.what() << "\n" << desc << std::endl;
        return 1;
    }

    auto P = loadProblem(inputFile);
    auto C = loadCfg(cfgFile, plannerDesc, timelimit);

    std::ofstream stats(statsFile);
    stats << "stats:\n";

    SSTResult R;
    bool ok = sst_solve(P, C, R, [&](double t, double cost){
        stats << "  - t: " << t << "\n    cost: " << cost << "\n";
        stats.flush();
    });
    if (!ok) {
        std::cerr << "sst_solve failed (internal error)\n";
        return 2;
    }

    // Write solution.yaml (same schema as db-cbs outputs)
    std::ofstream out(outputFile);
    out << "result:\n";
    for (size_t i = 0; i < R.states.size(); ++i) {
        out << "  - states:\n";
        for (const auto& s : R.states[i]) {
            out << "      - [";
            for (size_t k = 0; k < s.size(); ++k) {
                out << s[k]; if (k+1 < s.size()) out << ",";
            }
            out << "]\n";
        }
        out << "    actions:\n";
        for (const auto& u : R.actions[i]) {
            out << "      - [";
            for (size_t k = 0; k < u.size(); ++k) {
                out << u[k]; if (k+1 < u.size()) out << ",";
            }
            out << "]\n";
        }
    }

    return R.solved ? 0 : 3;
}
