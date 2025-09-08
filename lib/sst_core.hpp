#pragma once
#include <vector>
#include <string>
#include <functional>
#include <utility>

namespace dbcbs_sst {

// ---- Geometry & footprint ---------------------------------------------------
struct BoxObs { double cx{}, cy{}, sx{}, sy{}; };

struct Footprint {
    // "circle" | "box" | "multibox" | ""
    std::string shape;
    double radius{-1.0};
    double size_x{-1.0}, size_y{-1.0};
    struct Part { double cx{}, cy{}, sx{}, sy{}, angle{}; };
    std::vector<Part> boxes;
    bool valid{false};
};

struct RobotMeta {
    std::string type; // e.g., "unicycle_first_order_0"
    Footprint fp;
};

// ---- Problem spec -----------------------------------------------------------
struct ProblemSpec {
    // Workspace bounds [min=(x,y), max=(x,y)]
    double min_x{0.0}, min_y{0.0};
    double max_x{10.0}, max_y{10.0};

    std::vector<BoxObs> obstacles;

    // Robots: each provides type/meta + start and goal (full state vectors)
    struct Robot {
        RobotMeta meta;
        std::vector<double> start; // x,y,theta... (as required by type)
        std::vector<double> goal;
    };
    std::vector<Robot> robots;
};

// ---- Planner cfg ------------------------------------------------------------
struct SSTConfig {
    // OMPL/propagation params
    double propagation_step_size{0.1};     // seconds
    int    control_duration_min{1};
    int    control_duration_max{10};

    // Goal / planner params
    double goal_epsilon{0.25};
    double goal_bias{0.05};

    // SST only
    double selection_radius{0.5};
    double pruning_radius{0.1};

    // Solver
    int timelimit_s{60};

    // "sst" or "rrt" (for quick comparisons)
    std::string planner{"sst"};
};

// ---- Result container -------------------------------------------------------
struct SSTResult {
    // Per-robot [time-ordered states][component]
    std::vector<std::vector<std::vector<double>>> states;
    // Per-robot [time-ordered controls][component]
    std::vector<std::vector<std::vector<double>>> actions;
    bool solved{false};
};

// Optional callback: (t_seconds, cost_value)
using IntermediateCallback = std::function<void(double,double)>;

// ---- API --------------------------------------------------------------------
bool sst_solve(const ProblemSpec& P,
               const SSTConfig&  C,
               SSTResult&        out,
               IntermediateCallback cb = nullptr);

} // namespace dbcbs_sst
