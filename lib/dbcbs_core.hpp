#pragma once
#include <string>
#include <vector>

namespace dbcbs {

struct BoxObs { float cx{}, cy{}, sx{}, sy{}; };

struct Footprint {
  enum Kind { None, Circle, Box, MultiBox } kind{None};
  float radius{0.f};                 // circle
  struct Part { float cx{}, cy{}, sx{}, sy{}, angle{}; };
  std::vector<Part> parts;           // for box (single part) & multibox
};

struct RobotSpec {
  std::string type;
  Footprint   fp;                    // optional footprint
  std::vector<double> start;         // state vector (x,y,theta, …)
  std::vector<double> goal;          // state vector
};

struct Environment {
  float minx{}, miny{}, maxx{}, maxy{};

  // Legacy (boxes-only) — still supported
  std::vector<BoxObs> obstacles;

  // NEW: if non-empty, this YAML snippet is used to build FCL obstacles.
  // It can be either:
  //  - a full "environment" map with "obstacles"
  //  - OR a plain sequence that is the "obstacles" array
  std::string obstacles_yaml;
};

struct Settings {
  float       alpha               {0.5f};
  bool        filter_duplicates   {true};
  std::string heuristic1          {"reverse-search"}; // or "none"
  float       heuristic1_delta    {1.0f};
  float       delta0              {0.5f};
  int         num_primitives0     {1000};
  float       delta_rate          {0.9f};
  float       num_primitives_rate {1.5f};
  double      timelimit_s         {300.0};            // hard wall from platform
};

// One agent’s detailed result (what we serialize in solution.yaml)
struct Trajectory {
  float cost{};
  std::vector<std::vector<double>> states;   // state[i] is [x,y,...]
  std::vector<std::vector<double>> actions;  // control vectors
};

struct Solution {
  float cost{};
  std::vector<Trajectory> agents;            // one per robot
};

// Core API: builds robots/motions, runs HL search, fills Solution.
// Returns true on success (feasible found before timelimit).
bool solve(const Environment& env,
           const std::vector<RobotSpec>& robots,
           const Settings& cfg,
           Solution& out);

// YAML helpers (raw states/actions, independent from OMPL print helpers)
bool export_solution_yaml(const Solution& S,
                          const std::vector<RobotSpec>& robots,
                          const std::string& solution_yaml_path);

bool export_joint_yaml(const Solution& S,
                       const std::vector<RobotSpec>& robots,
                       const std::string& joint_yaml_path);

// dynoplan wrapper (same call you use in db_cbs.cpp)
bool optimize_to_yaml(const std::string& problem_yaml,
                      const std::string& solution_yaml,
                      const std::string& opt_yaml,
                      bool sum_robot_cost);

} // namespace dbcbs
