#include "dbcbs_core.hpp"

#include <algorithm>
#include <cassert>
#include <chrono>
#include <fstream>
#include <iostream>
#include <memory>
#include <unordered_map>

// yaml
#include <yaml-cpp/yaml.h>

// motion library / planner internals (same includes as db_cbs.cpp)
#include "robots.h"
#include "robotStatePropagator.hpp"
#include "fclStateValidityChecker.hpp"
#include "db_astar.hpp"
#include "planresult.hpp"

// dynoplan optimizer (same header as your binary)
#include "../dynoplan/include/dynoplan/optimization/multirobot_optimization.hpp"

// OMPL / FCL / Boost heap
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/control/SpaceInformation.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <boost/heap/d_ary_heap.hpp>

// msgpack for motions
#include <msgpack.hpp>

// Filesystem
#include <filesystem>
namespace fs = std::filesystem;

namespace ob = ompl::base;
namespace oc = ompl::control;

namespace dbcbs {

// ---------------- internal helpers ----------------

struct Constraint {
  float time;
  ob::State* constrained_state;
};

struct Conflict {
  float time;
  size_t robot_idx_i;
  ob::State* robot_state_i;
  size_t robot_idx_j;
  ob::State* robot_state_j;
};

struct HighLevelNode {
  std::vector<LowLevelPlan<AStarNode*, ob::State*, oc::Control*>> solution;
  std::vector<std::vector<Constraint>> constraints;
  float cost{};
  int   id{};
  typename boost::heap::d_ary_heap<HighLevelNode, boost::heap::arity<2>,
                                   boost::heap::mutable_<true>>::handle_type handle;
  bool operator<(const HighLevelNode& n) const { return cost > n.cost; }
};

static bool getEarliestConflict(
    const std::vector<LowLevelPlan<AStarNode*,ob::State*, oc::Control*>>& solution,
    const std::vector<std::shared_ptr<Robot>>& all_robots,
    std::shared_ptr<fcl::BroadPhaseCollisionManagerf> col_mng_robots,
    const std::vector<fcl::CollisionObjectf*>& col_mng_objs,
    Conflict& early_conflict)
{
  size_t max_t = 0;
  for (const auto& sol : solution) max_t = std::max(max_t, sol.trajectory.size() - 1);

  ob::State* node_state;
  std::vector<ob::State*> node_states;

  for (size_t t = 0; t <= max_t; ++t){
    node_states.clear();
    size_t obj_idx = 0;
    for (size_t i = 0; i < all_robots.size(); ++i){
      if (t >= solution[i].trajectory.size()) node_state = solution[i].trajectory.back();
      else                                     node_state = solution[i].trajectory[t];
      node_states.push_back(node_state);
      for (size_t p = 0; p < all_robots[i]->numParts(); ++p) {
        const auto transform = all_robots[i]->getTransform(node_state,p);
        col_mng_objs[obj_idx]->setTranslation(transform.translation());
        col_mng_objs[obj_idx]->setRotation(transform.rotation());
        col_mng_objs[obj_idx]->computeAABB();
        ++obj_idx;
      }
    }
    col_mng_robots->update(col_mng_objs);
    fcl::DefaultCollisionData<float> collision_data;
    col_mng_robots->collide(&collision_data, fcl::DefaultCollisionFunction<float>);
    if (collision_data.result.isCollision()) {
      assert(collision_data.result.numContacts() > 0);
      const auto& contact = collision_data.result.getContact(0);

      early_conflict.time         = static_cast<float>(t * all_robots[0]->dt());
      early_conflict.robot_idx_i  = (size_t)contact.o1->getUserData();
      early_conflict.robot_idx_j  = (size_t)contact.o2->getUserData();
      early_conflict.robot_state_i= node_states[early_conflict.robot_idx_i];
      early_conflict.robot_state_j= node_states[early_conflict.robot_idx_j];
      return true;
    }
  }
  return false;
}

static inline void createConstraintsFromConflicts(
  const Conflict& c, std::map<size_t, std::vector<Constraint>>& constraints)
{
  constraints[c.robot_idx_i].push_back({c.time, c.robot_state_i});
  constraints[c.robot_idx_j].push_back({c.time, c.robot_state_j});
}

static ShapeSpec toShapeSpec(const Footprint& fp)
{
  ShapeSpec ss;
  if (fp.kind == Footprint::Circle) {
    ss.kind   = ShapeSpec::Kind::Circle;
    ss.radius = fp.radius;
  } else if (fp.kind == Footprint::Box) {
    ss.kind = ShapeSpec::Kind::Box;
    if (!fp.parts.empty()) {
      ss.sx = fp.parts[0].sx;
      ss.sy = fp.parts[0].sy;
    }
  } else if (fp.kind == Footprint::MultiBox) {
    ss.kind = ShapeSpec::Kind::MultiBox;
    for (const auto& p : fp.parts) {
      ShapeBoxPart part;
      part.cx = p.cx; part.cy = p.cy; part.sx = p.sx; part.sy = p.sy; part.angle = p.angle;
      ss.parts.push_back(part);
    }
  }
  return ss;
}

static int defaultDimForType(const std::string& ty) {
  if (ty.find("double_integrator") != std::string::npos) return 4; // x,y,vx,vy
  if (ty.find("unicycle") != std::string::npos)          return 3; // x,y,theta
  if (ty.find("car_") != std::string::npos)              return 3; // x,y,theta
  return 2;
}

// ---------------- core entry ----------------

bool solve(const Environment& env,
           const std::vector<RobotSpec>& robots_in,
           const Settings& cfg,
           Solution& out)
{
  out = Solution{};

  // bounds
  ob::RealVectorBounds position_bounds(2);
  position_bounds.setLow (0, env.minx);
  position_bounds.setLow (1, env.miny);
  position_bounds.setHigh(0, env.maxx);
  position_bounds.setHigh(1, env.maxy);

  fcl::AABBf workspace_aabb(
    fcl::Vector3f(env.minx, env.miny, -1),
    fcl::Vector3f(env.maxx, env.maxy,  1));

  // obstacles
  std::vector<fcl::CollisionObjectf*> obstacles;
  obstacles.reserve(env.obstacles.size());
  std::vector<std::shared_ptr<fcl::CollisionGeometryf>> obst_geoms; obst_geoms.reserve(env.obstacles.size());
  for (const auto& o : env.obstacles) {
    std::shared_ptr<fcl::CollisionGeometryf> geom(new fcl::Boxf(o.sx, o.sy, 1.0));
    auto co = new fcl::CollisionObjectf(geom);
    co->setTranslation(fcl::Vector3f(o.cx, o.cy, 0));
    co->computeAABB();
    obstacles.push_back(co);
    obst_geoms.push_back(geom);
  }

  // robots + motion libraries
  std::vector<std::shared_ptr<Robot>> robots;
  robots.reserve(robots_in.size());
  std::vector<std::string> robot_types;
  robot_types.reserve(robots_in.size());

  std::map<std::string, Motions> robot_motions;

  for (const auto& r : robots_in) {
    const std::string robotType = r.type;
    robot_types.push_back(robotType);

    ShapeSpec shape = toShapeSpec(r.fp);
    std::shared_ptr<Robot> robot = create_robot(robotType, position_bounds, shape);
    robots.push_back(robot);

    // load motions per type once
    if (robot_motions.find(robotType) == robot_motions.end()) {
      std::string motionsFile;
      if (robotType == "unicycle_first_order_0" || robotType == "unicycle_first_order_0_sphere") {
        motionsFile = (DBCBS_MOTIONS_BASE "/unicycle_first_order_0_sorted.msgpack");
      } else if (robotType == "unicycle_second_order_0") {
        motionsFile = (DBCBS_MOTIONS_BASE "/unicycle_second_order_0_sorted.msgpack");
      } else if (robotType == "double_integrator_0") {
        motionsFile = (DBCBS_MOTIONS_BASE "/double_integrator_0_sorted.msgpack");
      } else if (robotType == "car_first_order_with_1_trailers_0") {
        motionsFile = (DBCBS_MOTIONS_BASE "/car_first_order_with_1_trailers_0_sorted.msgpack");
      } else {
        throw std::runtime_error("dbcbs_core: Unknown motion filename for robottype: " + robotType);
      }

      std::ifstream is(motionsFile.c_str(), std::ios::in | std::ios::binary);
      if (!is) throw std::runtime_error("dbcbs_core: cannot open motion file: " + motionsFile);
      is.seekg(0, is.end); int length = static_cast<int>(is.tellg()); is.seekg(0, is.beg);
      msgpack::unpacker unpacker;
      unpacker.reserve_buffer(length);
      is.read(unpacker.buffer(), length);
      unpacker.buffer_consumed(length);
      msgpack::object_handle oh;
      unpacker.next(oh);
      load_motions(oh.get(), robot, robotType, 2 /*x,y min*/, robot_motions[robotType]);
    }
  }

  // starts / goals
  std::vector<std::vector<double>> starts, goals;
  starts.reserve(robots_in.size()); goals.reserve(robots_in.size());
  for (const auto& r : robots_in) {
    const int dim = std::max<int>(2, std::max(r.start.size(), r.goal.size()));
    std::vector<double> s(dim, 0.0), g(dim, 0.0);
    for (int i=0;i<dim && i<(int)r.start.size();++i) s[i]=r.start[i];
    for (int i=0;i<dim && i<(int)r.goal .size();++i) g[i]=r.goal [i];
    starts.push_back(std::move(s));
    goals .push_back(std::move(g));
  }

  // Heuristics
  const bool filter_duplicates = cfg.filter_duplicates;
  const float alpha            = cfg.alpha;
  std::vector<ompl::NearestNeighbors<AStarNode*>*> heuristics(robots.size(), nullptr);

  if (cfg.heuristic1 == "reverse-search") {
    for (auto& iter : robot_motions) {
      for (size_t i = 0; i < robot_types.size(); ++i) {
        if (iter.first == robot_types[i]) {
          disable_motions(robots[i], cfg.heuristic1_delta, filter_duplicates, alpha, 99999, iter.second);
          break;
        }
      }
    }
    DBAstar<Constraint> llplanner(cfg.heuristic1_delta, alpha);
    for (size_t i = 0; i < robots.size(); ++i) {
      LowLevelPlan<AStarNode*,ob::State*,oc::Control*> ll_result;
      std::vector<double> v_nanf(starts[i].size(), nanf(""));
      llplanner.search(robot_motions.at(robot_types[i]), v_nanf, goals[i],
                       obstacles, workspace_aabb, robots[i], {},
                       /*reverse_search*/true, ll_result, nullptr, &heuristics[i]);
    }
  }

  // FCL for inter-robot checks
  std::vector<fcl::CollisionObjectf*> col_mng_objs;
  col_mng_objs.reserve(robots.size()*3);
  std::shared_ptr<fcl::BroadPhaseCollisionManagerf> col_mng_robots =
    std::make_shared<fcl::DynamicAABBTreeCollisionManagerf>();
  col_mng_robots->setup();
  for (size_t i = 0; i < robots.size(); ++i) {
    for (size_t p = 0; p < robots[i]->numParts(); ++p) {
      auto coll_obj = new fcl::CollisionObjectf(robots[i]->getCollisionGeometry(p));
      size_t userData = i;
      robots[i]->getCollisionGeometry(p)->setUserData((void*)userData);
      col_mng_objs.push_back(coll_obj);
    }
  }
  col_mng_robots->registerObjects(col_mng_objs);

  // Timelimit
  const auto t0 = std::chrono::steady_clock::now();
  auto time_exceeded = [&]() {
    if (cfg.timelimit_s <= 0) return false;
    const auto dt = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    return dt > cfg.timelimit_s;
  };

  // HL loop
  float  delta       = cfg.delta0;
  size_t max_motions = static_cast<size_t>(std::max(1, cfg.num_primitives0));
  bool   solved_db   = false;

  for (size_t iteration = 0; ; ++iteration) {
    if (iteration > 0) {
      if (solved_db) delta *= cfg.delta_rate;
      else           delta *= 0.99f;
      max_motions = std::min<size_t>(static_cast<size_t>(max_motions * cfg.num_primitives_rate), static_cast<size_t>(1e6));
    }

    // enable motions per type
    for (auto& iter : robot_motions) {
      for (size_t i = 0; i < robot_types.size(); ++i) {
        if (iter.first == robot_types[i]) {
          disable_motions(robots[i], delta, filter_duplicates, alpha, max_motions, iter.second);
          break;
        }
      }
    }

    solved_db = false;

    HighLevelNode start;
    start.solution.resize(robots.size());
    start.constraints.resize(robots.size());
    start.cost = 0.0f;
    start.id   = 0;

    bool start_valid = true;
    for (size_t i = 0; i < robots.size(); ++i) {
      DBAstar<Constraint> llplanner(delta, alpha);
      if (!llplanner.search(robot_motions.at(robot_types[i]), starts[i], goals[i],
                            obstacles, workspace_aabb, robots[i], start.constraints[i],
                            /*reverse_search*/false, start.solution[i], heuristics[i])) {
        start_valid = false; break;
      }
      start.cost += start.solution[i].cost;
    }
    if (!start_valid) {
      if (time_exceeded()) break;
      continue;
    }

    typename boost::heap::d_ary_heap<HighLevelNode, boost::heap::arity<2>,
                                     boost::heap::mutable_<true> > open;
    auto handle = open.push(start);
    (*handle).handle = handle;
    int id = 1;

    size_t expands = 0;
    while (!open.empty()) {
      if (time_exceeded()) break;

      HighLevelNode P = open.top();
      open.pop();
      Conflict inter_robot_conflict;
      if (!getEarliestConflict(P.solution, robots, col_mng_robots, col_mng_objs, inter_robot_conflict)) {
        solved_db = true;

        // Fill output
        out.cost = 0.f;
        out.agents.resize(P.solution.size());
        for (size_t i = 0; i < P.solution.size(); ++i) {
          const auto& src = P.solution[i];
          auto&       dst = out.agents[i];
          dst.cost = src.cost;
          out.cost += src.cost;

          // states
          dst.states.reserve(src.trajectory.size());
          std::vector<double> reals;
          auto si = robots[i]->getSpaceInformation();
          for (auto* s : src.trajectory) {
            reals.clear();
            si->getStateSpace()->copyToReals(reals, s);
            dst.states.push_back(reals);
          }
          // actions
          dst.actions.reserve(src.actions.size());
          const size_t dim = robots[i]->getSpaceInformation()->getControlSpace()->getDimension();
          for (auto* u : src.actions) {
            std::vector<double> uvec;
            uvec.reserve(dim);
            for (size_t d = 0; d < dim; ++d) {
              double* addr = si->getControlSpace()->getValueAddressAtIndex(u, d);
              uvec.push_back(*addr);
            }
            dst.actions.push_back(std::move(uvec));
          }
        }
        return true;
      }

      ++expands;

      // branch on conflict
      std::map<size_t, std::vector<Constraint>> constraints;
      createConstraintsFromConflicts(inter_robot_conflict, constraints);

      for (const auto& c : constraints) {
        HighLevelNode newNode = P;
        const size_t i = c.first;
        newNode.id = id++;
        newNode.constraints[i].insert(newNode.constraints[i].end(), c.second.begin(), c.second.end());
        newNode.cost -= newNode.solution[i].cost;

        DBAstar<Constraint> llplanner(delta, alpha);
        if (llplanner.search(robot_motions.at(robot_types[i]), starts[i], goals[i],
                             obstacles, workspace_aabb, robots[i], newNode.constraints[i],
                             /*reverse_search*/false, newNode.solution[i], heuristics[i])) {
          newNode.cost += newNode.solution[i].cost;
          auto h = open.push(newNode);
          (*h).handle = h;
        }
      }
    }

    if (solved_db || time_exceeded()) break;
  }

  return false;
}

// ---------------- YAML writers (library-agnostic) ----------------

static inline void write_flow_seq(std::ostream& os, const std::vector<double>& v) {
  os << "[";
  for (size_t i = 0; i < v.size(); ++i) {
    os << v[i];
    if (i + 1 < v.size()) os << ",";
  }
  os << "]";
}

bool export_solution_yaml(const Solution& S,
                          const std::vector<RobotSpec>& /*robots*/,
                          const std::string& path)
{
  std::ofstream out(path);
  if (!out) return false;

  out << "cost: " << S.cost << "\n";
  out << "result:\n";
  for (const auto& A : S.agents) {
    out << "  - states:\n";
    for (const auto& st : A.states) {
      out << "      - ";
      write_flow_seq(out, st);
      out << "\n";
    }
    out << "    actions:\n";
    for (const auto& u : A.actions) {
      out << "      - ";
      write_flow_seq(out, u);
      out << "\n";
    }
  }
  out.flush();
  return static_cast<bool>(out);
}

bool export_joint_yaml(const Solution& S,
                       const std::vector<RobotSpec>& /*robots*/,
                       const std::string& path)
{
  std::ofstream out(path);
  if (!out) return false;

  // gather max lengths
  size_t max_t = 0, max_a = 0;
  double total_cost = 0.0;
  for (const auto& A : S.agents) {
    max_t = std::max(max_t, A.states.size());
    max_a = std::max(max_a, A.actions.size());
    total_cost += A.cost;
  }

  out << "cost: " << total_cost << "\n";
  out << "result:\n";
  out << "  - states:\n";
  for (size_t t = 0; t < max_t; ++t) {
    std::vector<double> joint;
    for (const auto& A : S.agents) {
      const auto& st = (t < A.states.size()) ? A.states[t] : A.states.back();
      joint.insert(joint.end(), st.begin(), st.end());
    }
    out << "      - ";
    write_flow_seq(out, joint);
    out << "\n";
  }

  out << "    actions:\n";
  for (size_t t = 0; t < max_a; ++t) {
    std::vector<double> joint;
    for (const auto& A : S.agents) {
      if (A.actions.empty()) continue;
      const auto& u = (t < A.actions.size()) ? A.actions[t] : A.actions.back();
      joint.insert(joint.end(), u.begin(), u.end());
    }
    out << "      - ";
    write_flow_seq(out, joint);
    out << "\n";
  }

  out.flush();
  return static_cast<bool>(out);
}

// ---------------- optimizer wrapper ----------------

bool optimize_to_yaml(const std::string& problem_yaml,
                      const std::string& solution_yaml,
                      const std::string& opt_yaml,
                      bool sum_robot_cost)
{
  try {
    const bool feasible = execute_optimizationMultiRobot(
      problem_yaml, solution_yaml, opt_yaml, DBCBS_DYNOBENCH_BASE, sum_robot_cost);
    (void)feasible; // we don’t trust/require this; caller parses opt.yaml regardless
    return true;
  } catch (...) {
    return false;
  }
}

} // namespace dbcbs
