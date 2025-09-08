#include "sst_core.hpp"

#include <chrono>
#include <cassert>
#include <iostream>
#include <memory>
#include <algorithm>

// OMPL
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/control/SpaceInformation.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/control/planners/rrt/RRT.h>
#include <ompl/control/planners/sst/SST.h>
#include <ompl/base/objectives/ControlDurationObjective.h>

#include "robots.h"
#include "robotStatePropagator.hpp"
#include "fclStateValidityChecker.hpp"

namespace ob = ompl::base;
namespace oc = ompl::control;

namespace dbcbs_sst {

static ShapeSpec toShape(const Footprint& fp) {
    ShapeSpec s; // defaults: "use hardcoded"
    if (!fp.valid) return s;
    if (fp.shape == "circle") {
        s.kind = ShapeSpec::Kind::Circle;
        s.radius = static_cast<float>(fp.radius);
    } else if (fp.shape == "box") {
        s.kind = ShapeSpec::Kind::Box;
        s.sx = static_cast<float>(fp.size_x);
        s.sy = static_cast<float>(fp.size_y);
    } else if (fp.shape == "multibox") {
        s.kind = ShapeSpec::Kind::MultiBox;
        for (const auto& b : fp.boxes) {
            ShapeBoxPart p;
            p.cx = static_cast<float>(b.cx);
            p.cy = static_cast<float>(b.cy);
            p.sx = static_cast<float>(b.sx);
            p.sy = static_cast<float>(b.sy);
            p.angle = static_cast<float>(b.angle);
            s.parts.push_back(p);
        }
    }
    return s;
}

bool sst_solve(const ProblemSpec& P,
               const SSTConfig&  C,
               SSTResult&        out,
               IntermediateCallback cb)
{
    out = SSTResult{};

    // 1) Obstacles -> FCL
    std::vector<fcl::CollisionObjectf*> obstacles;
    obstacles.reserve(P.obstacles.size());
    for (const auto& o : P.obstacles) {
        auto geom = std::make_shared<fcl::Boxf>(static_cast<float>(o.sx),
                                                static_cast<float>(o.sy),
                                                1.0f);
        auto co = new fcl::CollisionObjectf(geom);
        co->setTranslation(fcl::Vector3f(static_cast<float>(o.cx),
                                         static_cast<float>(o.cy), 0.f));
        co->computeAABB();
        obstacles.push_back(co);
    }
    std::shared_ptr<fcl::BroadPhaseCollisionManagerf> bpcm_env(
        new fcl::DynamicAABBTreeCollisionManagerf());
    bpcm_env->registerObjects(obstacles);
    bpcm_env->setup();

    // 2) Position bounds
    ob::RealVectorBounds posb(2);
    posb.setLow(0, P.min_x);
    posb.setLow(1, P.min_y);
    posb.setHigh(0, P.max_x);
    posb.setHigh(1, P.max_y);

    // 3) Build individual robots
    std::vector<std::shared_ptr<Robot>> robots;
    robots.reserve(P.robots.size());
    for (const auto& R : P.robots) {
        robots.push_back(create_robot(R.meta.type, posb, toShape(R.meta.fp)));
    }

    // 4) Create joint robot and space info
    auto joint = create_joint_robot(robots);
    auto si = joint->getSpaceInformation();

    // OMPL setup
    si->setPropagationStepSize(C.propagation_step_size);
    si->setMinMaxControlDuration(C.control_duration_min, C.control_duration_max);

    auto svc = std::make_shared<fclStateValidityChecker>(si, bpcm_env, joint, true);
    si->setStateValidityChecker(svc);

    auto prop = std::make_shared<RobotStatePropagator>(si, joint);
    si->setStatePropagator(prop);
    si->setup();

    // 5) Problem definition
    auto pdef = std::make_shared<ob::ProblemDefinition>(si);

    // Concatenate starts / goals
    std::vector<double> start_reals, goal_reals;
    start_reals.reserve(64); goal_reals.reserve(64);
    for (const auto& R : P.robots) {
        start_reals.insert(start_reals.end(), R.start.begin(), R.start.end());
        goal_reals.insert(goal_reals.end(), R.goal.begin(), R.goal.end());
    }

    auto s0 = si->allocState();
    si->getStateSpace()->copyFromReals(s0, start_reals);
    si->enforceBounds(s0);
    pdef->addStartState(s0);
    si->freeState(s0);

    auto g0 = si->allocState();
    si->getStateSpace()->copyFromReals(g0, goal_reals);
    si->enforceBounds(g0);
    auto gs = std::make_shared<MultiRobotGoalState>(si);
    gs->setState(g0);
    gs->setThreshold(C.goal_epsilon);
    pdef->setGoal(gs);
    setMultiRobotGoals(joint, gs);
    si->freeState(g0);

    // 6) Planner
    std::shared_ptr<ob::Planner> planner;
    if (C.planner == "rrt") {
        auto rrt = new oc::RRT(si);
        rrt->setGoalBias(C.goal_bias);
        planner.reset(rrt);
    } else {
        auto sst = new oc::SST(si);
        sst->setGoalBias(C.goal_bias);
        sst->setSelectionRadius(C.selection_radius);
        sst->setPruningRadius(C.pruning_radius);
        planner.reset(sst);
    }

    pdef->setOptimizationObjective(std::make_shared<ob::ControlDurationObjective>(si));

    auto t0 = std::chrono::steady_clock::now();
    if (cb) {
        pdef->setIntermediateSolutionCallback(
            [t0, cb](const ob::Planner*, const std::vector<const ob::State*>&, const ob::Cost cost) {
                auto t1 = std::chrono::steady_clock::now();
                double t = std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count() / 1000.0;
                cb(t, cost.value());
            });
    }

    planner->setProblemDefinition(pdef);
    planner->setup();

    ob::PlannerStatus solved = planner->ob::Planner::solve(C.timelimit_s);
    out.solved = solved;

    if (!solved) return true; // no exception; just "unsolved"

    auto path = pdef->getSolutionPath()->as<oc::PathControl>();
    if (!path) return true;

    // Re-interpolate at robot dt
    si->setPropagationStepSize(joint->dt());
    path->interpolate(); // normalize to 1-step controls

    // Figure out state/control partition per robot
    const size_t N = robots.size();

    // states dims
    std::vector<size_t> state_dim(N), state_off(N);
    size_t acc = 0;
    for (size_t i = 0; i < N; ++i) {
        state_dim[i] = robots[i]->getSpaceInformation()->getStateSpace()->getDimension();
        state_off[i] = acc; acc += state_dim[i];
    }

    // control dims
    std::vector<size_t> ctrl_dim(N), ctrl_off(N);
    acc = 0;
    for (size_t i = 0; i < N; ++i) {
        ctrl_dim[i] = robots[i]->getSpaceInformation()->getControlSpace()->getDimension();
        ctrl_off[i] = acc; acc += ctrl_dim[i];
    }

    out.states.assign(N, {});
    out.actions.assign(N, {});

    // States
    for (size_t i = 0; i < path->getStateCount(); ++i) {
        std::vector<double> reals;
        si->getStateSpace()->copyToReals(reals, path->getState(i));
        for (size_t r = 0; r < N; ++r) {
            std::vector<double> slice;
            slice.reserve(state_dim[r]);
            slice.insert(slice.end(),
                         reals.begin() + static_cast<long>(state_off[r]),
                         reals.begin() + static_cast<long>(state_off[r] + state_dim[r]));
            out.states[r].push_back(std::move(slice));
        }
    }

    // Actions
    const size_t dim = si->getControlSpace()->getDimension();
    (void)dim;
    for (size_t i = 0; i < path->getControlCount(); ++i) {
        std::vector<double> u;
        u.reserve(dim);
        auto* action = path->getControl(i);
        for (size_t d = 0; d < dim; ++d) {
            double *addr = si->getControlSpace()->getValueAddressAtIndex(action, d);
            u.push_back(*addr);
        }
        for (size_t r = 0; r < N; ++r) {
            std::vector<double> slice;
            slice.reserve(ctrl_dim[r]);
            slice.insert(slice.end(),
                         u.begin() + static_cast<long>(ctrl_off[r]),
                         u.begin() + static_cast<long>(ctrl_off[r] + ctrl_dim[r]));
            out.actions[r].push_back(std::move(slice));
        }
    }

    return true;
}

} // namespace dbcbs_sst
