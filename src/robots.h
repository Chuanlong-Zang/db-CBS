#pragma once

// OMPL headers
#include <ompl/control/SpaceInformation.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/base/goals/GoalState.h>
// FCL
#include <fcl/fcl.h>

class Robot
{
public:
  Robot()
  {
  }

  virtual void propagate(
    const ompl::base::State *start,
    const ompl::control::Control *control,
    const double duration,
    ompl::base::State *result) = 0;

  virtual fcl::Transform3f getTransform(
    const ompl::base::State *state,
    size_t part = 0) = 0;

  virtual void setPosition(ompl::base::State* state, const fcl::Vector3f position, size_t part = 0) = 0;

  virtual size_t numParts()
  {
    return 1;
  }

  virtual std::shared_ptr<fcl::CollisionGeometryf> getCollisionGeometry(size_t part = 0)
  {
    return geom_[part];
  }

  std::shared_ptr<ompl::control::SpaceInformation> getSpaceInformation()
  {
    return si_;
  }

  float dt() const
  {
    return dt_;
  }

  float is2D() const
  {
    return is2D_;
  }

  float maxSpeed() const
  {
    return max_speed_;
  }

protected:
  std::vector<std::shared_ptr<fcl::CollisionGeometryf>> geom_;
  std::shared_ptr<ompl::control::SpaceInformation> si_;
  float dt_;
  bool is2D_;
  float max_speed_;
};

class MultiRobotGoalState: public ompl::base::GoalState
{
public:
    MultiRobotGoalState(const ompl::base::SpaceInformationPtr &si): 
        ompl::base::GoalState(si)
    {
    }

    size_t numRobots() const
    {
      auto csi = dynamic_cast<ompl::control::SpaceInformation*>(si_.get()); 
      auto csp = csi->getStateSpace()->as<ompl::base::CompoundStateSpace>();
      return csp->getSubspaceCount();
    }

    double distanceGoal(const ompl::base::State* st, int robot_idx) const
    {
      auto csi = dynamic_cast<ompl::control::SpaceInformation*>(si_.get()); 
      auto csp = csi->getStateSpace()->as<ompl::base::CompoundStateSpace>();
      
      // auto csi = dynamic_cast<ompl::base::CompoundStateSpace*>(si_.get()); 
      auto cst = st->as<ompl::base::CompoundStateSpace::StateType>();
      auto cgoal_state = state_->as<ompl::base::CompoundStateSpace::StateType>();
      auto si_k = csp->getSubspace(robot_idx);
      auto st_k = (*cst)[robot_idx];
      auto goal_state_k = (*cgoal_state)[robot_idx];
      double dist = si_k->distance(st_k, goal_state_k);

      return dist;
    }

    // distance is the max of the individual robot distances
    double distanceGoal(const ompl::base::State *st) const
    {
      double result = 0;
      for (size_t k = 0; k < numRobots(); ++k) {
        result = fmax(result, distanceGoal(st, k));
      }
      return result;
    }

    bool isSatisfied(const ompl::base::State* st, int robot_idx)
    {
      double dist = distanceGoal(st, robot_idx);
      return dist < threshold_;
    }

};

std::shared_ptr<Robot> create_joint_robot(
  std::vector<std::shared_ptr<Robot>> robots);

// HACK for multi-robot
void setMultiRobotGoals(std::shared_ptr<Robot>, std::shared_ptr<MultiRobotGoalState> goals);


struct ShapeBoxPart { float cx{0}, cy{0}, sx{0.5f}, sy{0.25f}, angle{0}; }; // full sizes sx×sy
struct ShapeSpec {
  enum class Kind { Default, Circle, Box, MultiBox };
  Kind  kind = Kind::Default;
  float radius{0.0f};     // for Circle
  float sx{0.0f}, sy{0.0f}; // for Box (full size, not half)
  std::vector<ShapeBoxPart> parts; // for MultiBox
};
std::shared_ptr<Robot> create_robot(
  const std::string& robotType,
  const ompl::base::RealVectorBounds& positionBounds,
  const ShapeSpec& shape = {});