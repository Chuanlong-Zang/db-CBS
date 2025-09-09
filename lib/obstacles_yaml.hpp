// lib/obstacles_yaml.hpp
#pragma once
#include <vector>
#include <memory>
#include <yaml-cpp/yaml.h>
#include <fcl/fcl.h>

// Build FCL collision objects for environment obstacles.
//
// Accepted shapes in YAML (env["obstacles"]):
//   - {type: box, center: [x,y], size: [w,h]}
//   - {type: circle, center: [x,y], radius: r}
//   - {type: polygon, points: [[x1,y1], [x2,y2], ...]}   # convex, no holes
//   - {type: capsule, p0: [x0,y0], p1: [x1,y1], radius: r}
//
// All obstacles are extruded to 3D with unit-ish thickness in Z.
// Returns true on success (malformed entries are skipped with warnings).
bool buildEnvironmentObstaclesFCL(
    const YAML::Node& envNode,
    std::vector<fcl::CollisionObjectf*>& outObjects,
    float prismThicknessZ = 1.0f  // Z thickness for polygon prisms & bars
);
