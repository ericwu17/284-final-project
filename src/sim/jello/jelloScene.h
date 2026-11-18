#ifndef JELLO_SCENE_H
#define JELLO_SCENE_H

#include <memory>
#include <string>

#include "softBody.h"
#include "tetMesh.h"

namespace Jello {

// A jello scene file (scene/jello/*.json). Example:
//   {
//     "description": "free text, ignored",
//     "jello":  { "size": [0.1, 0.1, 0.1], "cells": [8, 8, 8],
//                 "origin": [-0.05, 0.0, -0.05], "density": 1050 },
//     "solver": "static",
//     "ground": { "y": 0.0, "friction": 0.5 }
//   }
// Units are SI (meters, kg/m^3), Y is up. Every field is optional and
// defaults to the values below; unknown keys are an error, to catch typos.
struct JelloScene {
  Vector3D size = Vector3D(0.1, 0.1, 0.1);
  int cells[3] = {8, 8, 8};
  Vector3D origin = Vector3D(-0.05, 0.0, -0.05);  // minimum corner of the block
  double density = 1050;                          // kg/m^3

  std::string solver = "static";

  bool hasGround = true;
  double groundY = 0.0;
  double groundFriction = 0.5;

  // Parses `path`. On failure returns false and sets *error.
  bool load(const std::string &path, std::string *error);

  TetMesh buildMesh() const;
  // Mesh + solver. Returns nullptr (and sets *error) for an unknown solver.
  std::unique_ptr<SoftBody> buildBody(std::string *error) const;

  // True if the JSON file at `path` has a top-level "jello" object.
  static bool isJelloScene(const std::string &path);
};

} // namespace Jello

#endif // JELLO_SCENE_H
