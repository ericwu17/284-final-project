#include "jelloScene.h"

#include <fstream>
#include <set>

#include "json.hpp"

using json = nlohmann::json;

namespace Jello {

namespace {

// The bundled json.hpp (v3.0) predates json::contains().
bool has(const json &obj, const char *key) { return obj.is_object() && obj.find(key) != obj.end(); }

// Throws std::runtime_error with a readable message on bad input; load()
// turns that into a false return.
void checkKeys(const json &obj, const std::string &where, const std::set<std::string> &allowed) {
  if (!obj.is_object()) throw std::runtime_error("\"" + where + "\" must be an object");
  for (auto it = obj.begin(); it != obj.end(); ++it)
    if (!allowed.count(it.key()))
      throw std::runtime_error("unknown key \"" + it.key() + "\" in \"" + where + "\"");
}

Vector3D readVec3(const json &obj, const char *key, const Vector3D &fallback) {
  if (!has(obj, key)) return fallback;
  const json &a = obj[key];
  if (!a.is_array() || a.size() != 3)
    throw std::runtime_error(std::string("\"") + key + "\" must be an array of 3 numbers");
  return Vector3D(a[0].get<double>(), a[1].get<double>(), a[2].get<double>());
}

double readNumber(const json &obj, const char *key, double fallback) {
  if (!has(obj, key)) return fallback;
  if (!obj[key].is_number()) throw std::runtime_error(std::string("\"") + key + "\" must be a number");
  return obj[key].get<double>();
}

} // namespace

bool JelloScene::load(const std::string &path, std::string *error) {
  std::ifstream in(path);
  if (!in.good()) {
    *error = "cannot open " + path;
    return false;
  }
  try {
    json j;
    in >> j;
    checkKeys(j, "scene", {"description", "jello", "solver", "ground"});

    if (has(j, "jello")) {
      const json &b = j["jello"];
      checkKeys(b, "jello", {"size", "cells", "origin", "density"});
      size = readVec3(b, "size", size);
      origin = readVec3(b, "origin", origin);
      density = readNumber(b, "density", density);
      if (has(b, "cells")) {
        const json &c = b["cells"];
        if (!c.is_array() || c.size() != 3) throw std::runtime_error("\"cells\" must be [nx, ny, nz]");
        for (int i = 0; i < 3; i++) cells[i] = c[i].get<int>();
      }
      if (cells[0] < 1 || cells[1] < 1 || cells[2] < 1) throw std::runtime_error("\"cells\" must be >= 1");
      if (size.x <= 0 || size.y <= 0 || size.z <= 0) throw std::runtime_error("\"size\" must be > 0");
      if (density <= 0) throw std::runtime_error("\"density\" must be > 0");
    }

    if (has(j, "solver")) solver = j["solver"].get<std::string>();

    if (has(j, "ground")) {
      const json &g = j["ground"];
      if (g.is_null()) {
        hasGround = false;
      } else {
        checkKeys(g, "ground", {"y", "friction"});
        hasGround = true;
        groundY = readNumber(g, "y", groundY);
        groundFriction = readNumber(g, "friction", groundFriction);
      }
    }
  } catch (const std::exception &e) {
    *error = path + ": " + e.what();
    return false;
  }
  return true;
}

TetMesh JelloScene::buildMesh() const {
  TetMesh mesh;
  mesh.buildGrid(cells[0], cells[1], cells[2], size, origin);
  return mesh;
}

std::unique_ptr<SoftBody> JelloScene::buildBody(std::string *error) const {
  auto body = makeSoftBody(solver, buildMesh(), density);
  if (!body) *error = "unknown solver \"" + solver + "\" (available: static)";
  return body;
}

bool JelloScene::isJelloScene(const std::string &path) {
  std::ifstream in(path);
  if (!in.good()) return false;
  try {
    json j;
    in >> j;
    return j.is_object() && has(j, "jello");
  } catch (const std::exception &) {
    return false;
  }
}

} // namespace Jello
