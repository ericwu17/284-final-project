#include "jelloScene.h"

#include <fstream>
#include <set>
#include <sstream>

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

bool readBool(const json &obj, const char *key, bool fallback) {
  if (!has(obj, key)) return fallback;
  if (!obj[key].is_boolean()) throw std::runtime_error(std::string("\"") + key + "\" must be true or false");
  return obj[key].get<bool>();
}

// Sets j["a"]["b"]... = value for the dotted key "a.b...".
void applyOverride(json &j, const std::string &dotted, const std::string &value) {
  json parsed;
  try {
    parsed = json::parse(value);
  } catch (const std::exception &) {
    parsed = value;  // not JSON: treat as a string
  }
  json *node = &j;
  std::stringstream ss(dotted);
  std::string part;
  std::vector<std::string> parts;
  while (std::getline(ss, part, '.')) parts.push_back(part);
  if (parts.empty()) throw std::runtime_error("empty override key");
  for (size_t i = 0; i + 1 < parts.size(); i++) {
    if (!node->is_object() && !node->is_null())
      throw std::runtime_error("override \"" + dotted + "\": \"" + parts[i] + "\" is not an object");
    node = &(*node)[parts[i]];
  }
  (*node)[parts.back()] = parsed;
}

} // namespace

bool JelloScene::load(const std::string &path, std::string *error,
                      const std::vector<std::pair<std::string, std::string>> &overrides) {
  std::ifstream in(path);
  if (!in.good()) {
    *error = "cannot open " + path;
    return false;
  }
  try {
    json j;
    in >> j;
    for (const auto &o : overrides) applyOverride(j, o.first, o.second);
    checkKeys(j, "scene", {"description", "jello", "solver", "mass_spring", "xpbd", "gravity", "ground",
                           "camera"});

    if (has(j, "jello")) {
      const json &b = j["jello"];
      checkKeys(b, "jello", {"size", "cells", "origin", "density", "velocity"});
      velocity = readVec3(b, "velocity", velocity);
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
    gravity = readVec3(j, "gravity", gravity);

    if (has(j, "mass_spring")) {
      const json &m = j["mass_spring"];
      checkKeys(m, "mass_spring", {"ks", "kd", "substeps", "kv", "volume"});
      massSpring.ks = readNumber(m, "ks", massSpring.ks);
      massSpring.kd = readNumber(m, "kd", massSpring.kd);
      massSpring.substeps = (int)readNumber(m, "substeps", massSpring.substeps);
      massSpring.kv = readNumber(m, "kv", massSpring.kv);
      massSpring.volume = readBool(m, "volume", massSpring.volume);
      if (massSpring.substeps < 1) throw std::runtime_error("\"substeps\" must be >= 1");
    }

    if (has(j, "xpbd")) {
      const json &x = j["xpbd"];
      checkKeys(x, "xpbd", {"edge_compliance", "edge_damping", "substeps", "volume_compliance", "volume"});
      xpbd.edgeCompliance = readNumber(x, "edge_compliance", xpbd.edgeCompliance);
      xpbd.edgeDamping = readNumber(x, "edge_damping", xpbd.edgeDamping);
      xpbd.substeps = (int)readNumber(x, "substeps", xpbd.substeps);
      xpbd.volumeCompliance = readNumber(x, "volume_compliance", xpbd.volumeCompliance);
      xpbd.volume = readBool(x, "volume", xpbd.volume);
      if (xpbd.volumeCompliance < 0) throw std::runtime_error("\"volume_compliance\" must be >= 0");
      if (xpbd.substeps < 1) throw std::runtime_error("\"substeps\" must be >= 1");
      if (xpbd.edgeCompliance < 0) throw std::runtime_error("\"edge_compliance\" must be >= 0");
    }

    if (has(j, "ground")) {
      const json &g = j["ground"];
      if (g.is_null()) {
        hasGround = false;
      } else {
        checkKeys(g, "ground", {"y", "friction"});
        hasGround = true;
        groundY = readNumber(g, "y", groundY);
        groundFriction = readNumber(g, "friction", groundFriction);
        if (groundFriction < 0) throw std::runtime_error("\"friction\" must be >= 0");
      }
    }

    if (has(j, "camera")) {
      const json &c = j["camera"];
      checkKeys(c, "camera", {"target", "distance"});
      if (has(c, "target")) {
        hasCameraTarget = true;
        cameraTarget = readVec3(c, "target", cameraTarget);
      }
      cameraDistance = readNumber(c, "distance", cameraDistance);
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
  std::unique_ptr<SoftBody> body;
  if (solver == "static") body = std::make_unique<StaticBody>(buildMesh(), density);
  else if (solver == "mass_spring") body = std::make_unique<MassSpringBody>(buildMesh(), density, massSpring);
  else if (solver == "xpbd") body = std::make_unique<XPBDBody>(buildMesh(), density, xpbd);
  if (!body) {
    *error = "unknown solver \"" + solver + "\" (available: static, mass_spring, xpbd)";
    return nullptr;
  }
  if (solver != "static") {
    body->initialVelocity = velocity;
    body->reset();
  }
  return body;
}

std::vector<std::unique_ptr<Collider>> JelloScene::buildColliders() const {
  std::vector<std::unique_ptr<Collider>> out;
  if (hasGround) out.push_back(std::make_unique<GroundPlane>(groundY, groundFriction));
  return out;
}

std::string JelloScene::toJson() const {
  auto vec = [](const Vector3D &v) { return json::array({v.x, v.y, v.z}); };
  json j;
  j["jello"] = {{"size", vec(size)},
                {"cells", json::array({cells[0], cells[1], cells[2]})},
                {"origin", vec(origin)},
                {"density", density},
                {"velocity", vec(velocity)}};
  j["solver"] = solver;
  j["mass_spring"] = {{"ks", massSpring.ks}, {"kd", massSpring.kd},
                      {"substeps", massSpring.substeps}, {"kv", massSpring.kv},
                      {"volume", massSpring.volume}};
  j["xpbd"] = {{"edge_compliance", xpbd.edgeCompliance}, {"edge_damping", xpbd.edgeDamping},
               {"substeps", xpbd.substeps}, {"volume_compliance", xpbd.volumeCompliance},
               {"volume", xpbd.volume}};
  j["gravity"] = vec(gravity);
  if (hasGround) j["ground"] = {{"y", groundY}, {"friction", groundFriction}};
  else j["ground"] = nullptr;
  if (hasCameraTarget || cameraDistance > 0) {
    json cam = json::object();
    if (hasCameraTarget) cam["target"] = vec(cameraTarget);
    if (cameraDistance > 0) cam["distance"] = cameraDistance;
    j["camera"] = cam;
  }
  return j.dump(2);
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
