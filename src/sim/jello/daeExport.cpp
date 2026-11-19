#include "daeExport.h"

#include <cstdio>
#include <fstream>
#include <sstream>
#include <vector>

namespace Jello {

namespace {

const char *PLACEHOLDERS[] = {"JELLO_FLOAT_COUNT", "JELLO_POS_COUNT", "JELLO_POSITIONS",
                              "JELLO_TRI_COUNT",   "JELLO_VCOUNT",    "JELLO_INDICES",
                              "PLATE_MATRIX"};

void replaceOnce(std::string &s, const std::string &name, const std::string &value) {
  const std::string key = "{{" + name + "}}";
  size_t at = s.find(key);
  if (at != std::string::npos) s.replace(at, key.size(), value);
}

// Removes whole lines from the one containing `begin` through the next line
// (after that one) containing `end`.
void removeLines(std::string &s, const std::string &begin, const std::string &end) {
  size_t b = s.find(begin);
  if (b == std::string::npos) return;
  size_t lineStart = s.rfind('\n', b);
  lineStart = lineStart == std::string::npos ? 0 : lineStart + 1;
  size_t beginLineEnd = s.find('\n', b);
  size_t e = beginLineEnd == std::string::npos ? std::string::npos : s.find(end, beginLineEnd);
  if (e == std::string::npos) return;
  size_t lineEnd = s.find('\n', e);
  lineEnd = lineEnd == std::string::npos ? s.size() : lineEnd + 1;
  s.erase(lineStart, lineEnd - lineStart);
}

} // namespace

bool DaeExporter::loadTemplate(const std::string &path, std::string *error) {
  std::ifstream in(path);
  if (!in) {
    *error = "cannot open template " + path;
    return false;
  }
  std::stringstream ss;
  ss << in.rdbuf();
  tmpl = ss.str();
  for (const char *name : PLACEHOLDERS) {
    const std::string key = std::string("{{") + name + "}}";
    size_t first = tmpl.find(key);
    if (first == std::string::npos || tmpl.find(key, first + 1) != std::string::npos) {
      *error = path + ": placeholder " + key + " must appear exactly once";
      return false;
    }
  }
  if (tmpl.find("PLATE_BEGIN") == std::string::npos || tmpl.find("PLATE_END") == std::string::npos) {
    *error = path + ": missing PLATE_BEGIN / PLATE_END markers";
    return false;
  }
  return true;
}

std::string DaeExporter::fill(const SoftBody &body, const KinematicPlate *plate, double t) const {
  const TetMesh &mesh = body.mesh();
  const std::vector<int> &surf = mesh.surfaceVerts;  // ascending

  // Mesh vertex -> compact surface index.
  std::vector<int> compact(mesh.numVerts(), -1);
  for (size_t i = 0; i < surf.size(); i++) compact[surf[i]] = (int)i;

  std::string positions, vcount, indices;
  positions.reserve(surf.size() * 30);
  char buf[96];
  for (size_t i = 0; i < surf.size(); i++) {
    const Vector3D &p = body.x[surf[i]];
    snprintf(buf, sizeof(buf), "%s%.6f %.6f %.6f", i ? " " : "", p.x, p.y, p.z);
    positions += buf;
  }
  indices.reserve(mesh.surfaceTris.size() * 15);
  for (size_t i = 0; i < mesh.surfaceTris.size(); i++) {
    const auto &tri = mesh.surfaceTris[i];
    vcount += i ? " 3" : "3";
    snprintf(buf, sizeof(buf), "%s%d %d %d", i ? " " : "", compact[tri[0]], compact[tri[1]],
             compact[tri[2]]);
    indices += buf;
  }

  std::string doc = tmpl;
  replaceOnce(doc, "JELLO_FLOAT_COUNT", std::to_string(3 * surf.size()));
  replaceOnce(doc, "JELLO_POS_COUNT", std::to_string(surf.size()));
  replaceOnce(doc, "JELLO_POSITIONS", positions);
  replaceOnce(doc, "JELLO_TRI_COUNT", std::to_string(mesh.surfaceTris.size()));
  replaceOnce(doc, "JELLO_VCOUNT", vcount);
  replaceOnce(doc, "JELLO_INDICES", indices);
  if (plate) {
    Vector3D c = plate->centerAt(t), s = plate->size();
    snprintf(buf, sizeof(buf), "%.6f 0 0 %.6f 0 %.6f 0 %.6f 0 0 %.6f %.6f 0 0 0 1", s.x, c.x, s.y,
             c.y, s.z, c.z);
    replaceOnce(doc, "PLATE_MATRIX", buf);
  } else {
    removeLines(doc, "PLATE_BEGIN", "PLATE_END");
  }
  return doc;
}

bool DaeExporter::writeFrame(const std::string &path, const SoftBody &body,
                             const KinematicPlate *plate, double t, std::string *error) const {
  const std::string doc = fill(body, plate, t);
  // A placeholder left in the file parses as garbage numbers (e.g. a broken
  // plate matrix), which corrupts the render without any loader error.
  size_t left = doc.find("{{");
  if (left != std::string::npos) {
    *error = "unfilled placeholder " + doc.substr(left, doc.find("}}", left) + 2 - left) + " in " + path;
    return false;
  }
  std::ofstream out(path);
  if (!out) {
    *error = "cannot write " + path;
    return false;
  }
  out << doc;
  if (!out) {
    *error = "error writing " + path;
    return false;
  }
  return true;
}

} // namespace Jello
