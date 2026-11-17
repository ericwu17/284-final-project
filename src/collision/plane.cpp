#include "iostream"
#include <nanogui/nanogui.h>

#include "../clothMesh.h"
#include "../clothSimulator.h"
#include "plane.h"

using namespace std;
using namespace CGL;

#define SURFACE_OFFSET 0.0001

void Plane::collide(PointMass &pm) {
  // TODO (Part 3): Handle collisions with planes.

  // Signed distance from the plane for the current and previous positions.
  // Positive means on the side the normal points to, negative the other side.
  double current_distance = dot(pm.position - point, normal);
  double last_distance = dot(pm.last_position - point, normal);

  // Which side of the plane each position is on.
  bool current_on_positive_side = current_distance >= 0;
  bool last_on_positive_side = last_distance >= 0;

  // No collision if the point mass stayed on the same side of the plane.
  if (current_on_positive_side == last_on_positive_side) {
    return;
  }

  // Project the current position straight onto the plane.
  Vector3D tangent_point = pm.position - current_distance * normal;

  // Push the target point slightly off the plane, back onto the side the
  // point mass started from.
  Vector3D offset_direction = last_on_positive_side ? normal : -normal;
  Vector3D target_point = tangent_point + SURFACE_OFFSET * offset_direction;

  // Move from last_position toward the target point, scaled down by friction.
  Vector3D correction = target_point - pm.last_position;
  pm.position = pm.last_position + (1.0 - friction) * correction;
}

void Plane::render(GLShader &shader) {
  nanogui::Color color(0.7f, 0.7f, 0.7f, 1.0f);

  Vector3f sPoint(point.x, point.y, point.z);
  Vector3f sNormal(normal.x, normal.y, normal.z);
  Vector3f sParallel(normal.y - normal.z, normal.z - normal.x,
                     normal.x - normal.y);
  sParallel.normalize();
  Vector3f sCross = sNormal.cross(sParallel);

  MatrixXf positions(3, 4);
  MatrixXf normals(3, 4);

  positions.col(0) << sPoint + 2 * (sCross + sParallel);
  positions.col(1) << sPoint + 2 * (sCross - sParallel);
  positions.col(2) << sPoint + 2 * (-sCross + sParallel);
  positions.col(3) << sPoint + 2 * (-sCross - sParallel);

  normals.col(0) << sNormal;
  normals.col(1) << sNormal;
  normals.col(2) << sNormal;
  normals.col(3) << sNormal;

  if (shader.uniform("u_color", false) != -1) {
    shader.setUniform("u_color", color);
  }
  shader.uploadAttrib("in_position", positions);
  if (shader.attrib("in_normal", false) != -1) {
    shader.uploadAttrib("in_normal", normals);
  }

  shader.drawArray(GL_TRIANGLE_STRIP, 0, 4);
}
