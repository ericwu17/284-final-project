#include <nanogui/nanogui.h>

#include "../clothMesh.h"
#include "../misc/sphere_drawing.h"
#include "sphere.h"

using namespace nanogui;
using namespace CGL;

void Sphere::collide(PointMass &pm) {
  // TODO (Part 3): Handle collisions with spheres.
  Vector3D from_origin = pm.position - origin;
  double distance = from_origin.norm();

  // No collision if the point mass is outside the sphere.
  if (distance > radius) {
    return;
  }

  // Avoid dividing by zero if the point mass sits exactly at the center.
  if (distance == 0) {
    return;
  }

  // The point on the sphere's surface along the ray from the origin through
  // the point mass's current position.
  Vector3D direction = from_origin / distance;
  Vector3D tangent_point = origin + radius * direction;

  // Move from last_position toward the tangent point, scaled down by friction.
  Vector3D correction = tangent_point - pm.last_position;
  pm.position = pm.last_position + (1.0 - friction) * correction;
}

void Sphere::render(GLShader &shader) {
  // We decrease the radius here so flat triangles don't behave strangely
  // and intersect with the sphere when rendered
  m_sphere_mesh.draw_sphere(shader, origin, radius * 0.92);
}
