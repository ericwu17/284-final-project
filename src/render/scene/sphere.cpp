#include "sphere.h"

#include <cmath>

#include "pathtracer/bsdf.h"
#include "util/sphere_drawing.h"

namespace CGL {
namespace SceneObjects {

bool Sphere::test(const Ray &r, double &t1, double &t2) const {
  // Solve |o + t*d - c|^2 = R^2, i.e. a*t^2 + b*t + c = 0.
  Vector3D oc = r.o - o;
  double a = dot(r.d, r.d);
  double b = 2.0 * dot(oc, r.d);
  double c = dot(oc, oc) - r2;

  double disc = b * b - 4.0 * a * c;
  if (disc < 0) return false;

  double sqrt_disc = sqrt(disc);
  t1 = (-b - sqrt_disc) / (2.0 * a);
  t2 = (-b + sqrt_disc) / (2.0 * a);
  return true;

}

// Returns the nearest root of the sphere test within [min_t, max_t].
static bool nearest_valid_t(double t1, double t2, const Ray &r, double &t) {
  if (t1 >= r.min_t && t1 <= r.max_t) {
    t = t1;
    return true;
  }
  if (t2 >= r.min_t && t2 <= r.max_t) {
    t = t2;
    return true;
  }
  return false;
}

bool Sphere::has_intersection(const Ray &r) const {

  // TODO (Part 1.4):
  // Implement ray - sphere intersection.
  // Note that you might want to use the the Sphere::test helper here.

  double t1, t2, t;
  if (!test(r, t1, t2) || !nearest_valid_t(t1, t2, r, t)) return false;

  r.max_t = t;
  return true;
}

bool Sphere::intersect(const Ray &r, Intersection *i) const {

  // TODO (Part 1.4):
  // Implement ray - sphere intersection.
  // Note again that you might want to use the the Sphere::test helper here.
  // When an intersection takes place, the Intersection data should be updated
  // correspondingly.

  double t1, t2, t;
  if (!test(r, t1, t2) || !nearest_valid_t(t1, t2, r, t)) return false;

  r.max_t = t;

  i->t = t;
  i->n = normal(r.at_time(t));
  i->ng = i->n;
  i->primitive = this;
  i->bsdf = get_bsdf();
  return true;
}

void Sphere::draw(const Color &c, float alpha) const {
  Misc::draw_sphere_opengl(o, r, c);
}

void Sphere::drawOutline(const Color &c, float alpha) const {
  // Misc::draw_sphere_opengl(o, r, c);
}

} // namespace SceneObjects
} // namespace CGL
