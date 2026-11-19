#include "pathtracer.h"

#include "scene/light.h"
#include "scene/sphere.h"
#include "scene/triangle.h"


using namespace CGL::SceneObjects;

namespace CGL {

PathTracer::PathTracer() {
  gridSampler = new UniformGridSampler2D();
  hemisphereSampler = new UniformHemisphereSampler3D();

  tm_gamma = 2.2f;
  tm_level = 1.0f;
  tm_key = 0.18;
  tm_wht = 5.0f;
}

PathTracer::~PathTracer() {
  delete gridSampler;
  delete hemisphereSampler;
}

void PathTracer::set_frame_size(size_t width, size_t height) {
  sampleBuffer.resize(width, height);
  sampleCountBuffer.resize(width * height);
}

void PathTracer::clear() {
  bvh = NULL;
  scene = NULL;
  camera = NULL;
  sampleBuffer.clear();
  sampleCountBuffer.clear();
  sampleBuffer.resize(0, 0);
  sampleCountBuffer.resize(0, 0);
}

void PathTracer::write_to_framebuffer(ImageBuffer &framebuffer, size_t x0,
                                      size_t y0, size_t x1, size_t y1) {
  sampleBuffer.toColor(framebuffer, x0, y0, x1, y1);
}

Vector3D
PathTracer::estimate_direct_lighting_hemisphere(const Ray &r,
                                                const Intersection &isect) {
  // Estimate the lighting from this intersection coming directly from a light.
  // For this function, sample uniformly in a hemisphere.

  // Note: When comparing Cornel Box (CBxxx.dae) results to importance sampling, you may find the "glow" around the light source is gone.
  // This is totally fine: the area lights in importance sampling has directionality, however in hemisphere sampling we don't model this behaviour.

  // make a coordinate system for a hit point
  // with N aligned with the Z direction.
  Matrix3x3 o2w;
  make_coord_space(o2w, isect.n);
  Matrix3x3 w2o = o2w.T();

  // w_out points towards the source of the ray (e.g.,
  // toward the camera if this is a primary ray)
  const Vector3D hit_p = r.o + r.d * isect.t;
  const Vector3D w_out = w2o * (-r.d);

  // This is the same number of total samples as
  // estimate_direct_lighting_importance (outside of delta lights). We keep the
  // same number of samples for clarity of comparison.
  int num_samples = scene->lights.size() * ns_area_light;
  Vector3D L_out;

  // Uniform hemisphere pdf is 1 / (2 pi)
  const double pdf = 1.0 / (2.0 * PI);
  for (int i = 0; i < num_samples; i++) {
    Vector3D w_in = hemisphereSampler->get_sample();
    Ray sample_ray(hit_p, o2w * w_in);
    sample_ray.min_t = EPS_F;

    Intersection light_isect;
    if (!bvh->intersect(sample_ray, &light_isect))
      continue;

    Vector3D L_in = light_isect.bsdf->get_emission();
    L_out += isect.bsdf->f(w_out, w_in) * L_in * w_in.z / pdf;
  }

  return L_out / num_samples;

}

Vector3D
PathTracer::estimate_direct_lighting_importance(const Ray &r,
                                                const Intersection &isect) {
  // Estimate the lighting from this intersection coming directly from a light.
  // To implement importance sampling, sample only from lights, not uniformly in
  // a hemisphere.

  // make a coordinate system for a hit point
  // with N aligned with the Z direction.
  Matrix3x3 o2w;
  make_coord_space(o2w, isect.n);
  Matrix3x3 w2o = o2w.T();

  // w_out points towards the source of the ray (e.g.,
  // toward the camera if this is a primary ray)
  const Vector3D hit_p = r.o + r.d * isect.t;
  const Vector3D w_out = w2o * (-r.d);
  Vector3D L_out;

  for (SceneLight *light : scene->lights) {
    // All samples from a point light are identical, so sample it once
    int ns = light->is_delta_light() ? 1 : ns_area_light;
    Vector3D L_light;

    for (int i = 0; i < ns; i++) {
      Vector3D wi;
      double dist_to_light, pdf;
      Vector3D L_in = light->sample_L(hit_p, &wi, &dist_to_light, &pdf);

      // Skip lights behind the surface
      Vector3D w_in = w2o * wi;
      if (w_in.z <= 0)
        continue;

      Ray shadow_ray(hit_p, wi);
      shadow_ray.min_t = EPS_F;
      shadow_ray.max_t = dist_to_light - EPS_F;

      Intersection shadow_isect;
      if (bvh->intersect(shadow_ray, &shadow_isect))
        continue;

      L_light += isect.bsdf->f(w_out, w_in) * L_in * w_in.z / pdf;
    }

    L_out += L_light / ns;
  }

  return L_out;
}

Vector3D PathTracer::zero_bounce_radiance(const Ray &r,
                                          const Intersection &isect) {
  // Returns the light that results from no bounces of light

  return isect.bsdf->get_emission();
}

Vector3D PathTracer::one_bounce_radiance(const Ray &r,
                                         const Intersection &isect) {
  // Returns either the direct illumination by hemisphere or importance sampling
  // depending on `direct_hemisphere_sample`

  if (direct_hemisphere_sample)
    return estimate_direct_lighting_hemisphere(r, isect);
  return estimate_direct_lighting_importance(r, isect);
}

// Beer-Lambert absorption along one ray segment (Stage R3). A segment that
// ends on the surface of an absorbing medium while heading out of it
// (along the outward geometric normal) ran inside that medium, so the light
// it carries is scaled by exp(-sigma_a * length). No inside/outside state is
// kept: this works because the jello surface is closed and outward-wound
// (checked by jellosim --selftest), and ng is outward on it (see Triangle).
static Vector3D segment_transmittance(const Ray &r, const Intersection &isect) {
  if (!isect.bsdf || !isect.bsdf->is_medium() || dot(r.d, isect.ng) <= 0)
    return Vector3D(1, 1, 1);
  Vector3D s = isect.bsdf->sigma_a();
  double len = isect.t * r.d.norm();
  return Vector3D(exp(-s.x * len), exp(-s.y * len), exp(-s.z * len));
}

// Smooth-normal guard. Interpolated normals near the block's edges can face
// the other way from the true surface for a ray, so the BSDF would pick the
// wrong side (and the wrong eta for refraction). When the shading normal and
// the geometric normal disagree about which side the ray came from, shade
// with the geometric normal.
static void guard_shading_normal(const Ray &r, Intersection *isect) {
  double s = dot(-r.d, isect->n), g = dot(-r.d, isect->ng);
  if ((s > 0) != (g > 0)) isect->n = isect->ng;
}

Vector3D PathTracer::at_least_one_bounce_radiance(const Ray &r,
                                                  const Intersection &isect) {
  Matrix3x3 o2w;
  make_coord_space(o2w, isect.n);
  Matrix3x3 w2o = o2w.T();

  Vector3D hit_p = r.o + r.d * isect.t;
  Vector3D w_out = w2o * (-r.d);

  Vector3D L_out(0, 0, 0);

  // Returns the one bounce radiance + radiance from extra bounces at this point.
  // Should be called recursively to simulate extra bounces.
  if (r.depth == 0)
    return L_out;

  // Delta BSDFs (mirror, glass) scatter into a single direction, which light
  // sampling never picks, so their direct lighting is zero. Light reaches them
  // only through the sampled bounce, below.
  const bool delta = isect.bsdf->is_delta();

  // Without accumulation, only the bounce at max_ray_depth contributes. A
  // bounce made here is bounce number (max_ray_depth - r.depth + 1).
  const bool count_this_bounce = isAccumBounces || r.depth == 1;
  if (!delta && count_this_bounce)
    L_out += one_bounce_radiance(r, isect);

  // A diffuse vertex at the last depth stops here; a delta vertex still has
  // to trace its one bounce to see what lies in the mirror/refracted direction.
  if (r.depth == 1 && !delta)
    return L_out;

  // Russian roulette, except on the first indirect bounce and on delta
  // bounces: a path through the jello needs several specular bounces in a row
  // (enter, internal reflections, exit), and killing 35% at each one would
  // make it mostly noise.
  double cont_prob = (delta || r.depth == max_ray_depth) ? 1.0 : 0.65;
  if (!coin_flip(cont_prob))
    return L_out;

  Vector3D w_in;
  double pdf;
  Vector3D f = isect.bsdf->sample_f(w_out, &w_in, &pdf);
  if (pdf <= 0)
    return L_out;

  Ray bounce_ray(hit_p, o2w * w_in, (int)r.depth - 1);
  bounce_ray.min_t = EPS_F;
  bounce_ray.specular_chain = r.specular_chain && delta;

  // Emission found after a delta bounce is added only on purely specular
  // paths (camera -> mirror/glass ... -> light): the light seen in or through
  // the jello. Paths that went diffuse first and then hit a light through a
  // delta bounce are caustics; a small light is reached that way so rarely
  // that they render as fireflies all over the walls, so they are dropped
  // (shadows behind glass are caustic-free, as in HW3).
  const bool add_emission = delta && count_this_bounce && r.specular_chain;

  Vector3D L_in;
  Intersection bounce_isect;
  bool hit = bvh->intersect(bounce_ray, &bounce_isect);
  if (hit) guard_shading_normal(bounce_ray, &bounce_isect);
  if (!hit) {
    // A delta bounce that escapes sees the environment directly.
    if (add_emission && envLight)
      L_in = envLight->sample_dir(bounce_ray);
  } else {
    // Emission seen through a delta bounce (the light reflected in or
    // refracted through the jello). After a non-delta bounce, emission is
    // already counted by light sampling at this vertex, so adding it there
    // would count it twice.
    if (add_emission)
      L_in += zero_bounce_radiance(bounce_ray, bounce_isect);
    L_in += at_least_one_bounce_radiance(bounce_ray, bounce_isect);
    L_in = L_in * segment_transmittance(bounce_ray, bounce_isect);
  }

  // |cos|: transmitted directions have w_in.z < 0.
  L_out += f * L_in * fabs(w_in.z) / pdf / cont_prob;

  return L_out;
}

Vector3D PathTracer::est_radiance_global_illumination(const Ray &r) {
  Intersection isect;
  Vector3D L_out;

  // You will extend this in assignment 3-2.
  // If no intersection occurs, we simply return black.
  // This changes if you implement hemispherical lighting for extra credit.

  // The following line of code returns a debug color depending
  // on whether ray intersection with triangles or spheres has
  // been implemented.
  //
  
  if (!bvh->intersect(r, &isect))
    return envLight ? envLight->sample_dir(r) : L_out;
  guard_shading_normal(r, &isect);

  // Emitted light is bounce 0, so only include it when accumulating
  // or when it is the requested bounce
  if (isAccumBounces || max_ray_depth == 0)
    L_out = zero_bounce_radiance(r, isect);

  L_out += at_least_one_bounce_radiance(r, isect);

  // A camera inside the jello would see through absorbing jello too.
  return L_out * segment_transmittance(r, isect);
}

void PathTracer::raytrace_pixel(size_t x, size_t y) {
  // Adaptive sampling: stop early once the pixel's 95% confidence interval
  // is within maxTolerance of its mean, checking every samplesPerBatch samples
  int max_samples = ns_aa;          // upper bound on samples to evaluate
  Vector2D origin = Vector2D(x, y); // bottom left corner of the pixel

  Vector3D radiance_sum(0, 0, 0);
  double s1 = 0, s2 = 0; // running sums of illuminance and its square
  int num_samples = 0;
  while (num_samples < max_samples) {
    // Jitter uniformly within the pixel, then normalize to [0,1]^2.
    Vector2D sample = origin + gridSampler->get_sample();
    Ray r = camera->generate_ray(sample.x / sampleBuffer.w,
                                 sample.y / sampleBuffer.h);
    r.depth = max_ray_depth;
    Vector3D radiance = est_radiance_global_illumination(r);
    radiance_sum += radiance;
    num_samples++;

    double illum = radiance.illum();
    s1 += illum;
    s2 += illum * illum;

    if (samplesPerBatch > 0 && num_samples > 1 &&
        num_samples % samplesPerBatch == 0) {
      double mean = s1 / num_samples;
      double var = (s2 - s1 * s1 / num_samples) / (num_samples - 1);
      double I = 1.96 * sqrt(std::max(var, 0.0) / num_samples);
      if (I <= maxTolerance * mean)
        break;
    }
  }

  sampleBuffer.update_pixel(radiance_sum / num_samples, x, y);
  sampleCountBuffer[x + y * sampleBuffer.w] = num_samples;


}

void PathTracer::autofocus(Vector2D loc) {
  Ray r = camera->generate_ray(loc.x / sampleBuffer.w, loc.y / sampleBuffer.h);
  Intersection isect;

  bvh->intersect(r, &isect);

  camera->focalDistance = isect.t;
}

} // namespace CGL
