#include "bsdf.h"

#include <algorithm>
#include <iostream>
#include <utility>

#include "application/visual_debugger.h"
#include "util/random_util.h"

using std::max;
using std::min;
using std::swap;

namespace CGL {

namespace {

// Smallest |cos| we divide by. Delta BSDFs return value / |cos(wi)| and the
// integrator multiplies by |cos(wi)| again, so this only guards against 0/0
// for an exactly grazing direction.
const double MIN_ABS_COS = 1e-12;

// Exact Fresnel reflectance for unpolarized light at a dielectric interface
// (average of the s and p terms), going from index etaI to etaT, with
// cosI = |cos| of the incident angle and cosT = |cos| of the transmitted one.
double fresnelDielectric(double cosI, double cosT, double etaI, double etaT) {
  double rs = (etaI * cosI - etaT * cosT) / (etaI * cosI + etaT * cosT);
  double rp = (etaT * cosI - etaI * cosT) / (etaT * cosI + etaI * cosT);
  return 0.5 * (rs * rs + rp * rp);
}

} // namespace

// Mirror BSDF //

Vector3D MirrorBSDF::f(const Vector3D wo, const Vector3D wi) {
  return Vector3D();
}

Vector3D MirrorBSDF::sample_f(const Vector3D wo, Vector3D* wi, double* pdf) {
  // Perfect specular reflection: the only direction is the mirror one, so it
  // is "sampled" with probability 1. Dividing by |cos| cancels the cosine the
  // integrator applies, leaving L_out = reflectance * L_in.
  reflect(wo, wi);
  double c = abs_cos_theta(*wi);
  if (c < MIN_ABS_COS) {
    *pdf = 0;
    return Vector3D();
  }
  *pdf = 1;
  return reflectance / c;
}

void MirrorBSDF::render_debugger_node()
{
  if (ImGui::TreeNode(this, "Mirror BSDF"))
  {
    DragDouble3("Reflectance", &reflectance[0], 0.005);
    ImGui::TreePop();
  }
}

// Microfacet BSDF //

double MicrofacetBSDF::G(const Vector3D wo, const Vector3D wi) {
  return 1.0 / (1.0 + Lambda(wi) + Lambda(wo));
}

double MicrofacetBSDF::D(const Vector3D h) {
  // TODO: proj3-2, part 3
  // Compute Beckmann normal distribution function (NDF) here.
  // You will need the roughness alpha.
  
  return 1.0;
}

Vector3D MicrofacetBSDF::F(const Vector3D wi) {
  // TODO: proj3-2, part 3
  // Compute Fresnel term for reflection on dielectric-conductor interface.
  // You will need both eta and etaK, both of which are Vector3D.

  double cosTheta = cos_theta(wi);
  
  return Vector3D();
}

Vector3D MicrofacetBSDF::f(const Vector3D wo, const Vector3D wi) {
  // TODO: proj3-2, part 3
  // Implement microfacet model here.

  return Vector3D();
}

Vector3D MicrofacetBSDF::sample_f(const Vector3D wo, Vector3D* wi, double* pdf) {
  // TODO: proj3-2, part 3
  // *Importance* sample Beckmann normal distribution function (NDF) here.
  // Note: You should fill in the sampled direction *wi and the corresponding *pdf,
  //       and return the sampled BRDF value.



  *wi = cosineHemisphereSampler.get_sample(pdf);

  return MicrofacetBSDF::f(wo, *wi);
}

void MicrofacetBSDF::render_debugger_node()
{
  if (ImGui::TreeNode(this, "Micofacet BSDF"))
  {
    DragDouble3("eta", &eta[0], 0.005);
    DragDouble3("K", &k[0], 0.005);
    DragDouble("alpha", &alpha, 0.005);
    ImGui::TreePop();
  }
}

// Refraction BSDF //

Vector3D RefractionBSDF::f(const Vector3D wo, const Vector3D wi) {
  return Vector3D();
}

Vector3D RefractionBSDF::sample_f(const Vector3D wo, Vector3D* wi, double* pdf) {
  // Transmission only (no Fresnel reflection). Total internal reflection
  // has no transmitted direction, so the path ends there.
  if (!refract(wo, wi, ior)) {
    *pdf = 0;
    return Vector3D();
  }
  double c = abs_cos_theta(*wi);
  if (c < MIN_ABS_COS) {
    *pdf = 0;
    return Vector3D();
  }
  *pdf = 1;
  // Radiance is compressed into a smaller solid angle in the denser medium:
  // L_wo = (eta_wo / eta_wi)^2 L_wi. See GlassBSDF::sample_f.
  double eta = wo.z > 0 ? 1.0 / ior : ior;  // eta_wo / eta_wi
  return transmittance * (eta * eta) / c;
}

void RefractionBSDF::render_debugger_node()
{
  if (ImGui::TreeNode(this, "Refraction BSDF"))
  {
    DragDouble3("Transmittance", &transmittance[0], 0.005);
    DragDouble("ior", &ior, 0.005);
    ImGui::TreePop();
  }
}

// Glass BSDF //

Vector3D GlassBSDF::f(const Vector3D wo, const Vector3D wi) {
  return Vector3D();
}

Vector3D GlassBSDF::sample_f(const Vector3D wo, Vector3D* wi, double* pdf) {
  // Smooth dielectric. wo is on the outside when wo.z > 0 (the normal points
  // out of the object). Fresnel: exact (unpolarized), not Schlick.
  //
  // With probability F the ray reflects (pdf F, value F * reflectance / |cos|),
  // otherwise it refracts (pdf 1 - F, value (1 - F) * transmittance * eta^2 /
  // |cos|), so each branch carries weight reflectance or transmittance * eta^2
  // after the integrator divides by the pdf and multiplies by |cos|.
  //
  // eta = eta_wo / eta_wi. Radiance follows L_wo = eta^2 * (1 - F) * L_wi
  // (PBRT 8.2, radiance transport). Through a closed object the entering and
  // leaving factors cancel, so this choice only matters for paths that start
  // or end inside the jello.
  const bool entering = wo.z > 0;
  const double etaI = entering ? 1.0 : ior;  // medium on the wo side
  const double etaT = entering ? ior : 1.0;  // medium on the other side
  const double eta = etaI / etaT;
  const double cosI = fabs(wo.z);

  double sin2T = eta * eta * std::max(0.0, 1.0 - cosI * cosI);
  if (sin2T >= 1.0) {
    // Total internal reflection: all of it is reflected.
    reflect(wo, wi);
    *pdf = 1;
    return reflectance / std::max(abs_cos_theta(*wi), MIN_ABS_COS);
  }
  double cosT = sqrt(1.0 - sin2T);
  double F = fresnelDielectric(cosI, cosT, etaI, etaT);

  if (coin_flip(F)) {
    reflect(wo, wi);
    *pdf = F;
    return F * reflectance / std::max(abs_cos_theta(*wi), MIN_ABS_COS);
  }
  refract(wo, wi, ior);
  *pdf = 1 - F;
  return (1 - F) * transmittance * (eta * eta) / std::max(abs_cos_theta(*wi), MIN_ABS_COS);
}

void GlassBSDF::render_debugger_node()
{
  if (ImGui::TreeNode(this, "Refraction BSDF"))
  {
    DragDouble3("Reflectance", &reflectance[0], 0.005);
    DragDouble3("Transmittance", &transmittance[0], 0.005);
    DragDouble("ior", &ior, 0.005);
    ImGui::TreePop();
  }
}

void BSDF::reflect(const Vector3D wo, Vector3D* wi) {

  // Mirror about the normal (0, 0, 1) in the local frame.
  *wi = Vector3D(-wo.x, -wo.y, wo.z);
}

bool BSDF::refract(const Vector3D wo, Vector3D* wi, double ior) {

  // Snell's law in the local frame (normal (0, 0, 1)). wo.z > 0 means wo is
  // outside (in vacuum) and the refracted wi goes inside, and vice versa.
  // Returns false on total internal reflection (no refracted direction).
  const bool entering = wo.z > 0;
  const double eta = entering ? 1.0 / ior : ior;  // eta_wo / eta_wi
  double cos2T = 1.0 - eta * eta * (1.0 - wo.z * wo.z);
  if (cos2T < 0) return false;
  double cosT = sqrt(cos2T);
  *wi = Vector3D(-eta * wo.x, -eta * wo.y, entering ? -cosT : cosT);
  return true;

}

} // namespace CGL
