#include <iostream>
#include <math.h>
#include <random>
#include <vector>

#include "cloth.h"
#include "collision/plane.h"
#include "collision/sphere.h"

using namespace std;

Cloth::Cloth(double width, double height, int num_width_points,
             int num_height_points, float thickness) {
  this->width = width;
  this->height = height;
  this->num_width_points = num_width_points;
  this->num_height_points = num_height_points;
  this->thickness = thickness;

  buildGrid();
  buildClothMesh();
}

Cloth::~Cloth() {
  point_masses.clear();
  springs.clear();

  if (clothMesh) {
    delete clothMesh;
  }
}

void Cloth::buildGrid() {
  const int W = num_width_points;
  const int H = num_height_points;

  point_masses.clear();
  springs.clear();
  // Reserve up front so the PointMass pointers held by springs stay valid.
  point_masses.reserve(W * H);

  const double dx = W > 1 ? width / (W - 1) : 0.0;
  const double dy = H > 1 ? height / (H - 1) : 0.0;

  // Point masses, stored in row-major order (x varies fastest).
  for (int y = 0; y < H; y++) {
    for (int x = 0; x < W; x++) {
      Vector3D pos;
      if (orientation == HORIZONTAL) {
        pos = Vector3D(x * dx, 1.0, y * dy);
      } else {
        double z = ((double)rand() / RAND_MAX) * 2.0 / 1000.0 - 1.0 / 1000.0;
        pos = Vector3D(x * dx, y * dy, z);
      }

      bool is_pinned = false;
      for (const vector<int> &p : pinned) {
        if (p.size() >= 2 && p[0] == x && p[1] == y) {
          is_pinned = true;
          break;
        }
      }

      point_masses.emplace_back(pos, is_pinned);
    }
  }

  // Springs. Each mass only links to neighbors to its left / above, so every
  // pair of masses gets at most one spring of each type.
  auto at = [&](int x, int y) { return &point_masses[y * W + x]; };

  for (int y = 0; y < H; y++) {
    for (int x = 0; x < W; x++) {
      PointMass *pm = at(x, y);

      // Structural: left and above.
      if (x >= 1) springs.emplace_back(pm, at(x - 1, y), STRUCTURAL);
      if (y >= 1) springs.emplace_back(pm, at(x, y - 1), STRUCTURAL);

      // Shearing: diagonal upper-left and upper-right.
      if (x >= 1 && y >= 1) springs.emplace_back(pm, at(x - 1, y - 1), SHEARING);
      if (x + 1 < W && y >= 1) springs.emplace_back(pm, at(x + 1, y - 1), SHEARING);

      // Bending: two to the left and two above.
      if (x >= 2) springs.emplace_back(pm, at(x - 2, y), BENDING);
      if (y >= 2) springs.emplace_back(pm, at(x, y - 2), BENDING);
    }
  }
}

void Cloth::simulate(double frames_per_sec, double simulation_steps, ClothParameters *cp,
                     vector<Vector3D> external_accelerations,
                     vector<CollisionObject *> *collision_objects) {
  double mass = width * height * cp->density / num_width_points / num_height_points;
  double delta_t = 1.0f / frames_per_sec / simulation_steps;

  // TODO (Part 2): Compute total force acting on each point mass.
  Vector3D external_force(0, 0, 0);
  for (const Vector3D &a : external_accelerations) {
    external_force += mass * a;
  }
  for (PointMass &pm : point_masses) {
    pm.forces = external_force;
  }

  for (Spring &s : springs) {
    double k;
    switch (s.spring_type) {
      case STRUCTURAL:
        if (!cp->enable_structural_constraints) continue;
        k = cp->ks;
        break;
      case SHEARING:
        if (!cp->enable_shearing_constraints) continue;
        k = cp->ks;
        break;
      case BENDING:
        if (!cp->enable_bending_constraints) continue;
        k = cp->ks * 0.2;
        break;
      default:
        continue;
    }
    Vector3D d = s.pm_b->position - s.pm_a->position;
    double len = d.norm();
    if (len == 0) continue;
    // Positive magnitude when stretched: pulls a toward b and b toward a.
    Vector3D f = k * (len - s.rest_length) * (d / len);
    s.pm_a->forces += f;
    s.pm_b->forces -= f;
  }

  // TODO (Part 2): Use Verlet integration to compute new point mass positions
  double d = cp->damping / 100.0;
  for (PointMass &pm : point_masses) {
    if (pm.pinned) continue;
    Vector3D x_t = pm.position;
    Vector3D a_t = pm.forces / mass;
    pm.position = x_t + (1.0 - d) * (x_t - pm.last_position) + a_t * delta_t * delta_t;
    pm.last_position = x_t;
  }

  // TODO (Part 4): Handle self-collisions.
  build_spatial_map();
  for (PointMass &pm : point_masses) {
    self_collide(pm, simulation_steps);
  }


  // TODO (Part 3): Handle collisions with other primitives.
  for (PointMass &pm : point_masses) {
    for (CollisionObject *object : *collision_objects) {
      object->collide(pm);
    }
  }


  // TODO (Part 2): Constrain the changes to be such that the spring does not change
  // in length more than 10% per timestep [Provot 1995].
  for (Spring &s : springs) {
    PointMass *a = s.pm_a;
    PointMass *b = s.pm_b;
    if (a->pinned && b->pinned) continue;

    Vector3D dir = b->position - a->position;
    double len = dir.norm();
    double max_len = 1.1 * s.rest_length;
    if (len <= max_len || len == 0) continue;

    dir /= len;
    double excess = len - max_len;
    if (a->pinned) {
      b->position -= excess * dir;
    } else if (b->pinned) {
      a->position += excess * dir;
    } else {
      a->position += 0.5 * excess * dir;
      b->position -= 0.5 * excess * dir;
    }
  }
}

void Cloth::build_spatial_map() {
  for (const auto &entry : map) {
    delete(entry.second);
  }
  map.clear();

  // TODO (Part 4): Build a spatial map out of all of the point masses.
  for (PointMass &pm : point_masses) {
    float key = hash_position(pm.position);

    // Create the bucket for this box the first time we see it.
    if (map.find(key) == map.end()) {
      map[key] = new vector<PointMass *>();
    }

    map[key]->push_back(&pm);
  }
}

void Cloth::self_collide(PointMass &pm, double simulation_steps) {
  // TODO (Part 4): Handle self-collision for a given point mass.

  // Pinned point masses never move.
  if (pm.pinned) {
    return;
  }

  float key = hash_position(pm.position);
  if (map.find(key) == map.end()) {
    return;
  }
  vector<PointMass *> *candidates = map[key];

  double min_distance = 2.0 * thickness;
  Vector3D total_correction(0, 0, 0);
  int num_corrections = 0;

  for (PointMass *other : *candidates) {
    // Don't collide a point mass with itself.
    if (other == &pm) {
      continue;
    }

    Vector3D from_other = pm.position - other->position;
    double distance = from_other.norm();

    // Only correct pairs that are too close. Skip the case where the two
    // masses are exactly on top of each other, since there is no direction to
    // push in.
    if (distance >= min_distance || distance == 0) {
      continue;
    }

    // Move pm directly away from the other mass so they end up exactly
    // 2 * thickness apart.
    Vector3D direction = from_other / distance;
    Vector3D correction = (min_distance - distance) * direction;

    total_correction += correction;
    num_corrections += 1;
  }

  if (num_corrections == 0) {
    return;
  }

  // Average the corrections, then scale down by the number of simulation steps
  // so the cloth doesn't jump suddenly.
  Vector3D average_correction = total_correction / num_corrections;
  pm.position += average_correction / simulation_steps;
}

float Cloth::hash_position(Vector3D pos) {
  // TODO (Part 4): Hash a 3D position into a unique float identifier that represents membership in some 3D box volume.

  // Size of each 3D box.
  double w = 3.0 * width / num_width_points;
  double h = 3.0 * height / num_height_points;
  double t = max(w, h);

  // Which box the position falls in along each axis. Subtracting the
  // remainder (fmod) snaps each coordinate down to the start of its box, and
  // dividing by the box size turns that into an integer box index.
  // fmod keeps the sign of its input, so for negative coordinates we step
  // back one more box so that all boxes are the same size.
  double x_remainder = fmod(pos.x, w);
  double y_remainder = fmod(pos.y, h);
  double z_remainder = fmod(pos.z, t);
  if (x_remainder < 0) x_remainder += w;
  if (y_remainder < 0) y_remainder += h;
  if (z_remainder < 0) z_remainder += t;

  int x_box = (int)round((pos.x - x_remainder) / w);
  int y_box = (int)round((pos.y - y_remainder) / h);
  int z_box = (int)round((pos.z - z_remainder) / t);

  // Combine the three box indices into one number, treating them like digits
  // in base 128. This is unique as long as the y and z indices stay within
  // (-64, 64), which holds for the scenes in this assignment. Even if two
  // boxes did share a key, self_collide still checks real distances, so the
  // result stays correct (it would just check a few extra candidates).
  const int base = 128;
  return (float)(x_box * base * base + y_box * base + z_box);
}

///////////////////////////////////////////////////////
/// YOU DO NOT NEED TO REFER TO ANY CODE BELOW THIS ///
///////////////////////////////////////////////////////

void Cloth::reset() {
  PointMass *pm = &point_masses[0];
  for (int i = 0; i < point_masses.size(); i++) {
    pm->position = pm->start_position;
    pm->last_position = pm->start_position;
    pm++;
  }
}

void Cloth::buildClothMesh() {
  if (point_masses.size() == 0) return;

  ClothMesh *clothMesh = new ClothMesh();
  vector<Triangle *> triangles;

  // Create vector of triangles
  for (int y = 0; y < num_height_points - 1; y++) {
    for (int x = 0; x < num_width_points - 1; x++) {
      PointMass *pm = &point_masses[y * num_width_points + x];
      // Get neighboring point masses:
      /*                      *
       * pm_A -------- pm_B   *
       *             /        *
       *  |         /   |     *
       *  |        /    |     *
       *  |       /     |     *
       *  |      /      |     *
       *  |     /       |     *
       *  |    /        |     *
       *      /               *
       * pm_C -------- pm_D   *
       *                      *
       */
      
      float u_min = x;
      u_min /= num_width_points - 1;
      float u_max = x + 1;
      u_max /= num_width_points - 1;
      float v_min = y;
      v_min /= num_height_points - 1;
      float v_max = y + 1;
      v_max /= num_height_points - 1;
      
      PointMass *pm_A = pm                       ;
      PointMass *pm_B = pm                    + 1;
      PointMass *pm_C = pm + num_width_points    ;
      PointMass *pm_D = pm + num_width_points + 1;
      
      Vector3D uv_A = Vector3D(u_min, v_min, 0);
      Vector3D uv_B = Vector3D(u_max, v_min, 0);
      Vector3D uv_C = Vector3D(u_min, v_max, 0);
      Vector3D uv_D = Vector3D(u_max, v_max, 0);
      
      
      // Both triangles defined by vertices in counter-clockwise orientation
      triangles.push_back(new Triangle(pm_A, pm_C, pm_B, 
                                       uv_A, uv_C, uv_B));
      triangles.push_back(new Triangle(pm_B, pm_C, pm_D, 
                                       uv_B, uv_C, uv_D));
    }
  }

  // For each triangle in row-order, create 3 edges and 3 internal halfedges
  for (int i = 0; i < triangles.size(); i++) {
    Triangle *t = triangles[i];

    // Allocate new halfedges on heap
    Halfedge *h1 = new Halfedge();
    Halfedge *h2 = new Halfedge();
    Halfedge *h3 = new Halfedge();

    // Allocate new edges on heap
    Edge *e1 = new Edge();
    Edge *e2 = new Edge();
    Edge *e3 = new Edge();

    // Assign a halfedge pointer to the triangle
    t->halfedge = h1;

    // Assign halfedge pointers to point masses
    t->pm1->halfedge = h1;
    t->pm2->halfedge = h2;
    t->pm3->halfedge = h3;

    // Update all halfedge pointers
    h1->edge = e1;
    h1->next = h2;
    h1->pm = t->pm1;
    h1->triangle = t;

    h2->edge = e2;
    h2->next = h3;
    h2->pm = t->pm2;
    h2->triangle = t;

    h3->edge = e3;
    h3->next = h1;
    h3->pm = t->pm3;
    h3->triangle = t;
  }

  // Go back through the cloth mesh and link triangles together using halfedge
  // twin pointers

  // Convenient variables for math
  int num_height_tris = (num_height_points - 1) * 2;
  int num_width_tris = (num_width_points - 1) * 2;

  bool topLeft = true;
  for (int i = 0; i < triangles.size(); i++) {
    Triangle *t = triangles[i];

    if (topLeft) {
      // Get left triangle, if it exists
      if (i % num_width_tris != 0) { // Not a left-most triangle
        Triangle *temp = triangles[i - 1];
        t->pm1->halfedge->twin = temp->pm3->halfedge;
      } else {
        t->pm1->halfedge->twin = nullptr;
      }

      // Get triangle above, if it exists
      if (i >= num_width_tris) { // Not a top-most triangle
        Triangle *temp = triangles[i - num_width_tris + 1];
        t->pm3->halfedge->twin = temp->pm2->halfedge;
      } else {
        t->pm3->halfedge->twin = nullptr;
      }

      // Get triangle to bottom right; guaranteed to exist
      Triangle *temp = triangles[i + 1];
      t->pm2->halfedge->twin = temp->pm1->halfedge;
    } else {
      // Get right triangle, if it exists
      if (i % num_width_tris != num_width_tris - 1) { // Not a right-most triangle
        Triangle *temp = triangles[i + 1];
        t->pm3->halfedge->twin = temp->pm1->halfedge;
      } else {
        t->pm3->halfedge->twin = nullptr;
      }

      // Get triangle below, if it exists
      if (i + num_width_tris - 1 < 1.0f * num_width_tris * num_height_tris / 2.0f) { // Not a bottom-most triangle
        Triangle *temp = triangles[i + num_width_tris - 1];
        t->pm2->halfedge->twin = temp->pm3->halfedge;
      } else {
        t->pm2->halfedge->twin = nullptr;
      }

      // Get triangle to top left; guaranteed to exist
      Triangle *temp = triangles[i - 1];
      t->pm1->halfedge->twin = temp->pm2->halfedge;
    }

    topLeft = !topLeft;
  }

  clothMesh->triangles = triangles;
  this->clothMesh = clothMesh;
}
