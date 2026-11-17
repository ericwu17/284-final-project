#include "bvh.h"

#include "CGL/CGL.h"
#include "triangle.h"

#include <algorithm>
#include <iostream>
#include <stack>

using namespace std;

namespace CGL {
namespace SceneObjects {

BVHAccel::BVHAccel(const std::vector<Primitive *> &_primitives,
                   size_t max_leaf_size) {

  primitives = std::vector<Primitive *>(_primitives);
  root = construct_bvh(primitives.begin(), primitives.end(), max_leaf_size);
}

BVHAccel::~BVHAccel() {
  if (root)
    delete root;
  primitives.clear();
}

BBox BVHAccel::get_bbox() const { return root->bb; }

void BVHAccel::draw(BVHNode *node, const Color &c, float alpha) const {
  if (node->isLeaf()) {
    for (auto p = node->start; p != node->end; p++) {
      (*p)->draw(c, alpha);
    }
  } else {
    draw(node->l, c, alpha);
    draw(node->r, c, alpha);
  }
}

void BVHAccel::drawOutline(BVHNode *node, const Color &c, float alpha) const {
  if (node->isLeaf()) {
    for (auto p = node->start; p != node->end; p++) {
      (*p)->drawOutline(c, alpha);
    }
  } else {
    drawOutline(node->l, c, alpha);
    drawOutline(node->r, c, alpha);
  }
}

BVHNode *BVHAccel::construct_bvh(std::vector<Primitive *>::iterator start,
                                 std::vector<Primitive *>::iterator end,
                                 size_t max_leaf_size) {

  // Construct a BVH from the given vector of primitives and maximum leaf
  // size configuration. The starter code build a BVH aggregate with a
  // single leaf node (which is also the root) that encloses all the
  // primitives.


  BBox bbox;
  BBox centroid_box;

  for (auto p = start; p != end; p++) {
    BBox bb = (*p)->get_bbox();
    bbox.expand(bb);
    centroid_box.expand(bb.centroid());
  }

  BVHNode *node = new BVHNode(bbox);

  size_t count = end - start;
  if (count <= max_leaf_size) {
    node->start = start;
    node->end = end;
    return node;
  }

  // Split along the axis where the centroids are most spread out, at the
  // midpoint of the centroids' extent on that axis.
  const Vector3D &extent = centroid_box.extent;
  int axis = 0;
  if (extent.y > extent[axis]) axis = 1;
  if (extent.z > extent[axis]) axis = 2;
  double split = centroid_box.centroid()[axis];

  auto mid = std::partition(start, end, [axis, split](Primitive *p) {
    return p->get_bbox().centroid()[axis] < split;
  });

  // If every centroid landed on one side (e.g. they all coincide), the
  // recursion would never shrink; fall back to splitting at the median.
  if (mid == start || mid == end) {
    mid = start + count / 2;
    std::nth_element(start, mid, end, [axis](Primitive *a, Primitive *b) {
      return a->get_bbox().centroid()[axis] < b->get_bbox().centroid()[axis];
    });
  }

  node->l = construct_bvh(start, mid, max_leaf_size);
  node->r = construct_bvh(mid, end, max_leaf_size);
  return node;
}

bool BVHAccel::has_intersection(const Ray &ray, BVHNode *node) const {
  // Fill in the intersect function.
  // Take note that this function has a short-circuit that the
  // Intersection version cannot, since it returns as soon as it finds
  // a hit, it doesn't actually have to find the closest hit.

  double t0 = ray.min_t, t1 = ray.max_t;
  if (!node->bb.intersect(ray, t0, t1)) return false;

  if (node->isLeaf()) {
    for (auto p = node->start; p != node->end; p++) {
      total_isects++;
      if ((*p)->has_intersection(ray))
        return true;
    }
    return false;
  }

  return has_intersection(ray, node->l) || has_intersection(ray, node->r);
}

bool BVHAccel::intersect(const Ray &ray, Intersection *i, BVHNode *node) const {
  // Fill in the intersect function.

  // Primitives shrink ray.max_t on every hit, so boxes beyond the closest
  // hit found so far are culled by this test automatically.
  double t0 = ray.min_t, t1 = ray.max_t;
  if (!node->bb.intersect(ray, t0, t1)) return false;

  if (node->isLeaf()) {
    bool hit = false;
    for (auto p = node->start; p != node->end; p++) {
      total_isects++;
      hit = (*p)->intersect(ray, i) || hit;
    }
    return hit;
  }

  // Both children must be visited: a nearer hit may lie in either one.
  bool hit_l = intersect(ray, i, node->l);
  bool hit_r = intersect(ray, i, node->r);
  return hit_l || hit_r;
}

} // namespace SceneObjects
} // namespace CGL
