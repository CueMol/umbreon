// libumbreon INTERNAL header -- not installed, not part of the public API.
// Implementation detail; may change without notice. Do not include downstream.
// Embree geometry / ray masks.
//
// Object-space edge lines (generateObjectSpaceEdges) carry kEdgeGeomMask;
// every other geometry keeps Embree's default all-ones mask. Primary rays use
// all ones and see the lines; secondary rays (shadow, AO, GI, reflection) use
// kSecondaryRayMask and skip them, so a line neither casts a shadow nor
// occludes AO / GI. kNoEdgeRayMask is the same value under the name the
// outline-mode back probe uses (a primary-ray query that ignores the lines).
#pragma once

namespace umbreon {
namespace detail {

constexpr unsigned int kEdgeGeomMask = 0x2u;
constexpr unsigned int kSecondaryRayMask = ~kEdgeGeomMask;
constexpr unsigned int kNoEdgeRayMask = ~kEdgeGeomMask;

}  // namespace detail
}  // namespace umbreon
