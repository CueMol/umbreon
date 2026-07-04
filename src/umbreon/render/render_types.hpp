// libumbreon PUBLIC API header (installed). Part of the supported public
// API surface; keep in sync with install(FILES) in CMakeLists.txt.
// Renderer-agnostic options and framebuffer result shared by the umbreon
// (Embree) renderer and the bench harness. Pure C++17, no
// rendering-library dependency.
//
// Umbrella header: the types live in three sub-headers, split by concern.
// Including this header keeps the whole historical surface available;
// consumers may also include a sub-header directly (all three are installed).
#pragma once

#include "render/edge_types.hpp"      // EdgeClass/EdgeStyle, Stroke/ObjectSpaceEdgeOptions
#include "render/frame_result.hpp"    // FrameResult, Pt1Timing, Pt1RayCounts
#include "render/render_options.hpp"  // RenderOptions
