// Unit tests for the surface irradiance cache (detail). Exercises the
// deterministic record placement directly on a hand-built first-hit G-buffer,
// independent of the full render pipeline.
#include <cstddef>
#include <cstdint>
#include <vector>

#include "render/irradiance_cache.hpp"
#include "render/render_types.hpp"
#include "test_util.hpp"

namespace {

// Build a WxH first-hit G-buffer where every pixel is a mesh hit at a regular
// world position p = {px*step, py*step, 0}, all in section 0, normal +z.
umbreon::FrameResult makeGrid(int W, int H, float step) {
  umbreon::FrameResult res;
  res.width = W;
  res.height = H;
  const std::size_t n = static_cast<std::size_t>(W) * H;
  res.position.assign(n * 3, 0.0f);
  res.normal.assign(n * 3, 0.0f);
  res.componentId.assign(n, 0xFFFFFFFFu);
  for (int py = 0; py < H; ++py)
    for (int px = 0; px < W; ++px) {
      const std::size_t pix = static_cast<std::size_t>(py) * W + px;
      res.position[pix * 3 + 0] = static_cast<float>(px) * step;
      res.position[pix * 3 + 1] = static_cast<float>(py) * step;
      res.position[pix * 3 + 2] = 0.0f;
      res.normal[pix * 3 + 2] = 1.0f;
      res.componentId[pix] = 0u;
    }
  return res;
}

}  // namespace

int main() {
  using namespace umbreon::detail;
  umbreon::test::Suite s("gi");

  // Placement is deterministic: same G-buffer => bit-identical record set.
  {
    umbreon::FrameResult g = makeGrid(10, 10, 0.1f);
    std::vector<IrradianceRecord> a = placeRecordsVoxel(g, 0.05f);
    std::vector<IrradianceRecord> b = placeRecordsVoxel(g, 0.05f);
    bool same = a.size() == b.size();
    for (std::size_t i = 0; same && i < a.size(); ++i)
      same = a[i].position.x == b[i].position.x &&
             a[i].position.y == b[i].position.y &&
             a[i].position.z == b[i].position.z &&
             a[i].componentId == b[i].componentId;
    s.check("placement: two runs produce identical records", same);
    // step 0.1 with spacing 0.05 => every pixel lands in its own voxel.
    s.check("placement: fine spacing seeds one record per mesh pixel",
            a.size() == 100);
  }

  // Record count is monotone in spacing (coarser voxels => fewer records).
  {
    umbreon::FrameResult g = makeGrid(10, 10, 0.1f);
    const std::size_t fine = placeRecordsVoxel(g, 0.05f).size();
    const std::size_t mid = placeRecordsVoxel(g, 0.2f).size();
    const std::size_t coarse = placeRecordsVoxel(g, 1.0f).size();
    s.check("placement: coarser spacing yields fewer records",
            fine > mid && mid > coarse);
    s.check("placement: very coarse spacing collapses to one voxel",
            coarse == 1);
  }

  // Non-mesh pixels (sentinel componentId) seed no records: GI is mesh-only.
  {
    umbreon::FrameResult g = makeGrid(4, 4, 1.0f);
    for (std::size_t i = 0; i < g.componentId.size(); ++i)
      g.componentId[i] = 0xFFFFFFFFu;  // mark everything background
    g.componentId[5] = 0u;             // leave a single mesh pixel
    std::vector<IrradianceRecord> r = placeRecordsVoxel(g, 0.5f);
    s.check("placement: only mesh pixels seed records", r.size() == 1);
    s.check("placement: seeded record keeps the mesh section id",
            !r.empty() && r[0].componentId == 0u);
  }

  return s.report();
}
