// Minimal libumbreon consumer: build a Scene in code, render it, and write a
// PPM. This is the integration starting point for CueMol -- it uses ONLY the
// public umbreon API (no .pov parser, no bench harness, no image library).
//
// Build (against an installed umbreon, see examples/CMakeLists.txt):
//   cmake -S examples -B build-ex -DCMAKE_PREFIX_PATH="<umbreon-prefix>;/opt/homebrew"
//   cmake --build build-ex && ./build-ex/minimal_render
#include <cstdio>
#include <vector>

#include <umbreon/postprocess/image_ops.hpp>  // srgbEncode8
#include <umbreon/umbreon.hpp>

int main() {
  umbreon::Scene scene;

  // --- geometry: one quad (two de-indexed triangles) in z=0, facing +Z --------
  // umbreon meshes are de-indexed: triangle i uses vertices [3i, 3i+1, 3i+2],
  // and every corner carries its own position, normal and rgb+opacity color.
  umbreon::Mesh& mesh = scene.mesh;
  const umbreon::Vec3 corners[6] = {{-1, -1, 0}, {1, -1, 0}, {1, 1, 0},
                                    {-1, -1, 0}, {1, 1, 0},  {-1, 1, 0}};
  for (const umbreon::Vec3& c : corners) {
    mesh.positions.push_back(c);
    mesh.normals.push_back({0.0f, 0.0f, 1.0f});
    mesh.colors.push_back({0.2f, 0.4f, 0.8f, 1.0f});  // rgb + opacity(=alpha)
  }
  mesh.material.ambient = 0.2f;  // POV finish ambient
  mesh.material.diffuse = 0.8f;  // POV finish diffuse

  // --- camera: orthographic, looking down -Z at the quad ----------------------
  scene.camera.position = {0.0f, 0.0f, 5.0f};
  scene.camera.direction = {0.0f, 0.0f, -1.0f};
  scene.camera.up = {0.0f, 1.0f, 0.0f};
  scene.camera.orthographic = true;
  scene.camera.height = 2.5f;  // image-plane height in world units

  // --- one head-on white distant light ----------------------------------------
  umbreon::DistantLight light;
  light.direction = {0.0f, 0.0f, -1.0f};  // direction the light travels
  light.color = {1.0f, 1.0f, 1.0f};
  light.intensity = 1.0f;
  scene.lights.push_back(light);

  scene.background = {0.0f, 0.0f, 0.0f};
  scene.ambientColor = {1.0f, 1.0f, 1.0f};

  // --- options ----------------------------------------------------------------
  umbreon::RenderOptions opt;
  opt.width = 256;
  opt.height = 256;
  opt.supersample = 2;   // render 2x and box-average down (antialiasing)
  opt.aoSamples = 16;    // ambient occlusion rays per hit (0 = AO off)
  opt.shadows = true;    // cast shadows from the lights

  // --- render -----------------------------------------------------------------
  umbreon::FrameResult frame = umbreon::render(scene, opt);
  std::printf("rendered %dx%d in %.3f s (%zu effective triangles)\n",
              frame.width, frame.height, frame.renderSeconds,
              frame.effectiveTriangles);

  // frame.color is linear HDR float RGBA, top-left origin. Encode to 8-bit sRGB
  // and write a binary PPM (a real consumer hands the bytes to its own pipeline).
  std::vector<std::uint8_t> rgb = umbreon::srgbEncode8(frame, 3);
  if (FILE* f = std::fopen("minimal.ppm", "wb")) {
    std::fprintf(f, "P6\n%d %d\n255\n", frame.width, frame.height);
    std::fwrite(rgb.data(), 1, rgb.size(), f);
    std::fclose(f);
    std::printf("wrote minimal.ppm\n");
  }
  return 0;
}
