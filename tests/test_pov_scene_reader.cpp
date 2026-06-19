// Tests for the focused POV-Ray scene reader: declare/conditional evaluation,
// orthographic / perspective camera, the SpecLighting / FlashLighting macros
// and the background color. A synthetic scene exercises the parser in
// isolation; the real CueMol .pov files (passed as path arguments) verify the
// reader against the exact reference render setup.
#include <cmath>
#include <cstddef>
#include <map>
#include <string>

#include "pov/pov_scene_reader.hpp"
#include "test_util.hpp"

namespace {

// The CueMol command-line constants for the reference render.
umbreon::PovParseOptions cuemolOptions(int w, int h) {
  umbreon::PovParseOptions o;
  o.imageWidth = w;
  o.imageHeight = h;
  o.predefined = {{"_stereo", 0.0},      {"_iod", 0.03},
                  {"_perspective", 0.0}, {"_shadow", 0.0},
                  {"_light_inten", 1.3}, {"_flash_frac", 0.6},
                  {"_amb_frac", 0.0}};
  return o;
}

bool approx(float a, float b, float eps) { return std::fabs(a - b) <= eps; }

bool dirApprox(const umbreon::Vec3& a, const umbreon::Vec3& b, float eps) {
  return approx(a.x, b.x, eps) && approx(a.y, b.y, eps) &&
         approx(a.z, b.z, eps);
}

// A small scene that mirrors the structure of the CueMol template: an #ifndef
// default that must NOT override the predefined value, an #if/#else camera
// branch, the two lighting macros and a background color.
const char* kSynthetic = R"POV(
#version 3.7;
global_settings { assumed_gamma 2.2 }
#declare _bgcolor = <0.1,0.2,0.3>;
background { color rgb _bgcolor }

#ifndef (_perspective)
  #declare _perspective = 1;
#end
#declare _distance = 100.0;
#declare _zoomy = 10.0;
#declare _zoomx = _zoomy * image_width/image_height;

#macro SpecLighting(a,b,c,d)
  light_source { <1,1,1> color rgb c }
#end
#macro FlashLighting(a)
  light_source { <0,0,1> color rgb a }
#end

camera {
 #if (_perspective)
 perspective
 location <0,0,_distance>
 angle 30
 #else
 orthographic
 direction <0,0,-1>
 up <0, _zoomy, 0>
 right <_zoomx, 0, 0>
 location <0,0,_distance>
 look_at <0,0,0>
 #end
}

SpecLighting(1, _distance, 0.5, 1)
FlashLighting(0.75)

fog {
  distance 30.0/2
  color rgbf <0.1,0.2,0.3,0>
  fog_type 2
  fog_offset 0
  fog_alt 1.0e-10
  up <0,0,1>
}

#declare _scene = #include "geom.inc"
object { _scene }
)POV";

void testSynthetic(umbreon::test::Suite& s) {
  umbreon::PovParseOptions o;
  o.imageWidth = 200;
  o.imageHeight = 100;
  o.predefined = {{"_perspective", 0.0}};  // forces the orthographic branch

  umbreon::PovSceneResult r = umbreon::readPovSceneFromString(kSynthetic, o);

  s.check("synthetic: orthographic branch selected", r.camera.orthographic);
  s.check("synthetic: ortho height = _zoomy (10)",
          approx(r.camera.height, 10.0f, 1e-4f));
  s.check("synthetic: camera at <0,0,100>",
          dirApprox(r.camera.position, umbreon::Vec3{0, 0, 100}, 1e-4f));
  s.check("synthetic: view direction is -z",
          dirApprox(r.camera.direction, umbreon::Vec3{0, 0, -1}, 1e-4f));
  s.check("synthetic: background = <0.1,0.2,0.3>",
          dirApprox(r.background, umbreon::Vec3{0.1f, 0.2f, 0.3f}, 1e-4f));
  s.check_eq("synthetic: two lights", r.lights.size(), std::size_t(2));
  if (r.lights.size() == 2) {
    s.check("synthetic: spec light intensity 0.5",
            approx(r.lights[0].intensity, 0.5f, 1e-4f));
    s.check("synthetic: spec light points along -<1,1,1>",
            dirApprox(r.lights[0].direction,
                      umbreon::normalize(umbreon::Vec3{-1, -1, -1}), 1e-3f));
    s.check("synthetic: flash light intensity 0.75",
            approx(r.lights[1].intensity, 0.75f, 1e-4f));
  }
  s.check_eq("synthetic: include path resolved", r.includePath,
             std::string("geom.inc"));

  // The predefined _perspective=0 must survive the file's #ifndef default.
  s.check("synthetic: predefined constant not overridden by #ifndef",
          r.camera.orthographic);

  // Fog block: distance expression (30/2=15), ground fog, color, up.
  s.check("synthetic: fog enabled", r.fog.enabled);
  s.check_eq("synthetic: fog type 2", r.fog.type, 2);
  s.check("synthetic: fog distance = 30/2 = 15",
          approx(r.fog.distance, 15.0f, 1e-4f));
  s.check("synthetic: fog color = <0.1,0.2,0.3>",
          dirApprox(r.fog.color, umbreon::Vec3{0.1f, 0.2f, 0.3f}, 1e-4f));
  s.check("synthetic: fog up = +Z",
          dirApprox(r.fog.up, umbreon::Vec3{0, 0, 1}, 1e-4f));
  s.check("synthetic: assumed_gamma = 2.2",
          approx(r.assumedGamma, 2.2f, 1e-4f));
}

// Verify the reader against a real CueMol .pov with the reference constants.
void testReal(umbreon::test::Suite& s, const std::string& path, const char* tag,
              float expectHeight, const umbreon::Vec3& expectBg) {
  umbreon::PovSceneResult r = umbreon::readPovScene(path, cuemolOptions(300, 300));

  s.check(std::string(tag) + ": orthographic", r.camera.orthographic);
  s.check(std::string(tag) + ": ortho height matches _zoomy",
          approx(r.camera.height, expectHeight, 1e-2f));
  s.check(std::string(tag) + ": camera at <0,0,200>",
          dirApprox(r.camera.position, umbreon::Vec3{0, 0, 200}, 1e-3f));
  s.check(std::string(tag) + ": view direction is -z",
          dirApprox(r.camera.direction, umbreon::Vec3{0, 0, -1}, 1e-3f));
  s.check(std::string(tag) + ": background color",
          dirApprox(r.background, expectBg, 1e-3f));
  s.check_eq(std::string(tag) + ": two lights", r.lights.size(),
             std::size_t(2));
  if (r.lights.size() == 2) {
    // SpecLighting: 1.3*(1-0)*(1-0.6) = 0.52, along -<1,1,1>.
    s.check(std::string(tag) + ": key light intensity ~0.52",
            approx(r.lights[0].intensity, 0.52f, 1e-3f));
    s.check(std::string(tag) + ": key light direction -<1,1,1>",
            dirApprox(r.lights[0].direction,
                      umbreon::normalize(umbreon::Vec3{-1, -1, -1}), 1e-3f));
    // FlashLighting: 1.3*(1-0)*0.6 = 0.78, along -z (eye at +z).
    s.check(std::string(tag) + ": flash light intensity ~0.78",
            approx(r.lights[1].intensity, 0.78f, 1e-3f));
    s.check(std::string(tag) + ": flash light direction -z",
            dirApprox(r.lights[1].direction, umbreon::Vec3{0, 0, -1}, 1e-3f));
  }

  // Fog is enabled (neither _transpbg nor _radiosity is set): ground fog along
  // world +Z, distance 50/3, fog color equal to the background.
  s.check(std::string(tag) + ": fog enabled", r.fog.enabled);
  s.check_eq(std::string(tag) + ": fog is ground fog (type 2)", r.fog.type, 2);
  s.check(std::string(tag) + ": fog distance ~ 50/3",
          approx(r.fog.distance, 50.0f / 3.0f, 1e-2f));
  s.check(std::string(tag) + ": fog up is +Z",
          dirApprox(r.fog.up, umbreon::Vec3{0, 0, 1}, 1e-4f));
  s.check(std::string(tag) + ": fog color matches background",
          dirApprox(r.fog.color, expectBg, 1e-3f));

  // The current CueMol scenes use assumed_gamma 2.2.
  s.check(std::string(tag) + ": assumed_gamma 2.2",
          approx(r.assumedGamma, 2.2f, 1e-3f));
}

}  // namespace

int main(int argc, char** argv) {
  umbreon::test::Suite s("pov_scene_reader");

  testSynthetic(s);

  // argv[1] = data/test1.pov, argv[2] = data/1ab0_scene1.pov
  if (argc > 1) {
    testReal(s, argv[1], "test1", 18.084867f, umbreon::Vec3{0, 0, 0});
  }
  if (argc > 2) {
    testReal(s, argv[2], "1ab0", 44.533395f, umbreon::Vec3{1, 1, 1});
  }
  if (argc <= 1) {
    std::printf("  (skipped .pov integration tests: no path arguments)\n");
  }

  return s.report();
}
