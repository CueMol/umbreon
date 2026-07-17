/*
  Principled material matrix (S4 validation): CSG spheres in two rows --
  top = metals (POV `metallic` finishes), bottom = dielectrics (specular
  highlights) -- across a roughness column sweep, plus a pure mirror and a
  diffuse-only control. Render with --material principled to exercise the
  principled pipeline (the POV roughness values below are the inverse of
  the conversion's exponent map, targeting pbr.roughness 0.1/0.3/0.45/0.6);
  --material pov shows the POV source finishes for the A/B. Header
  structure follows data/emissive_test.pov (the bench reader's dialect).
 */

#version 3.7;

#declare _bgcolor = <1.000000,1.000000,1.000000>;

#ifndef (_transpbg)
background {color rgb _bgcolor}
#else
background {color rgbt <1.000000,1.000000,1.000000,0.999>}
#declare _no_fog = 1;
#end

#declare _distance = 200.000000;

#ifndef (_perspective)
  #declare _perspective = 0;
#end
#ifndef (_stereo)
  #declare _stereo = 0;
#end
#ifndef (_iod)
  #declare _iod = 0.03;
#end
#ifndef (_shadow)
  #declare _shadow = 0;
#end
#ifndef (_light_spread)
  #declare _light_spread = 1;
#end
#declare _zoomy = 12.000000;
#declare _zoomx = _zoomy * image_width/image_height;
#declare _fovx = 2.0*degrees( atan2(_zoomx, 2.0*_distance) );

camera {
 #if (_perspective)
 perspective
 direction <0,0,-1>
 up <0,1,0> * image_height/image_width
 right <1,0,0>
 angle _fovx
 location <_stereo*_distance*_iod,0,_distance>
 look_at <0,0,0>
 #else
 orthographic
 direction <0,0,-1>
 up <0, _zoomy, 0>
 right <_zoomx, 0, 0>
 location <_stereo*_distance*_iod,0,_distance>
 look_at <0,0,0>
 #end
}

global_settings {
  assumed_gamma 2.2
}

// Spec lighting macro (same shape as test1.pov; the bench reader keys on it)
#macro SpecLighting(aLightSpread, aDist, aInten, aShadow)
#local v1=<1,1,1>;
#local v2=<1,-0.5,-0.5>;
#local vecsz=aDist*aLightSpread/10;
light_source {
   vnormalize(v1)*aDist*2
   color rgb aInten
#if (aLightSpread>1)
   area_light vnormalize(v2)*vecsz, vcross(vnormalize(v1),vnormalize(v2))*vecsz, 10, 10
   adaptive 2
#else
   parallel point_at <0,0,0>
#if (!aShadow)
   shadowless
#end
#end
}
#end

// Flash lighting macro
#macro FlashLighting(aInten)
light_source {
   <_stereo*_distance*_iod,0,_distance>
   color rgb aInten
   shadowless
#if (!_perspective)
   parallel point_at <0,0,0>
#end
}
#end

#ifndef (_light_inten)
  #declare _light_inten=1.3;
  #declare _flash_frac=0.8/1.3;
  #declare _amb_frac=0;
#end

SpecLighting(_light_spread, _distance, _light_inten*(1-_amb_frac)*(1-_flash_frac), 1)
FlashLighting(_light_inten*(1-_amb_frac)*_flash_frac)

#declare _scene = #include "pbr_test.inc"

object{
  _scene
}
