/*
  POV-Ray output from CueMol Version 2.3.2.473 (build 8714f68)
 */

#version 3.7;
#include "colors.inc"
#include "metals.inc"
#include "woods.inc"
#include "stones.inc"

#declare _bgcolor = <1.000000,1.000000,1.000000>;

#ifndef (_transpbg)
background {color rgb _bgcolor}
#else
// transparent background
background {color rgbt <1.000000,1.000000,1.000000,0.999>}
#declare _no_fog = 1;
#end

#declare _distance = 200.000000;

// _stereo ... 0:none, 1:for right eye, -1:for left eye
// _perspective ... 0:orthogonal projection, 1:perspective projection
// _iod ... inter-ocullar distance

#ifndef (_perspective)
  #declare _perspective = 1;
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
#declare _zoomy = 23.472658;
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
  assumed_gamma 1.0
}


// Spec lighting macro
// aLightSpread: area light spread (1-10)
// aDist: overall distance (200)
// aInten: light intensity (0.8)
// aShadow: shadow flag (used in point light mode (aLightSpread==1))
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
// aInten: light intensity (0.8)
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


//////////////////////
//    Lighting

#ifdef (_radiosity)

  #ifndef (_light_inten)
    #declare _light_inten=1.6;
    #declare _amb_frac=0.5;
    #declare _flash_frac=0.5;
  #end

  #include "rad_def.inc"
  global_settings {
    radiosity {
      Rad_Settings(_radiosity, off, off)
    }
  }

sphere {
  <0, 0, 0>, 1
  texture {
  pigment { color rgb <1,1,1> }
   finish { diffuse 0 emission _light_inten*_amb_frac }
  }
  hollow on
  no_shadow
  scale _distance*10
}

#ifdef (_no_fog)
plane {z,-_distance*2
  texture {
  pigment { color rgb _bgcolor }
   finish { diffuse 1 emission 0 }
  }
}
#end

#else

  #ifndef (_light_inten)
    #declare _light_inten=1.3;
    #declare _flash_frac=0.8/1.3;
    #declare _amb_frac=0;
  #end

#end

SpecLighting(_light_spread, _distance, _light_inten*(1-_amb_frac)*(1-_flash_frac), 1)
FlashLighting(_light_inten*(1-_amb_frac)*_flash_frac)
//////////////////////
//    Fog

#ifndef (_no_fog)
fog {
  distance 199.100000/3
  color rgbf <1.000000,1.000000,1.000000,0>
  fog_type 2
  fog_offset 0
  fog_alt 1.0e-10
  up <0,0,1>
}
#end


/////////////////////////////////////////////
//
// rendering properties for _34_35
//
#ifndef (_show_34_35)
#declare _show_34_35 = 1;
#end
#declare _34_35_sl_scl = 1.00;
#declare _34_35_sl_rise = 0.500000;
#declare _34_35_sl_tex = 
  texture{finish{ambient 1.0 diffuse 0 specular 0}};

//
// rendering properties for _34_45
//
#ifndef (_show_34_45)
#declare _show_34_45 = 1;
#end

//
// rendering properties for _34_46
//
#ifndef (_show_34_46)
#declare _show_34_46 = 1;
#end

//
// rendering properties for _34_47
//
#ifndef (_show_34_47)
#declare _show_34_47 = 1;
#end

//
// rendering properties for _34_48
//
#ifndef (_show_34_48)
#declare _show_34_48 = 1;
#end


//////////////////////////////////////////////

#declare _scene = #include "1ab0_scene4_materials.inc"

object{
  _scene
}

