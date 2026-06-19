#!/bin/bash

set -eux

INPUT=1ab0_scene1
#GAMMA=sRGB
#GAMMA=sRGB
GAMMA=1

povray "Input_File_Name=$INPUT.pov" "Output_File_Name=${INPUT}_${GAMMA}.png" Declare=_stereo=0 Declare=_iod=0.03 Declare=_perspective=0 Declare=_shadow=0 Declare=_light_inten=1.3 \
       Declare=_flash_frac=0.6 Declare=_amb_frac=0 -D +WT4 +W300 +H300 +FN8 Quality=11 Antialias=On Antialias_Depth=3 Antialias_Threshold=0.1 Jitter=Off +V \
       File_Gamma=$GAMMA Display_Gamma=0.5

# povray "Input_File_Name=$INPUT.pov" "Output_File_Name=${INPUT}_${GAMMA}.png" "Library_Path='/Applications/CueMol2.app/Contents/Resources/povray/include'" Declare=_stereo=0 Declare=_iod=0.03 Declare=_perspective=0 Declare=_shadow=0 Declare=_light_inten=1.3 Declare=_flash_frac=0.6 Declare=_amb_frac=0 -D +WT4 +W300 +H300 +FN8 Quality=11 Antialias=On Antialias_Depth=3 Antialias_Threshold=0.1 Jitter=Off +V File_Gamma=$GAMMA

/Applications/CueMol2.app/Contents/Resources/blendpng ${INPUT}_${GAMMA}.png $INPUT.png 300
