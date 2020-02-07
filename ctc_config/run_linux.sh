#!/bin/bash
# set -x
set -e

########################################################################################################################
# CTC description
NAME=(ClassroomVideo TechnicolorMuseum InterdigitalHijack TechnicolorPainter IntelFrog OrangeKitchen PoznanFencing NokiaChess)
CODE=(A B C D E J L N)
SIZE=(2048x2048 2048x2048 2048x2048 1920x1080 1920x1080 1920x1080 1920x1080 2048x2048)
METRIC=(WS-PSNR WS-PSNR WS-PSNR PSNR PSNR PSNR PSNR WS-PSNR)
FIRSTVIEW=(0 0 0 0 1 0 0 0)
NBVIEW=(15 24 10 16 13 25 10 10)
NBDIGIT=(1 1 1 1 1 2 1 1)
NBCONTENT=${#NAME[@]}

QPC=(22 27 32 37 42)
QPD=(12 17 22 27 32)

ATLASNUMBER=(1 3 2 3 6 6 3 3)

ATLASWIDTHCOLOR=(4096 2048 4096 2048 1920 1920 1920 1920)
ATLASHEIGHTCOLOR=(4096 2736 2048 2736 1456 1456 2912 2736)

ATLASWIDTHDEPTH=(2048 1024 2048 1024 960 960 960 1024)
ATLASHEIGHTDEPTH=(2048 1376 1024 1376 736 736 1456 1376)


########################################################################################################################
# Script configuration
SCRIPT_DIR=$(cd "$(dirname "$0")"; pwd)

CONFIG_DIR="$SCRIPT_DIR/config/A97"
SOURCE_DIR="/run/media/julien/Samsung_T5_9/MPEG/content"
OUTPUT_DIR="/run/media/julien/Samsung_T5_9/MPEG/output"

DEBUG_MODE=""

ENCODER_EXE="$DEBUG_MODE /home/julien/Technicolor/dev/git/TMIV_MERGE/bin/Encoder"
HM_EXE="$DEBUG_MODE /home/julien/Technicolor/dev/git/HM/bin/TAppEncoderStatic"
DECODER_EXE="$DEBUG_MODE /home/julien/Technicolor/dev/git/TMIV_MERGE/bin/Decoder"
FFMPEG_EXE="$DEBUG_MODE ffmpeg"
PYTHON_EXE="$DEBUG_MODE python"

CONFIG_SCRIPT=$SCRIPT_DIR/make_tmiv_configs_m52414.py

# CONTENT_LIST=$(seq 0 $((${#NAME[@]}-1)))
CONTENT_LIST="4"

# QP_LIST=$(seq 0 $((${#QPC[@]}-1)))
QP_LIST="0"

# TRACE_LIST=$(seq 0 2)
TRACE_LIST="0"

# NB_FRAME=97
NB_FRAME=1

# NB_EXTRA_FRAME=203
NB_EXTRA_FRAME=99

CONFIG_CREATION=0
ATLAS_CREATION=1
ATLAS_CODING=1
SOURCE_RECOVERY=1
POSETRACE_GENERATION=0
OVERALL_RESULT=0

########################################################################################################################
# FFmpeg helpers
FFMPEG_ENCODE () {
${FFMPEG_EXE} -y -f rawvideo -pix_fmt yuv420p10le -s:v $1 -r 30 -i $2 -c:v libx265 -crf 10 $3
}

FFMPEG_SIDEBYSIDE () {
${FFMPEG_EXE} -y -i $1 -i $2 -c:v libx265 -crf 10 -filter_complex hstack $3
}

########################################################################################################################
# HM helpers
HM_CODING () {

# Parameters
CONTENT=$1
QP=$2
ATLAS=$3

# Global
CONFIG="$SCRIPT_DIR/encoder_randomaccess_main10.cfg"
BITDEPTH=10
FRAMERATE=30
MD5=1
LEVEL=5.2

WIDTHC=${ATLASWIDTHCOLOR[$CONTENT]}
HEIGHTC=${ATLASHEIGHTCOLOR[$CONTENT]}

WIDTHD=${ATLASWIDTHDEPTH[$CONTENT]}
HEIGHTD=${ATLASHEIGHTDEPTH[$CONTENT]}

# Color coding
COLOR_INPUT_ATLAS_BASENAME=ATL_S${CODE[$CONTENT]}_R0_Tt_c0${ATLAS}_${WIDTHC}x${HEIGHTC}_yuv420p10le
COLOR_OUTPUT_ATLAS_BASENAME=ATL_S${CODE[$CONTENT]}_QP$(($QP+1))_Tt_c0${ATLAS}_${WIDTHC}x${HEIGHTC}_yuv420p10le

COLOR_ATLAS_INPUT=$OUTPUT_DIR/${NAME[$CONTENT]}/$COLOR_INPUT_ATLAS_BASENAME.yuv
COLOR_ATLAS_BITSTREAM=$OUTPUT_DIR/${NAME[$CONTENT]}/$COLOR_OUTPUT_ATLAS_BASENAME.bit
COLOR_ATLAS_OUTPUT=$OUTPUT_DIR/${NAME[$CONTENT]}/$COLOR_OUTPUT_ATLAS_BASENAME.yuv

$HM_EXE -c $CONFIG -i $COLOR_ATLAS_INPUT -wdt $WIDTHC -hgt $HEIGHTC --InputBitDepth=$BITDEPTH --FrameRate=$FRAMERATE --FramesToBeEncoded=$NB_FRAME -b $COLOR_ATLAS_BITSTREAM -o $COLOR_ATLAS_OUTPUT -q ${QPC[$QP]} --SEIDecodedPictureHash=$MD5 --Level=$LEVEL 1> $OUTPUT_DIR/${NAME[$CONTENT]}/$COLOR_OUTPUT_ATLAS_BASENAME.log &

# Depth coding
DEPTH_INPUT_ATLAS_BASENAME=ATL_S${CODE[$CONTENT]}_R0_Td_c0${ATLAS}_${WIDTHD}x${HEIGHTD}_yuv420p10le
DEPTH_OUTPUT_ATLAS_BASENAME=ATL_S${CODE[$CONTENT]}_QP$(($QP+1))_Td_c0${ATLAS}_${WIDTHD}x${HEIGHTD}_yuv420p10le

DEPTH_ATLAS_INPUT=$OUTPUT_DIR/${NAME[$CONTENT]}/$DEPTH_INPUT_ATLAS_BASENAME.yuv
DEPTH_ATLAS_BITSTREAM=$OUTPUT_DIR/${NAME[$CONTENT]}/$DEPTH_OUTPUT_ATLAS_BASENAME.bit
DEPTH_ATLAS_OUTPUT=$OUTPUT_DIR/${NAME[$CONTENT]}/$DEPTH_OUTPUT_ATLAS_BASENAME.yuv

$HM_EXE -c $CONFIG -i $DEPTH_ATLAS_INPUT -wdt $WIDTHD -hgt $HEIGHTD --InputBitDepth=$BITDEPTH --FrameRate=$FRAMERATE --FramesToBeEncoded=$NB_FRAME -b $DEPTH_ATLAS_BITSTREAM -o $DEPTH_ATLAS_OUTPUT -q ${QPD[$QP]} --SEIDecodedPictureHash=$MD5 --Level=$LEVEL 1> $OUTPUT_DIR/${NAME[$CONTENT]}/$DEPTH_OUTPUT_ATLAS_BASENAME.log &

# Metadata
METADATA_INPUT_ATLAS_NAME=ATL_S${CODE[$CONTENT]}_R0_Tm_c00.bit
METADATA_OUTPUT_ATLAS_NAME=ATL_S${CODE[$CONTENT]}_QP$(($QP+1))_Tm_c00.bit

cp $OUTPUT_DIR/${NAME[$CONTENT]}/$METADATA_INPUT_ATLAS_NAME $OUTPUT_DIR/${NAME[$CONTENT]}/$METADATA_OUTPUT_ATLAS_NAME

}

########################################################################################################################
# Configuration generation
if [ $CONFIG_CREATION -eq 1 ]
then

echo ""
echo "#############################"
echo "###### CONFIG CREATION ######"
echo "#############################"
echo ""

mkdir -p $SCRIPT_DIR/config
cd $SCRIPT_DIR/config
rm -fR *
$PYTHON_EXE $CONFIG_SCRIPT

fi

########################################################################################################################
# Atlas creation
if [ $ATLAS_CREATION -eq 1 ]
then

echo ""
echo "##############################"
echo "####### ATLAS CREATION #######"
echo "##############################"
echo ""

for content in ${CONTENT_LIST}
do

mkdir -p $OUTPUT_DIR/${NAME[$content]}
$ENCODER_EXE -c $CONFIG_DIR/S${CODE[$content]}/TMIV_A97_S${CODE[$content]}.json -p SourceDirectory $SOURCE_DIR/${NAME[$content]} -p OutputDirectory $OUTPUT_DIR/${NAME[$content]} -p numberOfFrames $NB_FRAME

done

fi

########################################################################################################################
# Atlas coding
if [ $ATLAS_CODING -eq 1 ]
then

# HM
echo ""
echo "######################################"
echo "######## HM CODING / DECODING ########"
echo "######################################"
echo ""

for qp in ${QP_LIST}
do

echo ""
echo "----- QP$(($qp+1)) -----"
echo ""

for content in ${CONTENT_LIST}
do

mkdir -p $OUTPUT_DIR/${NAME[$content]}

for atlas in $(seq 0 $((${ATLASNUMBER[$content]}-1)))
do

HM_CODING $content $qp $atlas

done

done

done

wait

fi

########################################################################################################################
# Source recovery
if [ $SOURCE_RECOVERY -eq 1 ]
then

echo ""
echo "##############################"
echo "###### SOURCE  RECOVERY ######"
echo "##############################"
echo ""

for qp in ${QP_LIST}
do

echo ""
echo "----- QP$(($qp+1)) -----"
echo ""

for content in ${CONTENT_LIST}
do

mkdir -p $OUTPUT_DIR/${NAME[$content]}

# for ((view=${FIRSTVIEW[content]};view<$((${FIRSTVIEW[content]}+${NBVIEW[content]}));view++))
# do

view=5

if [ ${NBDIGIT[content]} -eq 2 ] && [ $view -lt 10 ]
then

viewName=0$view

else

viewName=$view

fi

$DECODER_EXE -c $CONFIG_DIR/S${CODE[$content]}/QP$(($qp+1))/TMIV_A97_S${CODE[$content]}_QP$(($qp+1))_v$viewName.json -p SourceDirectory $SOURCE_DIR/${NAME[$content]} -p OutputDirectory $OUTPUT_DIR/${NAME[$content]} -p numberOfFrames $NB_FRAME

# done

done

done

fi

########################################################################################################################
# Pose traces
if [ $POSETRACE_GENERATION -eq 1 ]
then

echo ""
echo "##############################"
echo "#### POSETRACE GENERATION ####"
echo "##############################"
echo ""

for trace in ${TRACE_LIST}
do

for qp in ${QP_LIST}
do

echo ""
echo "----- Trace #0$(($trace+1)) / QP$(($qp+1)) -----"
echo ""

for content in ${CONTENT_LIST}
do

echo ""
echo "*** ${NAME[$content]} ***"
echo ""

mkdir -p $OUTPUT_DIR/${NAME[$content]}

$DECODER_EXE -c $CONFIG_DIR/S${CODE[$content]}/QP$(($qp+1))/TMIV_A97_S${CODE[$content]}_QP$(($qp+1))_p0$(($trace+1)).json -p SourceDirectory $SOURCE_DIR/${NAME[$content]} -p OutputDirectory $OUTPUT_DIR/${NAME[$content]} -p numberOfFrames $NB_FRAME

FFMPEG_ENCODE ${SIZE[$content]} "$OUTPUT_DIR/${NAME[$content]}/${CODE[$content]}p0$(($trace+1))_QP$(($qp+1))_${SIZE[$content]}_yuv420p10le.yuv" "$OUTPUT_DIR/${NAME[$content]}/QP$qp/${CODE[$content]}p0$traceId.mp4"

done

done

done

fi
