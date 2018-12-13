#!/bin/bash

#######################################################################################################################
# This script is designed to post process a PSoC 6 application. It performs sign and merge.
#
# usage:
# 	cymcuelftool_postbuild.bash <MCUELFTOOL_LOC> <CM0P_LOC> <CM4_LOC> <MCU_CORE>
#
#######################################################################################################################

MCUELFTOOL_LOC=$1
CM0P_LOC=$2
CM4_LOC=$3
MCU_CORE=$4

echo Script: cymcuelftool_postbuild
echo 1: MCUELFTOOL_LOC : $MCUELFTOOL_LOC
echo 2: CM0P_LOC       : $CM0P_LOC
echo 3: CM4_LOC        : $CM4_LOC
echo 3: MCU_CORE       : $MCU_CORE
echo

filenameNoExt_cm0p="${CM0P_LOC%.*}"
filenameNoExt_cm4="${CM4_LOC%.*}"

if [ "$MCU_CORE" == "ARM_CM4" ]; then
$MCUELFTOOL_LOC --sign $CM4_LOC --output $filenameNoExt_cm4"_signed.elf"
$MCUELFTOOL_LOC --merge $filenameNoExt_cm4"_signed.elf" $filenameNoExt_cm0p"_signed.elf" --output $filenameNoExt_cm4"_final.elf"

#DFU
$MCUELFTOOL_LOC --sign $filenameNoExt_cm4"_final.elf" CRC --output $filenameNoExt_cm4"_dfu.elf"
$MCUELFTOOL_LOC -P $filenameNoExt_cm4"_dfu.elf" --output $filenameNoExt_cm4"_dfu.cyacd2"

else
$MCUELFTOOL_LOC --sign $CM0P_LOC --output $filenameNoExt_cm0p"_signed.elf"
    if [ -e $filenameNoExt_cm4"_signed.elf" ]; then
    $MCUELFTOOL_LOC --merge $filenameNoExt_cm4"_signed.elf" $filenameNoExt_cm0p"_signed.elf" --output $filenameNoExt_cm4"_final.elf"
    fi
fi
