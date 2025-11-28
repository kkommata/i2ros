#!/bin/bash

WIDTH=${1:-1280}
HEIGHT=${2:-720}

$(rospack find simulation)/unity_sim/Build_Ubuntu/AD_Sim.x86_64 \
    -screen-width "$WIDTH" -screen-height "$HEIGHT"
