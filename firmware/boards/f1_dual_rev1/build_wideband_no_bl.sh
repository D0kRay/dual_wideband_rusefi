#!/bin/bash

BOARD=f1_dual_rev1 \
USE_OPT="-O0 -ggdb -fomit-frame-pointer -falign-functions=16 -fsingle-precision-constant" \../build_f1_board.sh
