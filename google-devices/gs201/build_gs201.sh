#!/bin/bash
# SPDX-License-Identifier: GPL-2.0-only

source private/devices/google/common/shell_utils.sh
setup_cog_env_if_needed

exec tools/bazel run \
    --config=stamp \
    --config=gs201 \
    //private/devices/google/gs201:dist "$@"
