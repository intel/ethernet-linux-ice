#!/bin/bash
# SPDX-License-Identifier: GPL-2.0
# Copyright (C) 2018-2019, Intel Corporation.

# Copyright (C) 2018 Intel Corporation
# For licensing information, see the file 'LICENSE' in the root folder
#
# gcc-i-sys.sh
#
# System and kernel headers often generate warnings which are not fixable,
# because we can't change the header files to resolve the issue. In general,
# this only occurs at high levels of warning. However, this makes testing a
# driver for warnings set to higher levels is unweildy. To resolve this,
# gcc-i-sys.sh is a wrapper script which will invoke gcc after replacing -I
# with -isystem on all included files. We do this in order so that all kernel
# header directories included by the kernel Kbuild system are modified, not
# just the ones specified in our own Makefile.
#
# This script allows building a driver with maximum warnings enabled by the
# compiler, without showing the multitude of complains within system headers.
#
# To use this script, simply export REAL_CC as the real C compiler used in the
# Makefile, and set this as the CC variable.
args=()

while [[ ${#} > 0 ]]; do
    if [[ "${1:0:2}" == "-I" ]]; then
        # Only insert the -isystem if we have a directory parameter. This fixes
        # a bug in some older kernel builds which insert spurious -I options on
        # their own.
        if [[ -n "${1/#-I}" ]] || [[ -d "${2}" ]]; then
            args+=("-isystem")
            args+=("${1/#-I}")
        fi
        shift
        continue
    fi
    args+=("$1")
    shift
done

# Enable verbose output
if [[ "${V}" == "1" ]] || [[ "${V}" == "2" ]]; then
	set -x
fi

exec ${REAL_CC} "${args[@]}"
