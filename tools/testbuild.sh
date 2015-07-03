#!/bin/bash
# version.sh
#
#   Copyright (C) 2011 Gregory Nutt. All rights reserved.
#   Author: Gregory Nutt <gnutt@nuttx.org>
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name NuttX nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#

WD=$PWD
nuttx=$WD/../nuttx

progname=$0
host=linux
wenv=cygwin
sizet=uint
unset testfile

function showusage {
    echo ""
    echo "USAGE: $progname [-w|l] [-c|n] [-s] <testlist-file>"
    echo "USAGE: $progname -h"
    echo ""
    echo "where"
    echo "  -w|l selects Windows (w) or Linux (l).  Default: Linux"
    echo "  -c|n selects Windows native (n) or Cygwin (c).  Default Cygwin"
    echo "  -s Use C++ long size_t in new operator. Default unsigned long"
    echo "  -h will show this help test and terminate"
    echo "  <testlist-file> selects the list of configurations to test.  No default"
    echo ""
    echo "Your PATH variable must include the path to both the build tools and the"
    echo "kconfig-frontends tools"
    echo ""
    exit 1
}

# Parse command line

while [ ! -z "$1" ]; do
    case $1 in
    -w )
    host=windows
    ;;
    -l )
    host=linux
    ;;
    -c )
    wenv=cygwin
    ;;
    -n )
    wenv=n
    ;;
    -s )
    sizet=long
    ;;
    -h )
    showusage
    ;;
    * )
    testfile="$1"
    shift
    break;
    ;;
  esac
  shift
done

if [ ! -z "$1" ]; then
   echo "ERROR: Garbage at the end of line"
   showusage
fi

if [ -z "$testfile" ]; then
    echo "ERROR: Missing test list file"
    showusage
fi

if [ ! -r "$testfile" ]; then
    echo "ERROR: No readable file exists at $testfile"
    echo $USAGE
    showusage
fi

if [ ! -d "$nuttx" ]; then
    echo "ERROR: Expected to find nuttx/ at $nuttx"
    echo $USAGE
    showusage
fi

# Clean up after the last build

function distclean {
    cd $nuttx || { echo "ERROR: failed to CD to $nuttx"; exit 1; }
    if [ -f .config ]; then
        echo "  Cleaning..."
        make distclean 1>/dev/null
    fi
}

# Configure for the next build

function configure {
    cd $nuttx/tools || { echo "ERROR: failed to CD to $nuttx/tools"; exit 1; }
    echo "  Configuring..."
    ./configure.sh $config

    cd $nuttx || { echo "ERROR: failed to CD to $nuttx"; exit 1; }

    if [ "X$host" == "Xlinux" ]; then
        echo "  Select CONFIG_HOST_LINUX=y"

        kconfig-tweak --file $nuttx/.config --enable CONFIG_HOST_LINUX
        kconfig-tweak --file $nuttx/.config --disable CONFIG_HOST_WINDOWS

        kconfig-tweak --file $nuttx/.config --disable CONFIG_WINDOWS_NATIVE
        kconfig-tweak --file $nuttx/.config --disable CONFIG_WINDOWS_CYGWIN
        kconfig-tweak --file $nuttx/.config --disable CONFIG_WINDOWS_MSYS
        kconfig-tweak --file $nuttx/.config --disable CONFIG_WINDOWS_OTHER

    else
        echo "  Select CONFIG_HOST_WINDOWS=y"
        kconfig-tweak --file $nuttx/.config --enable CONFIG_HOST_WINDOWS
        kconfig-tweak --file $nuttx/.config --disable CONFIG_HOST_LINUX

        if [ "X$wenv" == "Xcygwin" ]; then
          echo "  Select CONFIG_HOST_CYGWIN=y"
          kconfig-tweak --file $nuttx/.config --enable CONFIG_WINDOWS_CYGWIN
          kconfig-tweak --file $nuttx/.config --disable CONFIG_WINDOWS_NATIVE
        else
          echo "  Select CONFIG_HOST_MSYS=y"
          kconfig-tweak --file $nuttx/.config --enable CONFIG_WINDOWS_NATIVE
          kconfig-tweak --file $nuttx/.config --disable CONFIG_WINDOWS_CYGWIN
        fi

        kconfig-tweak --file $nuttx/.config --disable CONFIG_WINDOWS_MSYS
        kconfig-tweak --file $nuttx/.config --disable CONFIG_WINDOWS_OTHER
    fi

    kconfig-tweak --file $nuttx/.config --disable CONFIG_HOST_OSX
    kconfig-tweak --file $nuttx/.config --disable CONFIG_HOST_OTHER

    if [ "X$sizet" == "Xlong" ]; then
        echo "  Select CONFIG_CXX_NEWLONG=y"

        kconfig-tweak --file $nuttx/.config --enable CONFIG_CXX_NEWLONG
    else
        echo "  Disable CONFIG_CXX_NEWLONG"
        kconfig-tweak --file $nuttx/.config --disable CONFIG_CXX_NEWLONG
    fi

    setting=`grep TOOLCHAIN $nuttx/.config | grep =y`
    varname=`echo $setting | cut -d'=' -f1`
    if [ ! -z "varname" ]; then
        echo "  Disabling $varname"
        kconfig-tweak --file $nuttx/.config --disable $varname
    fi

    echo "  Enabling $toolchain"
    kconfig-tweak --file $nuttx/.config --enable $toolchain

    echo "  Refreshing..."
    kconfig-conf --olddefconfig Kconfig 1>/dev/null
}

# Perform the next build

function build {
    cd $nuttx || { echo "ERROR: failed to CD to $nuttx"; exit 1; }
    echo "  Building..."
    echo "------------------------------------------------------------------------------------"
    make -i 1>/dev/null
}

# Coordinate the steps for the next build test

function dotest {
    echo "------------------------------------------------------------------------------------"
    distclean
    configure
    build
}

# Perform the build test for each entry in the test list file

export APPSDIR=../apps

# Shouldn't have to do this

testlist=`cat $testfile`

#while read -r line || [[ -n $line ]]; do
for line in $testlist; do
    echo "===================================================================================="
    echo "Configuration/Tool Config: $line"

    # Parse the next line

    config=`echo $line | cut -d',' -f1`

    path=$nuttx/configs/$config
    if [ ! -r "$path/defconfig" ]; then
        echo "ERROR: no configuration found at $path"
        showusage
    fi

    toolchain=`echo $line | cut -d',' -f2`
    if [ -z "$toolchain" ]; then
        echo "ERROR no tool configuration"
        showusage
    fi

    # Perform the build test

    dotest
    cd $WD || { echo "ERROR: Failed to CD to $WD"; exit 1; }
done # < $testfile

echo "===================================================================================="
