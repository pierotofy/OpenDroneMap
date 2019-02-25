#!/bin/bash

RUNPATH="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
export PYTHONPATH=$RUNPATH/SuperBuild/install/lib/python2.7/dist-packages:$RUNPATH/SuperBuild/src/opensfm:$RUNPATH/SuperBuild/install/lib/python2.7/site-packages:$PYTHONPATH
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$RUNPATH/SuperBuild/install/lib
export PATH=$PATH:$RUNPATH/SuperBuild/install/lib:$RUNPATH/SuperBuild/install/bin

python $RUNPATH/run.py "$@"

