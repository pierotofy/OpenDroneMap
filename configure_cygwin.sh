#!/bin/bash

if [[ $2 =~ ^[0-9]+$ ]] ; then
    processes=$2
else
    processes=$(nproc)
fi

install() {
    ## Set up library paths
    export PYTHONPATH=$RUNPATH/SuperBuild/install/lib/python2.7/dist-packages:$RUNPATH/SuperBuild/src/opensfm:$RUNPATH/SuperBuild/install/lib/python2.7/site-packages:$PYTHONPATH
    export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$RUNPATH/SuperBuild/install/lib
    export PATH=$PATH:$RUNPATH/SuperBuild/install/lib:$RUNPATH/SuperBuild/install/bin

    ## Before installing
    echo "Updating the system"
    apt-cyg update

    echo "Installing Required Requisites"
    apt-cyg install make automake autogen autoconf libtool patch gcc-core gcc-g++ git cmake python-pip gdal libgdal-devel libgeos-devel libgeotiff-devel libjsoncpp-devel python-gdal python-devel liblapack-devel eigen3 libboost-devel zlib-devel libexpat-devel gettext-devel libpng-devel libtiff-devel libjpeg-devel libxml2-devel libiconv-devel libboost_python-devel gcc-fortran libcrypt-devel  libsuitesparseconfig-devel libamd-devel libcamd-devel libcolamd-devel libccolamd-devel libcholmod-devel libcxsparse-devel libspqr-devel

    # Upgrade pip
    pip2 install --upgrade pip

    ## Installing OpenSfM Requisites
    echo "Installing OpenSfM Dependencies"
    pip install -U PyYAML cloudpickle six exifread gpxpy xmltodict appsettings https://github.com/gipit/gippy/archive/1.0.0.zip loky shapely scipy numpy pyproj repoze.lru

    echo "Installing Ecto Dependencies"

    # TODO: missing pyside (needs qmake?)
    # TODO missing liblas-bin
    echo "Compiling SuperBuild"
    cd ${RUNPATH}/SuperBuild
    mkdir -p build && cd build
    cmake .. && make -j$processes

    echo "Compiling build"
    cd ${RUNPATH}
    mkdir -p build && cd build
    cmake .. && make -j$processes

    echo "Configuration Finished"
}

uninstall() {
    echo "Removing SuperBuild and build directories"
    cd ${RUNPATH}/SuperBuild
    rm -rfv build src download install
    cd ../
    rm -rfv build
}

reinstall() {
    echo "Reinstalling ODM modules"
    uninstall
    install
}

usage() {
    echo "Usage:"
    echo "bash configure.sh <install|update|uninstall|help> [nproc]"
    echo "Subcommands:"
    echo "  install"
    echo "    Installs all dependencies and modules for running OpenDroneMap"
    echo "  reinstall"
    echo "    Removes SuperBuild and build modules, then re-installs them. Note this does not update OpenDroneMap to the latest version. "
    echo "  uninstall"
    echo "    Removes SuperBuild and build modules. Does not uninstall dependencies"
    echo "  help"
    echo "    Displays this message"
    echo "[nproc] is an optional argument that can set the number of processes for the make -j tag. By default it uses $(nproc)"
}

if [[ $1 =~ ^(install|reinstall|uninstall|usage)$ ]]; then
    RUNPATH="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
    "$1"
else
    echo "Invalid instructions." >&2
    usage
    exit 1
fi
