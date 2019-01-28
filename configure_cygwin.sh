#!/bin/bash

if [[ $2 =~ ^[0-9]+$ ]] ; then
    processes=$2
else
    processes=$(nproc)
fi

install() {
    ## Set up library paths
    export PYTHONPATH=$RUNPATH/SuperBuild/install/lib/python2.7/dist-packages:$RUNPATH/SuperBuild/src/opensfm:$PYTHONPATH
    export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$RUNPATH/SuperBuild/install/lib

    ## Before installing
    echo "Updating the system"
    apt-cyg update

    echo "Installing Required Requisites"
    apt-cyg install make automake gcc-core gcc-g++ git cmake python-pip gdal libgdal-devel libgeos-devel libgeotiff-devel libjsoncpp-devel python-gdal python-devel liblapack-devel eigen3 libboost-devel apt-cyg install zlib-devel libexpat-devel gettext-devel

    # Upgrade pip
    pip2 install --upgrade pip

    echo "Installing OpenCV Dependencies"
    apt-cyg install opencv libopencv-devel

    ## Installing OpenSfM Requisites
    echo "Installing OpenSfM Dependencies"
    pip install -U numpy==1.15.4
    pip install -U PyYAML cloudpickle six exifread gpxpy xmltodict appsettings https://github.com/gipit/gippy/archive/1.0.0.zip loky shapely scipy numpy==1.15.4 pyproj repoze.lru libboost_python-devel

    # apt-get install -y -qq python-networkx \
    #                      libgoogle-glog-dev \
    #                      libsuitesparse-dev \
    #                      libboost-filesystem-dev \
    #                      libboost-iostreams-dev \
    #                      libboost-regex-dev \
    #                      libboost-python-dev \
    #                      libboost-date-time-dev \
    #                      libboost-thread-dev \
    #                      python-pyproj

    # pip install -U PyYAML \
    #                     exifread \
    #                     gpxpy \
    #                     xmltodict \
    #                     appsettings \
    #                     loky \
    #                     repoze.lru

    echo "Installing Ecto Dependencies"

    # TODO: missing pyside (needs qmake?)
    # TODO missing liblas-bin
    pip install -U catkin-pkg empy nose

    # echo "Installing lidar2dems Dependencies"
    # apt-get install -y -qq swig2.0 \
    #                      python-wheel \
    #                      libboost-log-dev

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
