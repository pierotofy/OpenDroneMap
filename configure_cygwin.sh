#!/bin/bash

if [[ $2 =~ ^[0-9]+$ ]] ; then
    processes=$2
else
    processes=$(nproc)
fi

install() {
    ## Before installing
    echo "Updating the system"
    apt-cyg update

    echo "Installing Required Requisites"
    apt-cyg install make automake autogen autoconf libtool patch gcc-core gcc-g++ git cmake python-pip gdal libgdal-devel libgeos-devel libgeotiff-devel libjsoncpp-devel python-gdal python3-devel liblapack-devel eigen3 libboost-devel zlib-devel libexpat-devel gettext-devel libpng-devel libtiff-devel libjpeg-devel libxml2-devel libxslt-devel libiconv-devel libboost_python3-devel gcc-fortran libcrypt-devel libffi-devel  libsuitesparseconfig-devel libamd-devel libcamd-devel libcolamd-devel libccolamd-devel libcholmod-devel libcxsparse-devel libspqr-devel proj libproj-devel

    echo "Replacing g++ and gcc with our scripts ..."
    if [ ! -e /usr/bin/gcc_real.exe ]; then
        mv -v /usr/bin/gcc.exe /usr/bin/gcc_real.exe
        cp -v cygwin/gcc /usr/bin/gcc.exe
    fi
    if [ ! -e /usr/bin/g++_real.exe ]; then
        mv -v /usr/bin/g++.exe /usr/bin/g++_real.exe
        cp -v cygwin/g++ /usr/bin/g++.exe
    fi
    if [ ! -e /usr/bin/c++_real.exe ]; then
        mv -v /usr/bin/c++.exe /usr/bin/c++_real.exe
        cp -v cygwin/g++ /usr/bin/c++.exe
    fi

    # Upgrade pip
    python -m pip install --upgrade pip

    # Install packages
    python -m pip install -r requirements.txt

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
