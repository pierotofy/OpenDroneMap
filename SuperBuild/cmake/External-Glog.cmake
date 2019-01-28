externalproject_add(glog
    GIT_REPOSITORY  https://github.com/google/glog
    GIT_TAG         7ffca211fe8bf30453da9e27b66000d3735f96b9
    GIT_SHALLOW     ON
    UPDATE_COMMAND  ""
    SOURCE_DIR      ${SB_SOURCE_DIR}/glog
    CONFIGURE_COMMAND ./autogen.sh COMMAND COMMAND ./configure --prefix=${SB_INSTALL_DIR}
    BUILD_IN_SOURCE ON
    BUILD_COMMAND   make
)