 set(_proj_name glog)
 set(_SB_BINARY_DIR "${SB_BINARY_DIR}/${_proj_name}")

externalproject_add(${_proj_name}
    PREFIX            ${_SB_BINARY_DIR}
    TMP_DIR           ${_SB_BINARY_DIR}/tmp
    STAMP_DIR         ${_SB_BINARY_DIR}/stamp
    GIT_REPOSITORY  https://github.com/google/glog
    GIT_TAG         5c576f78c49b28d89b23fbb1fc80f54c879ec02e
    GIT_SHALLOW     ON
    UPDATE_COMMAND  ""
    SOURCE_DIR      ${SB_SOURCE_DIR}/glog
    CONFIGURE_COMMAND ./autogen.sh COMMAND COMMAND ./configure --prefix=${SB_INSTALL_DIR}
    BUILD_IN_SOURCE ON
    BUILD_COMMAND   make
)