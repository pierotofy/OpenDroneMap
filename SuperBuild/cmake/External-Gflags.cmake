 set(_proj_name gflags)
 set(_SB_BINARY_DIR "${SB_BINARY_DIR}/${_proj_name}")

externalproject_add(${_proj_name}
    PREFIX            ${_SB_BINARY_DIR}
    TMP_DIR           ${_SB_BINARY_DIR}/tmp
    STAMP_DIR         ${_SB_BINARY_DIR}/stamp
    GIT_REPOSITORY  https://github.com/gflags/gflags
    GIT_TAG         28f50e0fed19872e0fd50dd23ce2ee8cd759338e
    GIT_SHALLOW     ON
    UPDATE_COMMAND  ""
    SOURCE_DIR        ${SB_SOURCE_DIR}/${_proj_name}
    CMAKE_ARGS
        -DCMAKE_INSTALL_PREFIX:PATH=${SB_INSTALL_DIR}
    
    #--Build step-----------------
    BINARY_DIR        ${_SB_BINARY_DIR}
    #--Install step---------------
    INSTALL_DIR       ${SB_INSTALL_DIR}
    #--Output logging-------------
    LOG_DOWNLOAD      OFF
    LOG_CONFIGURE     OFF
    LOG_BUILD         OFF
)
