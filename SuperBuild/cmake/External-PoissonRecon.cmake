set(_proj_name poissonrecon)
set(_SB_BINARY_DIR "${SB_BINARY_DIR}/${_proj_name}")

if (CYGWIN)
    # Use pre-built binaries
    externalproject_add(${_proj_name}
        PREFIX            ${_SB_BINARY_DIR}
        TMP_DIR           ${_SB_BINARY_DIR}/tmp
        STAMP_DIR         ${_SB_BINARY_DIR}/stamp
        DOWNLOAD_DIR      ${SB_DOWNLOAD_DIR}/PoissonRecon
        URL               http://www.cs.jhu.edu/~misha/Code/PoissonRecon/Version10.06/AdaptiveSolvers.x64.zip
        SOURCE_DIR        ${SB_SOURCE_DIR}/PoissonRecon
        UPDATE_COMMAND    ""
        CONFIGURE_COMMAND ""
        BUILD_COMMAND     mkdir -p ${SB_SOURCE_DIR}/PoissonRecon/Bin/Linux COMMAND chmod +x ${SB_SOURCE_DIR}/PoissonRecon/PoissonRecon || true
        INSTALL_COMMAND   mv ${SB_SOURCE_DIR}/PoissonRecon/PoissonRecon ${SB_SOURCE_DIR}/PoissonRecon/Bin/Linux/ || true
        LOG_DOWNLOAD      OFF
        LOG_CONFIGURE     OFF
        LOG_BUILD         OFF
    )
else()
    # Build from sources
    externalproject_add(${_proj_name}
        GIT_REPOSITORY    https://github.com/mkazhdan/PoissonRecon.git
        GIT_TAG           ce5005ae3094d902d551a65a8b3131e06f45e7cf
        SOURCE_DIR        ${SB_SOURCE_DIR}/PoissonRecon
        UPDATE_COMMAND    ""
        CONFIGURE_COMMAND ""
        BUILD_IN_SOURCE 1
        BUILD_COMMAND     make poissonrecon
        INSTALL_COMMAND   ""
    )
endif()
