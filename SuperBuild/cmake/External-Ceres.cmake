set(_proj_name ceres)
set(_SB_BINARY_DIR "${SB_BINARY_DIR}/${_proj_name}")

if (CYGWIN)
  set (EXTRA_CXX_FLAGS "\ -Wno-maybe-uninitialized")
endif()

ExternalProject_Add(${_proj_name}
  PREFIX            ${_SB_BINARY_DIR}
  TMP_DIR           ${_SB_BINARY_DIR}/tmp
  STAMP_DIR         ${_SB_BINARY_DIR}/stamp
  #--Download step--------------
  DOWNLOAD_DIR      ${SB_DOWNLOAD_DIR}
  URL               http://ceres-solver.org/ceres-solver-1.14.0.tar.gz
  #--Update/Patch step----------
  UPDATE_COMMAND    ""
  #--Configure step-------------
  SOURCE_DIR        ${SB_SOURCE_DIR}/${_proj_name}
  CMAKE_ARGS
    -DCMAKE_C_FLAGS=-fPIC
    -DCMAKE_CXX_FLAGS=-fPIC${EXTRA_CXX_FLAGS}
    -DBUILD_EXAMPLES=OFF
    -DBUILD_TESTING=OFF
    -DMINIGLOG=ON
    -DGFLAGS=OFF
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