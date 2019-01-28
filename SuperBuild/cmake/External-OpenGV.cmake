set(_proj_name opengv)
set(_SB_BINARY_DIR "${SB_BINARY_DIR}/${_proj_name}")

if(CYGWIN)
  set(EXTRA_CMAKE_ARGS -DPYBIND11_CPP_STANDARD=-std=gnu++11)
endif()

ExternalProject_Add(${_proj_name}
  PREFIX            ${_SB_BINARY_DIR}
  TMP_DIR           ${_SB_BINARY_DIR}/tmp
  STAMP_DIR         ${_SB_BINARY_DIR}/stamp
  #--Download step--------------
  DOWNLOAD_DIR      ${SB_DOWNLOAD_DIR}
  GIT_REPOSITORY    https://github.com/pierotofy/opengv/
  GIT_TAG           9c06e3c02bc78405b870547ada9fbdddab81cbc3
  GIT_SHALLOW       ON
  #--Update/Patch step----------
  PATCH_COMMAND    git submodule update --init --recursive
  #--Configure step-------------
  SOURCE_DIR        ${SB_SOURCE_DIR}/${_proj_name}
  CMAKE_ARGS
    -DBUILD_TESTS=OFF 
    -DBUILD_PYTHON=ON
    -DPYBIND11_PYTHON_VERSION=2.7
    -DCMAKE_INSTALL_PREFIX:PATH=${SB_INSTALL_DIR}
    ${EXTRA_CMAKE_ARGS}
  #--Build step-----------------
  BINARY_DIR        ${_SB_BINARY_DIR}
  #--Install step---------------
  INSTALL_DIR       ${SB_INSTALL_DIR}
  #--Output logging-------------
  LOG_DOWNLOAD      OFF
  LOG_CONFIGURE     OFF
  LOG_BUILD         OFF
)

