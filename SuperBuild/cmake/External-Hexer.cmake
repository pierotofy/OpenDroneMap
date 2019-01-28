set(_proj_name hexer)
set(_SB_BINARY_DIR "${SB_BINARY_DIR}/${_proj_name}")

if(CYGWIN)
  set(EXTRA_CMAKE_ARGS -DGDAL_LIBRARY=gdal -DCMAKE_CXX_FLAGS=-std=gnu++11 -DWIN32=ON)
endif()

ExternalProject_Add(${_proj_name}
  DEPENDS           
  PREFIX            ${_SB_BINARY_DIR}
  TMP_DIR           ${_SB_BINARY_DIR}/tmp
  STAMP_DIR         ${_SB_BINARY_DIR}/stamp
  #--Download step--------------
  DOWNLOAD_DIR      ${SB_DOWNLOAD_DIR}
  URL               https://github.com/hobu/hexer/archive/bc748fc16b51c562f68f6641574b7af4244adfa2.tar.gz
  #--Update/Patch step----------
  UPDATE_COMMAND    ""
  #--Configure step-------------
  SOURCE_DIR        ${SB_SOURCE_DIR}/${_proj_name}
  CMAKE_ARGS
    -DCMAKE_INSTALL_PREFIX:PATH=${SB_INSTALL_DIR} ${EXTRA_CMAKE_ARGS}
 
  #--Build step-----------------
  BINARY_DIR        ${_SB_BINARY_DIR}
  #--Install step---------------
  INSTALL_DIR       ${SB_INSTALL_DIR}
  #--Output logging-------------
  LOG_DOWNLOAD      OFF
  LOG_CONFIGURE     OFF
  LOG_BUILD         OFF
)
