set(_proj_name opengv)
set(_SB_BINARY_DIR "${SB_BINARY_DIR}/${_proj_name}")

if(CYGWIN)
  set(EXTRA_CMAKE_ARGS -DWIN32=ON)
endif()

ExternalProject_Add(${_proj_name}
  PREFIX            ${_SB_BINARY_DIR}
  TMP_DIR           ${_SB_BINARY_DIR}/tmp
  STAMP_DIR         ${_SB_BINARY_DIR}/stamp
  #--Download step--------------
  DOWNLOAD_DIR      ${SB_DOWNLOAD_DIR}
  URL               https://github.com/paulinus/opengv/archive/7436794df04d85433a966395088e38b107e69fc2.zip
  URL_MD5           9b303c3ab9f210b242941e851572d2c8
  #--Update/Patch step----------
  UPDATE_COMMAND    ""
  #--Configure step-------------
  SOURCE_DIR        ${SB_SOURCE_DIR}/${_proj_name}
  CMAKE_ARGS
    -DBUILD_TESTS=OFF 
    -DBUILD_PYTHON=ON
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

