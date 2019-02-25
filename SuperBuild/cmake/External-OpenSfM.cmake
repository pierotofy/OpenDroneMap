set(_proj_name opensfm)
set(_SB_BINARY_DIR "${SB_BINARY_DIR}/${_proj_name}")

set (OPENSFM_DEPENDS ceres opengv opencv)
set (OPENSFM_CONFIGURE_COMMAND cmake <SOURCE_DIR>/${_proj_name}/src -DCERES_ROOT_DIR=${SB_INSTALL_DIR} -DGLOG_INCLUDE_DIR=${SB_INSTALL_DIR}/include -DGLOG_LIBRARY=${SB_INSTALL_DIR}/lib/libglog.a -DOPENSFM_BUILD_TESTS=off -DOpenCV_DIR=${SB_INSTALL_DIR}/share/OpenCV)

if (CYGWIN)
  set (OPENSFM_DEPENDS ${OPENSFM_DEPENDS} glog gflags)
  set (OPENSFM_CONFIGURE_COMMAND ${OPENSFM_CONFIGURE_COMMAND} -DCMAKE_CXX_FLAGS="-Wa,-mbig-obj -DM_PI=3.14159265358979323846")
  set (OPENSFM_INSTALL_COMMAND ln -s ${_SB_BINARY_DIR}/csfm.so ${SB_SOURCE_DIR}/${_proj_name}/opensfm/csfm.dll )
endif()

ExternalProject_Add(${_proj_name}
  DEPENDS           ${OPENSFM_DEPENDS}
  PREFIX            ${_SB_BINARY_DIR}
  TMP_DIR           ${_SB_BINARY_DIR}/tmp
  STAMP_DIR         ${_SB_BINARY_DIR}/stamp
  #--Download step--------------
  DOWNLOAD_DIR      ${SB_DOWNLOAD_DIR}
  URL               https://github.com/OpenDroneMap/OpenSfM/archive/9b4b17f238a3762c4267cdaeb5f64173c0f704a6.zip
  #--Update/Patch step----------
  UPDATE_COMMAND    ""
  #--Configure step-------------
  SOURCE_DIR        ${SB_SOURCE_DIR}/${_proj_name}
  CONFIGURE_COMMAND ${OPENSFM_CONFIGURE_COMMAND}
  
  #--Build step-----------------
  BINARY_DIR        ${_SB_BINARY_DIR}
  #--Install step---------------
  INSTALL_COMMAND    ${OPENSFM_INSTALL_COMMAND}
  #--Output logging-------------
  LOG_DOWNLOAD      OFF
  LOG_CONFIGURE     OFF
  LOG_BUILD         OFF
)
