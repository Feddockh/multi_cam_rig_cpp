# Install script for directory: /home/hayden/ros_ws/src/multi_cam_rig_cpp

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/hayden/ros_ws/src/multi_cam_rig_cpp/install/multi_cam_rig_cpp")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/multi_cam_rig_cpp/director_gui" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/multi_cam_rig_cpp/director_gui")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/multi_cam_rig_cpp/director_gui"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/multi_cam_rig_cpp" TYPE EXECUTABLE FILES "/home/hayden/ros_ws/src/multi_cam_rig_cpp/build/multi_cam_rig_cpp/director_gui")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/multi_cam_rig_cpp/director_gui" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/multi_cam_rig_cpp/director_gui")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/multi_cam_rig_cpp/director_gui"
         OLD_RPATH "/usr/lib/libm3api.so:/usr/local/zed/lib/libsl_zed.so:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_calib3d:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_core:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_dnn:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_features2d:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_flann:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_highgui:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_imgcodecs:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_imgproc:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_ml:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_objdetect:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_photo:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_stitching:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_video:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_videoio:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_alphamat:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_aruco:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_barcode:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_bgsegm:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_bioinspired:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_ccalib:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_datasets:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_dnn_objdetect:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_dnn_superres:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_dpm:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_face:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_freetype:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_fuzzy:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_hdf:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_hfs:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_img_hash:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_intensity_transform:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_line_descriptor:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_mcc:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_optflow:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_phase_unwrapping:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_plot:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_quality:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_rapid:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_reg:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_rgbd:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_saliency:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_shape:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_stereo:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_structured_light:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_superres:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_surface_matching:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_text:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_tracking:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_videostab:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_viz:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_wechat_qrcode:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_ximgproc:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_xobjdetect:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_xphoto:/opt/ros/humble/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/multi_cam_rig_cpp/director_gui")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/multi_cam_rig_cpp/firefly_capture_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/multi_cam_rig_cpp/firefly_capture_node")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/multi_cam_rig_cpp/firefly_capture_node"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/multi_cam_rig_cpp" TYPE EXECUTABLE FILES "/home/hayden/ros_ws/src/multi_cam_rig_cpp/build/multi_cam_rig_cpp/firefly_capture_node")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/multi_cam_rig_cpp/firefly_capture_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/multi_cam_rig_cpp/firefly_capture_node")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/multi_cam_rig_cpp/firefly_capture_node"
         OLD_RPATH "/usr/lib/libm3api.so:/usr/local/zed/lib/libsl_zed.so:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_calib3d:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_core:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_dnn:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_features2d:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_flann:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_highgui:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_imgcodecs:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_imgproc:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_ml:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_objdetect:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_photo:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_stitching:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_video:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_videoio:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_alphamat:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_aruco:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_barcode:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_bgsegm:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_bioinspired:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_ccalib:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_datasets:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_dnn_objdetect:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_dnn_superres:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_dpm:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_face:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_freetype:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_fuzzy:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_hdf:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_hfs:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_img_hash:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_intensity_transform:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_line_descriptor:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_mcc:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_optflow:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_phase_unwrapping:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_plot:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_quality:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_rapid:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_reg:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_rgbd:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_saliency:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_shape:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_stereo:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_structured_light:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_superres:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_surface_matching:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_text:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_tracking:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_videostab:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_viz:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_wechat_qrcode:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_ximgproc:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_xobjdetect:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_xphoto:/opt/ros/humble/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/multi_cam_rig_cpp/firefly_capture_node")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/multi_cam_rig_cpp/ximea_capture_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/multi_cam_rig_cpp/ximea_capture_node")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/multi_cam_rig_cpp/ximea_capture_node"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/multi_cam_rig_cpp" TYPE EXECUTABLE FILES "/home/hayden/ros_ws/src/multi_cam_rig_cpp/build/multi_cam_rig_cpp/ximea_capture_node")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/multi_cam_rig_cpp/ximea_capture_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/multi_cam_rig_cpp/ximea_capture_node")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/multi_cam_rig_cpp/ximea_capture_node"
         OLD_RPATH "/usr/lib/libm3api.so:/usr/local/zed/lib/libsl_zed.so:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_calib3d:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_core:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_dnn:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_features2d:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_flann:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_highgui:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_imgcodecs:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_imgproc:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_ml:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_objdetect:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_photo:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_stitching:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_video:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_videoio:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_alphamat:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_aruco:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_barcode:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_bgsegm:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_bioinspired:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_ccalib:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_datasets:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_dnn_objdetect:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_dnn_superres:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_dpm:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_face:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_freetype:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_fuzzy:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_hdf:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_hfs:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_img_hash:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_intensity_transform:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_line_descriptor:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_mcc:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_optflow:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_phase_unwrapping:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_plot:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_quality:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_rapid:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_reg:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_rgbd:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_saliency:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_shape:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_stereo:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_structured_light:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_superres:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_surface_matching:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_text:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_tracking:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_videostab:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_viz:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_wechat_qrcode:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_ximgproc:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_xobjdetect:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_xphoto:/opt/ros/humble/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/multi_cam_rig_cpp/ximea_capture_node")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/multi_cam_rig_cpp/zed_capture_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/multi_cam_rig_cpp/zed_capture_node")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/multi_cam_rig_cpp/zed_capture_node"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/multi_cam_rig_cpp" TYPE EXECUTABLE FILES "/home/hayden/ros_ws/src/multi_cam_rig_cpp/build/multi_cam_rig_cpp/zed_capture_node")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/multi_cam_rig_cpp/zed_capture_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/multi_cam_rig_cpp/zed_capture_node")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/multi_cam_rig_cpp/zed_capture_node"
         OLD_RPATH "/usr/lib/libm3api.so:/usr/local/zed/lib/libsl_zed.so:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_calib3d:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_core:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_dnn:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_features2d:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_flann:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_highgui:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_imgcodecs:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_imgproc:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_ml:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_objdetect:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_photo:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_stitching:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_video:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_videoio:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_alphamat:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_aruco:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_barcode:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_bgsegm:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_bioinspired:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_ccalib:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_datasets:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_dnn_objdetect:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_dnn_superres:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_dpm:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_face:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_freetype:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_fuzzy:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_hdf:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_hfs:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_img_hash:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_intensity_transform:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_line_descriptor:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_mcc:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_optflow:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_phase_unwrapping:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_plot:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_quality:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_rapid:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_reg:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_rgbd:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_saliency:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_shape:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_stereo:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_structured_light:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_superres:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_surface_matching:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_text:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_tracking:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_videostab:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_viz:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_wechat_qrcode:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_ximgproc:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_xobjdetect:/home/hayden/ros_ws/src/multi_cam_rig_cpp/opencv_xphoto:/opt/ros/humble/lib:/usr/local/zed/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/multi_cam_rig_cpp/zed_capture_node")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/multi_cam_rig_cpp" TYPE DIRECTORY FILES "/home/hayden/ros_ws/src/multi_cam_rig_cpp/launch")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/multi_cam_rig_cpp" TYPE DIRECTORY FILES "/home/hayden/ros_ws/src/multi_cam_rig_cpp/config")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/multi_cam_rig_cpp" TYPE DIRECTORY FILES "/home/hayden/ros_ws/src/multi_cam_rig_cpp/calibration")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/package_run_dependencies" TYPE FILE FILES "/home/hayden/ros_ws/src/multi_cam_rig_cpp/build/multi_cam_rig_cpp/ament_cmake_index/share/ament_index/resource_index/package_run_dependencies/multi_cam_rig_cpp")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/parent_prefix_path" TYPE FILE FILES "/home/hayden/ros_ws/src/multi_cam_rig_cpp/build/multi_cam_rig_cpp/ament_cmake_index/share/ament_index/resource_index/parent_prefix_path/multi_cam_rig_cpp")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/multi_cam_rig_cpp/environment" TYPE FILE FILES "/opt/ros/humble/share/ament_cmake_core/cmake/environment_hooks/environment/ament_prefix_path.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/multi_cam_rig_cpp/environment" TYPE FILE FILES "/home/hayden/ros_ws/src/multi_cam_rig_cpp/build/multi_cam_rig_cpp/ament_cmake_environment_hooks/ament_prefix_path.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/multi_cam_rig_cpp/environment" TYPE FILE FILES "/opt/ros/humble/share/ament_cmake_core/cmake/environment_hooks/environment/path.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/multi_cam_rig_cpp/environment" TYPE FILE FILES "/home/hayden/ros_ws/src/multi_cam_rig_cpp/build/multi_cam_rig_cpp/ament_cmake_environment_hooks/path.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/multi_cam_rig_cpp" TYPE FILE FILES "/home/hayden/ros_ws/src/multi_cam_rig_cpp/build/multi_cam_rig_cpp/ament_cmake_environment_hooks/local_setup.bash")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/multi_cam_rig_cpp" TYPE FILE FILES "/home/hayden/ros_ws/src/multi_cam_rig_cpp/build/multi_cam_rig_cpp/ament_cmake_environment_hooks/local_setup.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/multi_cam_rig_cpp" TYPE FILE FILES "/home/hayden/ros_ws/src/multi_cam_rig_cpp/build/multi_cam_rig_cpp/ament_cmake_environment_hooks/local_setup.zsh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/multi_cam_rig_cpp" TYPE FILE FILES "/home/hayden/ros_ws/src/multi_cam_rig_cpp/build/multi_cam_rig_cpp/ament_cmake_environment_hooks/local_setup.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/multi_cam_rig_cpp" TYPE FILE FILES "/home/hayden/ros_ws/src/multi_cam_rig_cpp/build/multi_cam_rig_cpp/ament_cmake_environment_hooks/package.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/packages" TYPE FILE FILES "/home/hayden/ros_ws/src/multi_cam_rig_cpp/build/multi_cam_rig_cpp/ament_cmake_index/share/ament_index/resource_index/packages/multi_cam_rig_cpp")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/multi_cam_rig_cpp/cmake" TYPE FILE FILES
    "/home/hayden/ros_ws/src/multi_cam_rig_cpp/build/multi_cam_rig_cpp/ament_cmake_core/multi_cam_rig_cppConfig.cmake"
    "/home/hayden/ros_ws/src/multi_cam_rig_cpp/build/multi_cam_rig_cpp/ament_cmake_core/multi_cam_rig_cppConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/multi_cam_rig_cpp" TYPE FILE FILES "/home/hayden/ros_ws/src/multi_cam_rig_cpp/package.xml")
endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/hayden/ros_ws/src/multi_cam_rig_cpp/build/multi_cam_rig_cpp/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
