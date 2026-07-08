cmake_minimum_required(VERSION 3.16)
project(kinect-for-linux VERSION 1.0.0 LANGUAGES C CXX)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(PkgConfig REQUIRED)
find_package(Threads REQUIRED)

# ─── Dependencies ──────────────────────────────────
pkg_check_modules(LIBFREENECT REQUIRED libfreenect)
pkg_check_modules(LIBUSB REQUIRED libusb-1.0)
pkg_check_modules(PULSEAUDIO libpulse-simple)
pkg_check_modules(ALSA alsa)
pkg_check_modules(V4L2 libv4l2)

# Qt6 (for control app)
find_package(Qt6 COMPONENTS Core Widgets Network QUIET)
set(HAVE_QT6 ${Qt6_FOUND})

# OpenCV (optional, for skeleton tracking)
find_package(OpenCV COMPONENTS dnn objdetect imgproc highgui QUIET)
set(HAVE_OPENCV ${OpenCV_FOUND})

# OpenNI2 (optional, for driver plugin)
set(OPENNI2_DIR "/home/rexx/Proyectos/OpenNI2" CACHE PATH "OpenNI2 source dir")
set(HAVE_OPENNI2 FALSE)
if(EXISTS ${OPENNI2_DIR}/Include/OpenNI.h)
    set(HAVE_OPENNI2 TRUE)
endif()

# Kinect firmware binary
set(KINECT_FW_PATH "/usr/lib/firmware/kinect_uac_firmware.bin"
    CACHE FILEPATH "Path to Kinect UAC firmware binary")

# ─── Daemon (k4wd) ──────────────────────────────────
set(DAEMON_SOURCES
    src/daemon/main.c
    src/daemon/kinect.c
    src/daemon/motor.c
    src/daemon/firmware.c
    src/daemon/shm.c
    src/daemon/v4l2.c
    src/daemon/audio.c
    src/daemon/socket.c
    src/daemon/config.c
)

add_executable(k4wd ${DAEMON_SOURCES})
target_include_directories(k4wd PRIVATE
    ${CMAKE_SOURCE_DIR}/include
    ${LIBFREENECT_INCLUDE_DIRS}
    ${LIBUSB_INCLUDE_DIRS}
)
target_link_libraries(k4wd PRIVATE
    ${LIBFREENECT_LIBRARIES}
    freenect_sync
    ${LIBUSB_LIBRARIES}
    ${ALSA_LIBRARIES}
    Threads::Threads
    rt
    m
)
target_compile_options(k4wd PRIVATE -Wall -Wextra -Wpedantic)
target_compile_definitions(k4wd PRIVATE
    KINECT_FW_PATH="${KINECT_FW_PATH}"
)

if(PULSEAUDIO_FOUND)
    target_include_directories(k4wd PRIVATE ${PULSEAUDIO_INCLUDE_DIRS})
    target_link_libraries(k4wd PRIVATE ${PULSEAUDIO_LIBRARIES})
    target_compile_definitions(k4wd PRIVATE HAVE_PULSEAUDIO)
endif()

if(V4L2_FOUND)
    target_include_directories(k4wd PRIVATE ${V4L2_INCLUDE_DIRS})
    target_link_libraries(k4wd PRIVATE ${V4L2_LIBRARIES})
    target_compile_definitions(k4wd PRIVATE HAVE_V4L2)
endif()

# ─── Control App (kinect-for-linux) ──────────────────
if(HAVE_QT6)
    set(CTL_SOURCES
        src/ctl/main.cpp
        src/ctl/mainwindow.cpp
        src/ctl/tracker.cpp
    )

    add_executable(kinect-for-linux ${CTL_SOURCES})
    target_include_directories(kinect-for-linux PRIVATE ${CMAKE_SOURCE_DIR}/include)
    target_link_libraries(kinect-for-linux PRIVATE
        Qt6::Core Qt6::Widgets Qt6::Network
    )
    target_compile_options(kinect-for-linux PRIVATE -Wall -Wextra -Wno-unused-parameter)

    if(PULSEAUDIO_FOUND)
        target_include_directories(kinect-for-linux PRIVATE ${PULSEAUDIO_INCLUDE_DIRS})
        target_link_libraries(kinect-for-linux PRIVATE ${PULSEAUDIO_LIBRARIES})
        target_compile_definitions(kinect-for-linux PRIVATE HAVE_PULSEAUDIO)
    endif()

    if(HAVE_OPENCV)
        target_include_directories(kinect-for-linux PRIVATE ${OpenCV_INCLUDE_DIRS})
        target_link_libraries(kinect-for-linux PRIVATE ${OpenCV_LIBS})
        target_compile_definitions(kinect-for-linux PRIVATE HAVE_OPENCV)
    endif()
else()
    message(WARNING "Qt6 not found - kinect-for-linux GUI will not be built")
endif()

# ─── OpenNI2 Driver Plugin ──────────────────────────
if(HAVE_OPENNI2)
    add_library(K4WDriver SHARED src/openni2/K4WDriver.cpp)
    target_include_directories(K4WDriver PRIVATE
        ${OPENNI2_DIR}/Include
        ${OPENNI2_DIR}/Include/Driver
        ${OPENNI2_DIR}/ThirdParty/PSCommon/XnLib/Include
    )
    target_link_libraries(K4WDriver
        ${OPENNI2_DIR}/ThirdParty/PSCommon/XnLib/Bin/x64-Release/libXnLib.a
        dl pthread rt
    )
    target_compile_options(K4WDriver PRIVATE -fPIC -Wall -Wno-unused-parameter)
else()
    message(WARNING "OpenNI2 not found - driver plugin will not be built")
endif()

# ─── Install ────────────────────────────────────────
install(TARGETS k4wd DESTINATION bin)
if(HAVE_QT6)
    install(TARGETS kinect-for-linux DESTINATION bin)
    install(FILES kinect-for-linux.desktop DESTINATION share/applications)
    install(FILES icons/kinection-enabled.png
            DESTINATION share/icons/kinect-for-linux
            RENAME kinect-for-linux.png)
endif()
if(HAVE_OPENNI2)
    install(TARGETS K4WDriver DESTINATION lib/OpenNI2/Drivers)
endif()
install(FILES 90-kinect.rules DESTINATION /etc/udev/rules.d)
install(FILES v4l2loopback.conf DESTINATION /etc/modprobe.d)

# ─── Status ─────────────────────────────────────────
message(STATUS "")
message(STATUS "Kinect for Linux v${PROJECT_VERSION}")
message(STATUS "  Daemon (k4wd):           YES")
message(STATUS "  Control App:             ${HAVE_QT6}")
message(STATUS "  Skeleton Tracking:       ${HAVE_OPENCV}")
message(STATUS "  OpenNI2 Driver:          ${HAVE_OPENNI2}")
message(STATUS "  PulseAudio:              ${PULSEAUDIO_FOUND}")
message(STATUS "  V4L2 Loopback:           ${V4L2_FOUND}")
message(STATUS "")
