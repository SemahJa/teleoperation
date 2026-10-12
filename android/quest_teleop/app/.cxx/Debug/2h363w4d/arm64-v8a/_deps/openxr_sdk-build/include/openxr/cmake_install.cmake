# Install script for directory: /workspace/android/quest_teleop/app/.cxx/Debug/2h363w4d/arm64-v8a/_deps/openxr_sdk-src/include/openxr

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Debug")
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
  set(CMAKE_CROSSCOMPILING "TRUE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/opt/android-sdk/ndk/26.3.11579264/toolchains/llvm/prebuilt/linux-x86_64/bin/llvm-objdump")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xHeadersx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/openxr/openxr_platform_defines.h;/openxr/openxr.h;/openxr/openxr_loader_negotiation.h;/openxr/openxr_platform.h;/openxr/openxr_reflection.h;/openxr/openxr_reflection_structs.h;/openxr/openxr_reflection_parent_structs.h")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  file(INSTALL DESTINATION "/openxr" TYPE FILE FILES
    "/workspace/android/quest_teleop/app/.cxx/Debug/2h363w4d/arm64-v8a/_deps/openxr_sdk-src/include/openxr/openxr_platform_defines.h"
    "/workspace/android/quest_teleop/app/.cxx/Debug/2h363w4d/arm64-v8a/_deps/openxr_sdk-src/include/openxr/openxr.h"
    "/workspace/android/quest_teleop/app/.cxx/Debug/2h363w4d/arm64-v8a/_deps/openxr_sdk-src/include/openxr/openxr_loader_negotiation.h"
    "/workspace/android/quest_teleop/app/.cxx/Debug/2h363w4d/arm64-v8a/_deps/openxr_sdk-src/include/openxr/openxr_platform.h"
    "/workspace/android/quest_teleop/app/.cxx/Debug/2h363w4d/arm64-v8a/_deps/openxr_sdk-src/include/openxr/openxr_reflection.h"
    "/workspace/android/quest_teleop/app/.cxx/Debug/2h363w4d/arm64-v8a/_deps/openxr_sdk-src/include/openxr/openxr_reflection_structs.h"
    "/workspace/android/quest_teleop/app/.cxx/Debug/2h363w4d/arm64-v8a/_deps/openxr_sdk-src/include/openxr/openxr_reflection_parent_structs.h"
    )
endif()

