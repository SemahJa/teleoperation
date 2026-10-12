cmake_minimum_required(VERSION 3.22.1)
project(quest_teleop_android LANGUAGES C CXX)

set(QUEST_BRIDGE_PORT 45454 CACHE STRING "Loopback TCP port exposed by adb reverse")
if(NOT QUEST_BRIDGE_PORT MATCHES "^[0-9]+$" OR QUEST_BRIDGE_PORT LESS 1 OR QUEST_BRIDGE_PORT GREATER 65535)
  message(FATAL_ERROR "QUEST_BRIDGE_PORT must be in [1,65535]")
endif()

include(FetchContent)
set(BUILD_LOADER ON CACHE BOOL "Build pinned OpenXR loader" FORCE)
set(BUILD_API_LAYERS OFF CACHE BOOL "Do not build API layers" FORCE)
set(BUILD_CONFORMANCE_TESTS OFF CACHE BOOL "Do not build SDK conformance tests" FORCE)
FetchContent_Declare(
  openxr_sdk
  GIT_REPOSITORY https://github.com/KhronosGroup/OpenXR-SDK.git
  GIT_TAG 8899a91c17ce9618f565f42408b47db1d6e9ccc7
  GIT_SHALLOW FALSE
)
FetchContent_MakeAvailable(openxr_sdk)

add_library(quest_teleop_native SHARED native_activity.cpp openxr_program.cpp)
target_compile_features(quest_teleop_native PRIVATE cxx_std_17)
target_compile_definitions(quest_teleop_native PRIVATE
  XR_USE_PLATFORM_ANDROID XR_USE_GRAPHICS_API_OPENGL_ES
  QUEST_BRIDGE_PORT=${QUEST_BRIDGE_PORT})
target_compile_options(quest_teleop_native PRIVATE
  "$<$<COMPILE_LANGUAGE:CXX>:-Wall;-Wextra;-Werror;-Wno-missing-field-initializers>")
target_include_directories(quest_teleop_native PRIVATE
  "${ANDROID_NDK}/sources/android/native_app_glue")
target_sources(quest_teleop_native PRIVATE
  "${ANDROID_NDK}/sources/android/native_app_glue/android_native_app_glue.c")
target_link_libraries(quest_teleop_native PRIVATE
  OpenXR::openxr_loader android log EGL GLESv3)
