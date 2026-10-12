
if(NOT "/workspace/android/quest_teleop/app/.cxx/Debug/4e2d436t/arm64-v8a/_deps/openxr_sdk-subbuild/openxr_sdk-populate-prefix/src/openxr_sdk-populate-stamp/openxr_sdk-populate-gitinfo.txt" IS_NEWER_THAN "/workspace/android/quest_teleop/app/.cxx/Debug/4e2d436t/arm64-v8a/_deps/openxr_sdk-subbuild/openxr_sdk-populate-prefix/src/openxr_sdk-populate-stamp/openxr_sdk-populate-gitclone-lastrun.txt")
  message(STATUS "Avoiding repeated git clone, stamp file is up to date: '/workspace/android/quest_teleop/app/.cxx/Debug/4e2d436t/arm64-v8a/_deps/openxr_sdk-subbuild/openxr_sdk-populate-prefix/src/openxr_sdk-populate-stamp/openxr_sdk-populate-gitclone-lastrun.txt'")
  return()
endif()

execute_process(
  COMMAND ${CMAKE_COMMAND} -E rm -rf "/workspace/android/quest_teleop/app/.cxx/Debug/4e2d436t/arm64-v8a/_deps/openxr_sdk-src"
  RESULT_VARIABLE error_code
  )
if(error_code)
  message(FATAL_ERROR "Failed to remove directory: '/workspace/android/quest_teleop/app/.cxx/Debug/4e2d436t/arm64-v8a/_deps/openxr_sdk-src'")
endif()

# try the clone 3 times in case there is an odd git clone issue
set(error_code 1)
set(number_of_tries 0)
while(error_code AND number_of_tries LESS 3)
  execute_process(
    COMMAND "/usr/bin/git"  clone --no-checkout --config "advice.detachedHead=false" "https://github.com/KhronosGroup/OpenXR-SDK.git" "openxr_sdk-src"
    WORKING_DIRECTORY "/workspace/android/quest_teleop/app/.cxx/Debug/4e2d436t/arm64-v8a/_deps"
    RESULT_VARIABLE error_code
    )
  math(EXPR number_of_tries "${number_of_tries} + 1")
endwhile()
if(number_of_tries GREATER 1)
  message(STATUS "Had to git clone more than once:
          ${number_of_tries} times.")
endif()
if(error_code)
  message(FATAL_ERROR "Failed to clone repository: 'https://github.com/KhronosGroup/OpenXR-SDK.git'")
endif()

execute_process(
  COMMAND "/usr/bin/git"  checkout 8899a91c17ce9618f565f42408b47db1d6e9ccc7 --
  WORKING_DIRECTORY "/workspace/android/quest_teleop/app/.cxx/Debug/4e2d436t/arm64-v8a/_deps/openxr_sdk-src"
  RESULT_VARIABLE error_code
  )
if(error_code)
  message(FATAL_ERROR "Failed to checkout tag: '8899a91c17ce9618f565f42408b47db1d6e9ccc7'")
endif()

set(init_submodules TRUE)
if(init_submodules)
  execute_process(
    COMMAND "/usr/bin/git"  submodule update --recursive --init 
    WORKING_DIRECTORY "/workspace/android/quest_teleop/app/.cxx/Debug/4e2d436t/arm64-v8a/_deps/openxr_sdk-src"
    RESULT_VARIABLE error_code
    )
endif()
if(error_code)
  message(FATAL_ERROR "Failed to update submodules in: '/workspace/android/quest_teleop/app/.cxx/Debug/4e2d436t/arm64-v8a/_deps/openxr_sdk-src'")
endif()

# Complete success, update the script-last-run stamp file:
#
execute_process(
  COMMAND ${CMAKE_COMMAND} -E copy
    "/workspace/android/quest_teleop/app/.cxx/Debug/4e2d436t/arm64-v8a/_deps/openxr_sdk-subbuild/openxr_sdk-populate-prefix/src/openxr_sdk-populate-stamp/openxr_sdk-populate-gitinfo.txt"
    "/workspace/android/quest_teleop/app/.cxx/Debug/4e2d436t/arm64-v8a/_deps/openxr_sdk-subbuild/openxr_sdk-populate-prefix/src/openxr_sdk-populate-stamp/openxr_sdk-populate-gitclone-lastrun.txt"
  RESULT_VARIABLE error_code
  )
if(error_code)
  message(FATAL_ERROR "Failed to copy script-last-run stamp file: '/workspace/android/quest_teleop/app/.cxx/Debug/4e2d436t/arm64-v8a/_deps/openxr_sdk-subbuild/openxr_sdk-populate-prefix/src/openxr_sdk-populate-stamp/openxr_sdk-populate-gitclone-lastrun.txt'")
endif()

