cmake_minimum_required(VERSION 3.22)

if (WIN32)
    set(JLINK_DEIVCE_DIR $ENV{APPDATA}/Segger/JLinkDevices CACHE STRING "JlinkDevices目录")
endif()

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)
set(CMAKE_INCLUDE_CURRENT_DIR TRUE)

project(HFLink C CXX ASM)

# ============================================================
# 获取 Git 版本信息
# ============================================================
find_package(Git QUIET)
if(GIT_FOUND)
    execute_process(
        COMMAND ${GIT_EXECUTABLE} rev-parse --short=8 HEAD
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        OUTPUT_VARIABLE GIT_COMMIT_HASH_SHORT
        OUTPUT_STRIP_TRAILING_WHITESPACE
        ERROR_QUIET
    )
    execute_process(
        COMMAND ${GIT_EXECUTABLE} describe --tags --abbrev=0
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        OUTPUT_VARIABLE GIT_TAG
        OUTPUT_STRIP_TRAILING_WHITESPACE
        ERROR_QUIET
    )

    # 版本标签: 有 tag 则用 tag，否则用短哈希
    if(GIT_TAG)
        set(GIT_VERSION_LABEL "${GIT_TAG}")
    else()
        set(GIT_VERSION_LABEL "${GIT_COMMIT_HASH_SHORT}")
    endif()
else()
    set(GIT_VERSION_LABEL "unknown")
endif()

message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "Version: ${VERSION_STR}")
message(STATUS "Git Version: ${GIT_VERSION_LABEL}")

# 生成 git_version.h 头文件，供 C 代码使用
set(GIT_VERSION_H ${CMAKE_BINARY_DIR}/generated/git_version.h)

# 初次 configure 时生成
configure_file(
    ${CMAKE_SOURCE_DIR}/cmake/git_version.h.in
    ${GIT_VERSION_H}
    @ONLY
)

# 每次构建时自动刷新（检测 Git 状态变化）
add_custom_target(RefreshGitVersion ALL
    COMMAND ${CMAKE_COMMAND}
        -DGIT_EXECUTABLE=${GIT_EXECUTABLE}
        -DSOURCE_DIR=${CMAKE_SOURCE_DIR}
        -DOUTPUT_FILE=${GIT_VERSION_H}
        -DTEMPLATE_FILE=${CMAKE_SOURCE_DIR}/cmake/git_version.h.in
        -P ${CMAKE_SOURCE_DIR}/cmake/refresh_git_version.cmake
    COMMENT "Refreshing git version header..."
)

add_subdirectory(Firmware)
add_subdirectory(Third_Party/SEGGER_RTT_V812)

add_subdirectory(Application)
