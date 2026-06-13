# refresh_git_version.cmake
# 每次构建时重新获取 Git 信息并更新 git_version.h
# 由 CMakeLists.txt 中的 RefreshGitVersion target 调用

if(NOT GIT_EXECUTABLE)
    set(GIT_EXECUTABLE git)
endif()

execute_process(
    COMMAND ${GIT_EXECUTABLE} rev-parse --short=8 HEAD
    WORKING_DIRECTORY ${SOURCE_DIR}
    OUTPUT_VARIABLE GIT_COMMIT_HASH_SHORT
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET
)

execute_process(
    COMMAND ${GIT_EXECUTABLE} describe --tags --abbrev=0
    WORKING_DIRECTORY ${SOURCE_DIR}
    OUTPUT_VARIABLE GIT_TAG
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET
)

if(NOT GIT_COMMIT_HASH_SHORT)
    set(GIT_VERSION_LABEL "unknown")
elseif(GIT_TAG)
    set(GIT_VERSION_LABEL "${GIT_TAG}")
else()
    set(GIT_VERSION_LABEL "${GIT_COMMIT_HASH_SHORT}")
endif()

# 仅内容变化时才写入，避免触发不必要的重新编译
set(NEW_CONTENT "/**\n * @file git_version.h\n * @brief 由 CMake 自动生成，包含 Git 版本信息\n * @warning 请勿手动修改此文件\n */\n\n#ifndef GIT_VERSION_H\n#define GIT_VERSION_H\n\n#define GIT_VERSION_LABEL        \"${GIT_VERSION_LABEL}\"\n\n#endif /* GIT_VERSION_H */\n")

if(EXISTS ${OUTPUT_FILE})
    file(READ ${OUTPUT_FILE} OLD_CONTENT)
else()
    set(OLD_CONTENT "")
endif()

if(NOT NEW_CONTENT STREQUAL OLD_CONTENT)
    file(WRITE ${OUTPUT_FILE} "${NEW_CONTENT}")
    message(STATUS "Git version header updated: ${GIT_VERSION_LABEL}")
endif()
