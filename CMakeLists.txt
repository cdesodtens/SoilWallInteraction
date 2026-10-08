cmake_minimum_required(VERSION 3.16)
project(Verlet-Multithread LANGUAGES CXX)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
option(BUILD_SHARED_LIBS "Build shared libraries" OFF)

include(FetchContent)
FetchContent_Declare(SFML
    GIT_REPOSITORY https://github.com/SFML/SFML.git
    GIT_TAG 2.6.x)
FetchContent_MakeAvailable(SFML)

file(GLOB_RECURSE engine_sources "src/renderer/*.cpp" "src/engine/*.cpp" "src/physics/*.cpp" "src/thread_pool/*.cpp")

add_library(VerletCore STATIC ${engine_sources})
target_include_directories(VerletCore PUBLIC "src" "engine")
target_link_libraries(VerletCore PUBLIC sfml-graphics)
target_compile_features(VerletCore PUBLIC cxx_std_17)

# Wall Application
add_executable(VerletWall src/app_wall.cpp)
target_link_libraries(VerletWall PRIVATE VerletCore)

# Foundation Application
add_executable(VerletFoundation src/app_foundation.cpp)
target_link_libraries(VerletFoundation PRIVATE VerletCore)

# Copy res dir to the binary directory
add_custom_command(
    TARGET VerletWall
    COMMENT "Copy Res directory"
    PRE_BUILD COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_SOURCE_DIR}/res $<TARGET_FILE_DIR:VerletWall>/res
    VERBATIM)

add_custom_command(
    TARGET VerletFoundation
    COMMENT "Copy Res directory"
    PRE_BUILD COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_SOURCE_DIR}/res $<TARGET_FILE_DIR:VerletFoundation>/res
    VERBATIM)

if(WIN32)
    add_custom_command(
        TARGET VerletWall
        COMMENT "Copy OpenAL DLL"
        PRE_BUILD COMMAND ${CMAKE_COMMAND} -E copy ${SFML_SOURCE_DIR}/extlibs/bin/$<IF:$<EQUAL:${CMAKE_SIZEOF_VOID_P},8>,x64,x86>/openal32.dll $<TARGET_FILE_DIR:VerletWall>
        VERBATIM)
    add_custom_command(
        TARGET VerletFoundation
        COMMENT "Copy OpenAL DLL"
        PRE_BUILD COMMAND ${CMAKE_COMMAND} -E copy ${SFML_SOURCE_DIR}/extlibs/bin/$<IF:$<EQUAL:${CMAKE_SIZEOF_VOID_P},8>,x64,x86>/openal32.dll $<TARGET_FILE_DIR:VerletFoundation>
        VERBATIM)
endif()