cmake_minimum_required(VERSION 3.1.0)
project(VR_Haptics VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

if(CMAKE_VERSION VERSION_LESS "3.7.0")
    set(CMAKE_INCLUDE_CURRENT_DIR ON)
endif()

find_package(OpenXR REQUIRED)
find_package(Vulkan REQUIRED)
find_package(Qt5Core REQUIRED)
find_package(Qt5SerialPort REQUIRED)

add_definitions(-DXR_USE_GRAPHICS_API_VULKAN)

file(GLOB LOCAL_HEADERS "include/*.h")
file(GLOB LOCAL_SOURCE "src/*.cpp")
file(GLOB VULKAN_SHADERS "vulkan_shaders/*.glsl")

function(compile_glsl run_target_name)
    set(glsl_output_files "")
    foreach(in_file IN LISTS ARGN)
        get_filename_component(glsl_stage ${in_file} NAME_WE)
        set(out_file ${CMAKE_CURRENT_BINARY_DIR}/${glsl_stage}.spv)

        # Use the precompiled .spv files
        get_filename_component(glsl_src_dir ${in_file} DIRECTORY)
        set(precompiled_file ${glsl_src_dir}/${glsl_stage}.spv)
        configure_file(${precompiled_file} ${out_file} COPYONLY)

        list(APPEND glsl_output_files ${out_file})
    endforeach()
    add_custom_target(${run_target_name} ALL DEPENDS ${glsl_output_files})
endfunction()

# For including compiled shaders
include_directories("include" ${CMAKE_CURRENT_BINARY_DIR})

add_executable(scene
    "src/main.cc"
    ${LOCAL_SOURCE}
    ${LOCAL_HEADERS}
    ${VULKAN_SHADERS})

add_executable(motor_scene
    "src/motor_main.cc"
    "src/motor_communication.cc"
    "include/motor_communication.hpp"
    ${LOCAL_SOURCE}
    ${LOCAL_HEADERS}
    ${VULKAN_SHADERS})

compile_glsl(run_scene_glsl_compiles ${VULKAN_SHADERS})

add_dependencies(scene run_scene_glsl_compiles)
add_dependencies(motor_scene run_scene_glsl_compiles)

target_include_directories(scene 
    PRIVATE OpenXR::Headers
    PRIVATE ${Vulkan_INCLUDE_DIRS})

target_include_directories(motor_scene 
    PRIVATE OpenXR::Headers
    PRIVATE ${Vulkan_INCLUDE_DIRS})

target_link_libraries(scene 
    OpenXR::openxr_loader
    ${Vulkan_LIBRARY})

target_link_libraries(motor_scene 
    OpenXR::openxr_loader
    ${Vulkan_LIBRARY})

install(TARGETS scene
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    COMPONENT scene)

install(TARGETS motor_scene
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    COMPONENT motor_scene)

qt5_use_modules(motor_scene Core SerialPort)