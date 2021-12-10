cmake_minimum_required(VERSION 3.1.0)
project(VR_Haptics VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_VERSION VERSION_LESS "3.7.0")
    set(CMAKE_INCLUDE_CURRENT_DIR ON)
endif()

find_package(OpenXR REQUIRED)
find_package(Vulkan REQUIRED)
find_package(nuhal REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

add_definitions(-DXR_USE_GRAPHICS_API_VULKAN)

file(GLOB LOCAL_GRAPHICS_HEADERS "include/graphics/*.h")
file(GLOB LOCAL_GRAPHICS_SOURCE "src/graphics/*.cpp")
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
include_directories("include/graphics" "include/haptics" ${CMAKE_CURRENT_BINARY_DIR})

# Compile shaders
compile_glsl(run_glsl_compiles ${VULKAN_SHADERS})

# Declare haptics library
add_library(haptics
    src/haptics/haptics.cpp
    src/haptics/motor_communication.cpp
)

target_link_libraries(haptics
    Eigen3::Eigen
    nuhal
)


# VR Spring - render wall as spring based on VR tracking
add_executable(vr_spring
    src/vr_spring_main.cpp
    ${LOCAL_GRAPHICS_SOURCE}
    ${LOCAL_GRAPHICS_HEADERS}
    ${VULKAN_SHADERS})

add_dependencies(vr_spring run_glsl_compiles)

target_include_directories(vr_spring 
    PRIVATE OpenXR::Headers
    PRIVATE ${Vulkan_INCLUDE_DIRS})

target_link_libraries(vr_spring 
    haptics
    OpenXR::openxr_loader
    ${Vulkan_LIBRARY})

install(TARGETS vr_spring
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    COMPONENT vr_spring)


# Drumkit - haptic drum demo
add_executable(drumkit
src/drumkit_main.cpp
${LOCAL_GRAPHICS_SOURCE}
${LOCAL_GRAPHICS_HEADERS}
${VULKAN_SHADERS})

add_dependencies(drumkit run_glsl_compiles)

target_include_directories(drumkit 
PRIVATE OpenXR::Headers
PRIVATE ${Vulkan_INCLUDE_DIRS})

target_link_libraries(drumkit 
haptics
OpenXR::openxr_loader
${Vulkan_LIBRARY})

install(TARGETS drumkit
RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
COMPONENT drumkit)



# Encoder Spring Demo
add_executable(encoder_spring
    src/encoder_spring_main.cpp
)

target_link_libraries(encoder_spring
    haptics
)

install(TARGETS encoder_spring
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    COMPONENT encoder_spring)