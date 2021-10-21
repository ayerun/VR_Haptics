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

file(GLOB LOCAL_GRAPHICS_HEADERS "include/graphics/*.h")
file(GLOB LOCAL_HAPTICS_HEADERS "include/haptics/*.hpp")
file(GLOB LOCAL_GRAPHICS_SOURCE "src/graphics/*.cpp")
file(GLOB LOCAL_HAPTICS_SOURCE "src/haptics/*.cpp")
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


#Hand Tracking
add_executable(tracking
    "src/tracking_main.cpp"
    ${LOCAL_GRAPHICS_SOURCE}
    ${LOCAL_GRAPHICS_HEADERS}
    ${VULKAN_SHADERS})

compile_glsl(run_tracking_glsl_compiles ${VULKAN_SHADERS})

target_include_directories(tracking 
    PRIVATE OpenXR::Headers
    PRIVATE ${Vulkan_INCLUDE_DIRS})

target_link_libraries(tracking 
    OpenXR::openxr_loader
    ${Vulkan_LIBRARY})

install(TARGETS tracking
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    COMPONENT scene)


#Default Render Hello XR
add_executable(scene
    "src/scene_main.cpp"
    ${LOCAL_GRAPHICS_SOURCE}
    ${LOCAL_GRAPHICS_HEADERS}
    ${VULKAN_SHADERS})

compile_glsl(run_scene_glsl_compiles ${VULKAN_SHADERS})

add_dependencies(scene run_scene_glsl_compiles)

target_include_directories(scene 
    PRIVATE OpenXR::Headers
    PRIVATE ${Vulkan_INCLUDE_DIRS})

target_link_libraries(scene 
    OpenXR::openxr_loader
    ${Vulkan_LIBRARY})

install(TARGETS scene
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    COMPONENT scene)

    
#Render Hello XR and Communicate with motor
add_executable(haptic_scene
    "src/haptic_scene_main.cpp"
    ${LOCAL_GRAPHICS_SOURCE}
    ${LOCAL_GRAPHICS_HEADERS}
    ${LOCAL_HAPTICS_SOURCE}
    ${LOCAL_HAPTICS_HEADERS}
    ${VULKAN_SHADERS})

add_dependencies(haptic_scene run_scene_glsl_compiles)

target_include_directories(haptic_scene 
    PRIVATE OpenXR::Headers
    PRIVATE ${Vulkan_INCLUDE_DIRS})

target_link_libraries(haptic_scene 
    OpenXR::openxr_loader
    ${Vulkan_LIBRARY})

install(TARGETS haptic_scene
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    COMPONENT haptic_scene)

qt5_use_modules(haptic_scene Core SerialPort)


#Stop Motor
add_executable(stop_motor
    "src/motor_stop_main.cpp"
    ${LOCAL_HAPTICS_SOURCE}
    ${LOCAL_HAPTICS_HEADERS})

install(TARGETS stop_motor
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    COMPONENT stop_motor)

qt5_use_modules(stop_motor Core SerialPort)



#Encoder Wall
add_executable(encoder_wall
    "src/encoder_wall_main.cpp"
    ${LOCAL_HAPTICS_SOURCE}
    ${LOCAL_HAPTICS_HEADERS})

install(TARGETS encoder_wall
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    COMPONENT encoder_wall)

qt5_use_modules(encoder_wall Core SerialPort)