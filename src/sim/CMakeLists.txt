# Simulator (from HW4 ClothSim)
set(CLOTHSIM_VIEWER_SOURCE
    # Cloth simulation objects
    cloth.cpp
    clothMesh.cpp

    # Collision objects
    collision/sphere.cpp
    collision/plane.cpp

    # Application
    main.cpp
    clothSimulator.cpp

    # Miscellaneous
    # png.cpp
    misc/sphere_drawing.cpp
    misc/file_utils.cpp
    misc/frame_capture.cpp

    # Camera
    camera.cpp
)

# Windows-only sources
if(WIN32)
list(APPEND CLOTHSIM_VIEWER_SOURCE
    # For get-opt
    misc/getopt.c
)
endif(WIN32)

#-------------------------------------------------------------------------------
# Embed resources
#-------------------------------------------------------------------------------

# embed_resources(CLOTHSIM_SHADERS
#   shaders/vert.glsl
#   shaders/normalFrag.glsl
#   shaders/wireframeFrag.glsl
# )

#-------------------------------------------------------------------------------
# Set definitions
#-------------------------------------------------------------------------------
add_definitions(${NANOGUI_EXTRA_DEFS})

#-------------------------------------------------------------------------------
# Set include directories
#-------------------------------------------------------------------------------
include_directories(
  ${CGL_INCLUDE_DIRS}
  ${FREETYPE_INCLUDE_DIRS}
  ${NANOGUI_EXTRA_INCS}
)

#-------------------------------------------------------------------------------
# Set link directories
#-------------------------------------------------------------------------------
link_directories(
  ${CGL_LIBRARY_DIRS}
  ${FREETYPE_LIBRARY_DIRS}
)

#-------------------------------------------------------------------------------
# Add executable
#-------------------------------------------------------------------------------
add_executable(clothsim ${CLOTHSIM_VIEWER_SOURCE})

target_link_libraries(clothsim
    jello_core
    CGL ${CGL_LIBRARIES}
    nanogui ${NANOGUI_EXTRA_LIBS}
    ${FREETYPE_LIBRARIES}
    ${CMAKE_THREADS_INIT}
)

#-------------------------------------------------------------------------------
# Platform-specific configurations for target
#-------------------------------------------------------------------------------
if(APPLE)
  set_property( TARGET clothsim APPEND_STRING PROPERTY COMPILE_FLAGS
                "-Wno-deprecated-declarations -Wno-c++11-extensions")
endif(APPLE)

# Install to project root
install(TARGETS clothsim DESTINATION ${Jello_SOURCE_DIR})

#-------------------------------------------------------------------------------
# Jello simulation core (no OpenGL / nanogui), shared by clothsim and jellosim
#-------------------------------------------------------------------------------
add_library(jello_core STATIC
    jello/tetMesh.cpp
    jello/softBody.cpp
    jello/jelloScene.cpp
)
target_include_directories(jello_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(jello_core PUBLIC CGL)

# Headless command-line driver: ./jellosim --selftest
add_executable(jellosim jelloMain.cpp)
target_link_libraries(jellosim jello_core)
