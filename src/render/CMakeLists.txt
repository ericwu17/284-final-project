# Path tracer (from HW3 PathTracer)

set(IMGUI_DIR ${PROJECT_SOURCE_DIR}/ext/imgui)

set(PATHTRACER_SOURCE
    # Scene objects and acceleration structure
    scene/sphere.cpp
    scene/triangle.cpp
    scene/light.cpp
    scene/bvh.cpp
    scene/bbox.cpp
    scene/object.cpp
    scene/environment_light.cpp

    # Path tracer
    pathtracer/camera.cpp
    pathtracer/camera_lens.cpp
    pathtracer/bsdf.cpp
    pathtracer/advanced_bsdf.cpp
    pathtracer/pathtracer.cpp
    pathtracer/sampler.cpp
    pathtracer/raytraced_renderer.cpp

    # COLLADA (.dae) parser
    scene/collada/collada.cpp
    scene/collada/camera_info.cpp
    scene/collada/light_info.cpp
    scene/collada/sphere_info.cpp
    scene/collada/polymesh_info.cpp
    scene/collada/material_info.cpp

    # OpenGL preview scene
    scene/gl_scene/mesh.cpp
    scene/gl_scene/scene.cpp
    scene/gl_scene/sphere.cpp

    # MeshEdit
    util/halfEdgeMesh.cpp
    application/meshEdit.cpp

    # Misc
    util/sphere_drawing.cpp
    util/lodepng.cpp

    # Application
    application/application.cpp
    application/main.cpp
    application/visual_debugger.cpp
    application/pathtracer_launcher_gui.cpp

    # Dear ImGui
    ${IMGUI_DIR}/imgui.cpp
    ${IMGUI_DIR}/imgui_widgets.cpp
    ${IMGUI_DIR}/imgui_draw.cpp
    ${IMGUI_DIR}/imgui_tables.cpp
    ${IMGUI_DIR}/backends/imgui_impl_glfw.cpp
    ${IMGUI_DIR}/backends/imgui_impl_opengl2.cpp
)

if(WIN32)
  list(APPEND PATHTRACER_SOURCE util/win32/getopt.c)
endif()

add_executable(pathtracer ${PATHTRACER_SOURCE})

target_include_directories(pathtracer PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${IMGUI_DIR}
    ${IMGUI_DIR}/backends
)

find_package(OpenGL REQUIRED)
target_link_libraries(pathtracer PRIVATE CGL OpenGL::GL)
if(TARGET OpenGL::GLU)
  target_link_libraries(pathtracer PRIVATE OpenGL::GLU)
endif()

if(APPLE)
  # OpenGL and GLU are deprecated on macOS but still work
  target_compile_options(pathtracer PRIVATE -Wno-deprecated-declarations)
endif()

install(TARGETS pathtracer DESTINATION ${Jello_SOURCE_DIR})
