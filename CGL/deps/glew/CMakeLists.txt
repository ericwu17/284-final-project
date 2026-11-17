set(GLEW_ROOT_DIR ${CMAKE_CURRENT_SOURCE_DIR})

set(GLEW_SOURCE_DIR ${GLEW_ROOT_DIR}/src)
set(GLEW_HEADER_DIR ${GLEW_ROOT_DIR}/include)

# Always static: NanoGUI forces BUILD_SHARED_LIBS=ON for the whole project.
# glewinfo.c and visualinfo.c are standalone tools with their own main(), so
# only the loader itself is compiled.
add_library(glew STATIC ${GLEW_SOURCE_DIR}/glew.c)
target_include_directories(glew PUBLIC ${GLEW_HEADER_DIR})
target_compile_definitions(glew PUBLIC GLEW_STATIC)

find_package(OpenGL REQUIRED)
target_link_libraries(glew PUBLIC OpenGL::GL)

set(GLEW_LIBRARIES glew PARENT_SCOPE)
set(GLEW_INCLUDE_DIRS ${GLEW_HEADER_DIR} PARENT_SCOPE)
