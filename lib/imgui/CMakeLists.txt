file(GLOB HEADER_LIST CONFIGURE_DEPENDS "${mrcap_SOURCE_DIR}/include/imgui/*.h")
  
# SET(IMGUI_HEADER_LIST
#   ${mrcap_SOURCE_DIRECTORY}/include/imgui/glfw3.h
#   ${mrcap_SOURCE_DIRECTORY}/include/imgui/imgui_impl_opengl2.h
#   ${mrcap_SOURCE_DIRECTORY}/include/imgui/imgui.h
#   ${mrcap_SOURCE_DIRECTORY}/include/imgui/imgui_impl_opengl3.h
#   ${mrcap_SOURCE_DIRECTORY}/include/imgui/imgui_internal.h
#   ${mrcap_SOURCE_DIRECTORY}/include/imgui/imconfig.h
#   ${mrcap_SOURCE_DIRECTORY}/include/imgui/imstb_rectpack.h
#   ${mrcap_SOURCE_DIRECTORY}/include/imgui/imstb_textedit.h
#   ${mrcap_SOURCE_DIRECTORY}/include/imgui/imstb_truetype.h
#   )

SET(IMGUI_HEADER_LIST
  ../../include/glad/glad.h
  ../../include/imgui/glfw3.h
  ../../include/imgui/imgui.h
  ../../include/imgui/imconfig.h
  ../../include/imgui/imgui_impl_glfw.h
  ../../include/imgui/imgui_impl_opengl2.h
  ../../include/imgui/imgui_impl_opengl3.h
  ../../include/imgui/imgui_internal.h
  ../../include/imgui/imgui_stdlib.h
  ../../include/imgui/imstb_rectpack.h
  ../../include/imgui/imstb_textedit.h
  ../../include/imgui/imstb_truetype.h
  )

add_library(
    imgui
    imgui_demo.cpp
    imgui_draw.cpp
    imgui_impl_glfw.cpp
    imgui_impl_opengl2.cpp
    imgui_impl_opengl3.cpp
    imgui_tables.cpp
    imgui_widgets.cpp
    imgui.cpp
    imgui_stdlib.cpp
    ${IMGUI_HEADER_LIST}
    )


SET(OpenGL_GL_PREFERENCE "GLVND")

find_package(OpenGL REQUIRED)
find_package(glfw3 REQUIRED)
find_package(GLEW REQUIRED)
find_package(X11 REQUIRED)
find_package(Threads REQUIRED)

include_directories(${X11_INCLUDE_DIRS})
include_directories(${OPENGL_INCLUDE_DIRS})
#include_directories(${GLEW_INCLUDE_DIRS})
include_directories(${GLFW_INCLUDE_DIRS})
include_directories(${X11_INCLUDE_DIR})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DGLEW_STATIC")

target_link_libraries(imgui PUBLIC ${OPENGL_LIBRARIES})
target_link_libraries(imgui PUBLIC ${OPENGL_gl_LIBRARY})
target_link_libraries(imgui PUBLIC glfw3)
target_link_libraries(imgui PUBLIC ${GLFW_LIBRARIES})
target_link_libraries(imgui PUBLIC ${GL_LIBRARY})
target_link_libraries(imgui PUBLIC OpenGL::GL)
target_link_libraries(imgui PUBLIC ${CMAKE_DL_LIBS})
target_link_libraries(imgui PUBLIC GLEW::GLEW)
target_link_libraries(imgui PUBLIC ${X11_LIBRARIES})
target_link_libraries(imgui PUBLIC Threads::Threads)

target_include_directories(imgui PUBLIC ${OPENGL_INCLUDE_DIR})

target_include_directories(imgui PUBLIC ${mrcap_SOURCE_DIR}/include)
