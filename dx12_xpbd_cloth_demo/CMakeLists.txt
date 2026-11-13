cmake_minimum_required(VERSION 3.20)

project(dx12_xpbd_cloth_demo LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_executable(dx12_xpbd_cloth_demo
    src/main.cpp
    src/ClothXPBD.cpp
)

target_include_directories(dx12_xpbd_cloth_demo PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_link_libraries(dx12_xpbd_cloth_demo PRIVATE
    d3d12
    dxgi
    d3dcompiler
    dxguid
    user32
    gdi32
)

if(MSVC)
    target_compile_options(dx12_xpbd_cloth_demo PRIVATE /W4 /permissive-)
endif()
