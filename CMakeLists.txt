# Requires CMake version greater than or equal to the following
cmake_minimum_required(VERSION 3.20)

# We indicate the name of the project and the language used
project(debug-drawer-web-win-box2d-v3-sdl3-c LANGUAGES C)

# We write the name of the application. The file app.exe will be created
add_executable(app)

# Check for WebAssembly
if (CMAKE_SYSTEM_NAME MATCHES "Emscripten")
    target_include_directories(app PRIVATE E:/libs/box2d-3.1.0-prefix/web/include)
    target_link_directories(app PRIVATE E:/libs/box2d-3.1.0-prefix/web/lib)
endif()

# Check for Windows
if (WIN32)
    target_include_directories(app PRIVATE E:/libs/box2d-3.1.0-prefix/win/include)
    target_link_directories(app PRIVATE E:/libs/box2d-3.1.0-prefix/win/lib)
    # The build will be static
    target_link_options(app PRIVATE -static)
endif()

# Specify where the application source files are stored
target_sources(app
PRIVATE
    src/main.c
)

# We search for the library at the specified path in -DSDL_DIR
find_package(SDL3)

# Linking libraries
target_link_libraries(app PRIVATE box2d SDL3::SDL3)
