# Versión mínima de CMake requerida
cmake_minimum_required(VERSION 3.10)

# Nombre del proyecto
project(schc_gateway)

# Especificar el estándar de C++
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_CXX_FLAGS_DEBUG "-g -O0")


# Buscar las bibliotecas fmt y spdlog con versiones específicas
find_package(PkgConfig REQUIRED)
find_package(fmt 9.1.0 REQUIRED)
find_package(spdlog 1.12.0 REQUIRED)

# Buscar la biblioteca Mosquitto usando pkg-config
pkg_check_modules(MOSQUITTO REQUIRED libmosquitto)

# Mostrar información sobre Mosquitto para depuración
message(STATUS "Mosquitto folder included: ${MOSQUITTO_INCLUDE_DIRS}")
message(STATUS "Mosquitto libraries: ${MOSQUITTO_LIBRARIES}")

# Agregar archivos fuente (GLOB)
file(GLOB CPP_FILES "src/*.cpp")

# Crear el ejecutable y asociar las bibliotecas y directorios
add_executable(schc_gateway ${CPP_FILES})

# Agregar directorios de inclusión
target_include_directories(schc_gateway PRIVATE include ${MOSQUITTO_INCLUDE_DIRS})

# Enlazar las bibliotecas necesarias
target_link_libraries(schc_gateway PRIVATE 
    fmt::fmt
    spdlog::spdlog
    ${MOSQUITTO_LIBRARIES}
)

# Opcional: Mostrar los archivos fuente encontrados
message(STATUS "Source Files: ${CPP_FILES}")
