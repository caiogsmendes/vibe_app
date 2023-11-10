cmake_minimum_required(VERSION 3.10)
project(vibracao VERSION 1.0 LANGUAGES C CXX)

# Source files
# set(SOURCE
#     Vibracao_teste.c
#     hackrf.c
#     hackrf_info.c
#     HEtechSerial.cpp
# )

# # Header files
# set(DEPS
#     hackrf.h
#     hackrf_info.h
#     HEtechSerial.h
# )



# set(LIBUSB_LIBRARY_PATH /usr/lib/aarch64-linux-gnu/libusb-1.0.so)

# list(APPEND CMAKE_PREFIX_PATH "${CMAKE_SOURCE_DIR}/dependencies")

add_library(Vibe-lib
            ${CMAKE_SOURCE_DIR}/HEtechSerial.cpp
            # ${CMAKE_SOURCE_DIR}/hackrf.c
            # ${CMAKE_SOURCE_DIR}/hackrf_info.c
)

# Object files
add_executable(${PROJECT_NAME} vibracao_v2.c)

# target_include_directories(vibracao_v2 PUBLIC /usr/include/libusb-1.0)

# target_link_libraries(Vibracao_teste PUBLIC ${LIBUSB_LIBRARY_PATH})

# Include directories
target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_include_directories(${PROJECT_NAME} PUBLIC Vibe-lib)

# Compiler flags
# set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -lusb-1.0 -pthread")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -lusb-1.0 -pthread")

# find_package(PkgConfig REQUIRED)
# pkg_check_modules(libusb REQUIRED libusb-1.0)

# Find libusb
# find_package(libusb-1.0 REQUIRED)
# target_link_libraries(Vibracao_teste PRIVATE ${LIBUSB_1_LIBRARIES})
# target_include_directories(Vibracao_teste PRIVATE ${LIBUSB_1_INCLUDE_DIRS})

# Linker flags
# set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -lusb-1.0 -pthread")

target_link_libraries(${PROJECT_NAME} PUBLIC Vibe-lib
                                                gpiod
                                                pthread
                                                # ${LIBUSB_LIBRARY_PATH}
                                                )

# Install target
install(TARGETS ${PROJECT_NAME} DESTINATION bin)

# # Clean target
# add_custom_target(clean_all
#     COMMAND ${CMAKE_BUILD_TOOL} clean
#     COMMAND rm -rf ${CMAKE_CURRENT_SOURCE_DIR}/install
# )


