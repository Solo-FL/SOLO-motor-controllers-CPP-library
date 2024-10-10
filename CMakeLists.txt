cmake_minimum_required(VERSION 3.10)

# Set the project name and version
project(SOLOmotorcontrollersCPPlibrary VERSION 1.0)

# Specify C++ standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Define configurations (Release and Debug)
set(CMAKE_CONFIGURATION_TYPES "Debug;Release" CACHE STRING "" FORCE)

# Source files
set(LIB_SOURCE_FILES
    src/SOLOMotorControllers.cpp
    src/SOLOMotorControllersSerial.cpp
    src/SOLOMotorControllersUtils.cpp
)

# Header files
set(LIB_HEADER_FILES
    src/SOLOMotorControllers.h
    src/SOLOMotorControllersSerial.h
    src/SOLOMotorControllersUtils.h
)

# Create a static or shared library
add_library(SOLOMotorControllers SHARED ${LIB_SOURCE_FILES} ${LIB_HEADER_FILES})
target_include_directories(SOLOMotorControllers PRIVATE
    ${CMAKE_SOURCE_DIR}/inc
    ${CMAKE_SOURCE_DIR}/src
)

# and a test executable
add_executable(PMSM_SpeedControl_Encoder examples/serial/PMSM_SpeedControl_Encoder.cpp)
target_include_directories(PMSM_SpeedControl_Encoder PRIVATE ${CMAKE_SOURCE_DIR}/inc ${CMAKE_SOURCE_DIR}/src)

# link the library to the executable
target_link_libraries(PMSM_SpeedControl_Encoder PRIVATE SOLOMotorControllers)



# Set the character set (Unicode for Win32 and MultiByte for x64)
if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    target_compile_definitions(SOLOMotorControllers PRIVATE _LINUX)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -W -O2 -fpermissive")
else()
    # Windows setup, assuming Microsoft compiler
    # - Set the platform architecture
    if(CMAKE_SIZEOF_VOID_P EQUAL 8)
        set(PLATFORM_ARCH "x64")
        target_compile_definitions(SOLOMotorControllers PRIVATE _MBCS)
    else()
        set(PLATFORM_ARCH "Win32")
        target_compile_definitions(SOLOMotorControllers PRIVATE UNICODE)
    endif()
    # - Set flags for different configurations
    if(CMAKE_BUILD_TYPE STREQUAL "Debug")
        target_compile_definitions(SOLOMotorControllers PRIVATE _DEBUG _CONSOLE)
        set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /W3 /MTd")
    else()
        target_compile_definitions(SOLOMotorControllers PRIVATE NDEBUG _CONSOLE)
        set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /W3 /O2 /MT")
    endif()

    # - Define additional compiler options
    target_compile_options(SOLOMotorControllers PRIVATE
        /SDL
        /permissive-    # Conformance Mode
    )
endif()

# Installation instructions
install(TARGETS SOLOMotorControllers
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
)

# Optionally, install header files
install(FILES ${LIB_HEADER_FILES} DESTINATION include)
