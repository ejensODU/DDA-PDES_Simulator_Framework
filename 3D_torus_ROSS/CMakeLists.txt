cmake_minimum_required(VERSION 3.10)
project(ROSS_TORUS3D C)

# Set ROSS directories - update these paths as needed for your environment
set(ROSS_SOURCE_DIR /home/ejens/ROSS/build/core)
set(ROSS_BINARY_DIR /home/ejens/ROSS/build/core)

# Optimization flags
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O3 -g")

# Include ROSS directories
include_directories(${ROSS_SOURCE_DIR} ${ROSS_BINARY_DIR})

# Find MPI
find_package(MPI REQUIRED)
if (MPI_FOUND)
    message(STATUS "MPI found at: ${MPI_C_INCLUDE_PATH}")
    include_directories(${MPI_C_INCLUDE_PATH} /usr/lib/aarch64-linux-gnu/openmpi/include)
    link_directories(/usr/lib/aarch64-linux-gnu/openmpi/lib)
else()
    message(FATAL_ERROR "MPI not found!")
endif()

# Find ROSS library
find_library(ROSS_LIB NAMES ROSS PATHS ${ROSS_BINARY_DIR} NO_DEFAULT_PATH)
if (NOT ROSS_LIB)
    message(FATAL_ERROR "libROSS.a not found! Set ROSS_SOURCE_DIR and ROSS_BINARY_DIR correctly.")
endif()

# Define source files
set(TORUS3D_SRCS
    torus_3d_ross_model.c
    torus_3d_events.c
    torus_3d_utils.c
)

# Define executable
add_executable(torus3d ${TORUS3D_SRCS})

# Link libraries
target_link_libraries(torus3d ${ROSS_LIB} MPI::MPI_C m)

# Install target
install(TARGETS torus3d DESTINATION bin)

# Additional targets for testing
add_custom_target(run
    COMMAND mpirun -np 4 ./torus3d --synch=3 --x=4 --y=4 --z=2 --hop=1 --servers=1 --end_time=100.0
    DEPENDS torus3d
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
    COMMENT "Running Torus3D simulation with default parameters"
)

add_custom_target(test-seq
    COMMAND ./torus3d --synch=1 --x=2 --y=2 --z=2 --hop=1 --servers=1 --max_events=10 --end_time=100.0
    DEPENDS torus3d
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
    COMMENT "Running Torus3D test in sequential mode"
)

# Configuration summary output
message(STATUS "ROSS Torus3D Configuration:")
message(STATUS "  ROSS Source:  ${ROSS_SOURCE_DIR}")
message(STATUS "  ROSS Binary:  ${ROSS_BINARY_DIR}")
message(STATUS "  ROSS Library: ${ROSS_LIB}")
message(STATUS "  MPI Include:  ${MPI_C_INCLUDE_PATH}")
message(STATUS "  MPI Library:  ${MPI_C_LIBRARIES}")
message(STATUS "  Compiler:     ${CMAKE_C_COMPILER}")
message(STATUS "  Flags:        ${CMAKE_C_FLAGS}")