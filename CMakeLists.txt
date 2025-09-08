cmake_minimum_required(VERSION 3.16)
project(DBCBS LANGUAGES CXX)

# ---------------- options ----------------
option(DBCBS_BUILD_DRIVER       "Build db_cbs CLI (BIN mode)" ON)
option(DBCBS_BUILD_EXAMPLES     "Build extra example apps (kcbs/ompl demos)" OFF)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
add_compile_options(-Wall -Wextra)

# ---------------- deps -------------------
find_package(Boost 1.58 REQUIRED COMPONENTS program_options)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(fcl REQUIRED)
find_package(ompl REQUIRED)   # Some installs export a target; others only variables

# msgpack (header-only on many systems). Try to find include path gracefully.
find_path(MSGPACK_INCLUDE_DIR msgpack.hpp
        PATH_SUFFIXES msgpack include
        DOC "Path to msgpack.hpp (msgpack-c).")
if (MSGPACK_INCLUDE_DIR)
    message(STATUS "Found msgpack headers at: ${MSGPACK_INCLUDE_DIR}")
else()
    message(STATUS "msgpack.hpp not found via CMake; assuming vendored/available in include paths.")
endif()

if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    list(APPEND CMAKE_PREFIX_PATH "/opt/openrobots")
endif()

# Normalize imported targets / vars
set(YAML_TGT yaml-cpp)

# FCL target can be fcl::fcl or FCL::fcl
set(FCL_TGT "")
if (TARGET fcl::fcl)
    set(FCL_TGT fcl::fcl)
elseif (TARGET FCL::fcl)
    set(FCL_TGT FCL::fcl)
endif()

# --- libccd (needed by FCL) ---------------------------------
find_package(PkgConfig QUIET)
if (PkgConfig_FOUND)
    pkg_check_modules(CCD QUIET ccd)
endif()

# Fallback if no pkg-config:
if (NOT CCD_FOUND)
    find_library(CCD_LIBRARY NAMES ccd libccd)
    if (CCD_LIBRARY)
        set(CCD_LIBRARIES ${CCD_LIBRARY})
    endif()
endif()

if (NOT CCD_LIBRARIES)
    message(FATAL_ERROR "libccd not found. Install libccd-dev (Ubuntu/Debian) or provide CCD_LIBRARIES.")
endif()

# OMPL may be a target or just variables
set(OMPL_TGT "")
if (TARGET ompl)
    set(OMPL_TGT ompl)
endif()

# ---------------- includes (public) ------
set(DBCBS_INC_PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/src
        ${CMAKE_CURRENT_SOURCE_DIR}/lib
        ${CMAKE_CURRENT_SOURCE_DIR}/deps/libMultiRobotPlanning/include
)

if (MSGPACK_INCLUDE_DIR)
    list(APPEND DBCBS_INC_PUBLIC ${MSGPACK_INCLUDE_DIR})
endif()

# ---------------- dynoplan / optimizer ----
add_subdirectory(dynoplan)  # defines idbastar::optimization and its headers

# ---------------- core lib (SOURCE mode) -
add_library(dbcbs_core STATIC
        src/robots.cpp
        lib/dbcbs_core.cpp
)
add_library(dbcbs::core ALIAS dbcbs_core)

target_compile_features(dbcbs_core PUBLIC cxx_std_17)
target_include_directories(dbcbs_core
        PUBLIC
        ${DBCBS_INC_PUBLIC}
        # Ensure headers available when compiling the core (needed by dbcbs_core.cpp)
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/dynoplan/include>
)

# Paths for runtime (your previous defines)
target_compile_definitions(dbcbs_core PRIVATE
        DBCBS_DYNOBENCH_BASE="${CMAKE_CURRENT_SOURCE_DIR}/dynoplan/dynobench/"
        DBCBS_MOTIONS_BASE="${CMAKE_CURRENT_SOURCE_DIR}/motions"
)

# Link minimal essentials used by robots.cpp / dbcbs_core.cpp
target_link_libraries(dbcbs_core
        PUBLIC
        Eigen3::Eigen
        ${YAML_TGT}
        ${CCD_LIBRARIES}
)

# FCL (prefer imported target; also link raw libs if present)
if (NOT "${FCL_TGT}" STREQUAL "")
    target_link_libraries(dbcbs_core PUBLIC ${FCL_TGT})
endif()
if (FCL_LIBRARIES)
    target_link_libraries(dbcbs_core PUBLIC ${FCL_LIBRARIES})
endif()

# OMPL (target or legacy variables)
if (NOT "${OMPL_TGT}" STREQUAL "")
    target_link_libraries(dbcbs_core PUBLIC ${OMPL_TGT})
else()
    if (OMPL_INCLUDE_DIRS)
        target_include_directories(dbcbs_core PUBLIC ${OMPL_INCLUDE_DIRS})
    endif()
    if (OMPL_LIBRARIES)
        target_link_libraries(dbcbs_core PUBLIC ${OMPL_LIBRARIES})
    endif()
endif()

# DynoBench optimizer for opt.yaml
target_link_libraries(dbcbs_core PRIVATE idbastar::optimization)

# ---------------- driver (BIN mode) ------
if (DBCBS_BUILD_DRIVER)
    add_executable(dbcbs src/db_cbs.cpp)
    target_compile_features(dbcbs PRIVATE cxx_std_17)

    target_link_libraries(dbcbs PRIVATE
            dbcbs::core
            Boost::program_options
            ${YAML_TGT}
    )

    # OMPL add-ons if needed (core already links, this is harmless)
    if (NOT "${OMPL_TGT}" STREQUAL "")
        target_link_libraries(dbcbs PRIVATE ${OMPL_TGT})
    else()
        if (OMPL_INCLUDE_DIRS)
            target_include_directories(dbcbs PRIVATE ${OMPL_INCLUDE_DIRS})
        endif()
        if (OMPL_LIBRARIES)
            target_link_libraries(dbcbs PRIVATE ${OMPL_LIBRARIES})
        endif()
    endif()

    # FCL add-ons if needed (core already links, this is harmless)
    if (NOT "${FCL_TGT}" STREQUAL "")
        target_link_libraries(dbcbs PRIVATE ${FCL_TGT})
    endif()

    # Paths for runtime (your previous defines)
    target_compile_definitions(dbcbs PRIVATE
            DBCBS_DYNOBENCH_BASE="${CMAKE_CURRENT_SOURCE_DIR}/dynoplan/dynobench/"
            DBCBS_MOTIONS_BASE="${CMAKE_CURRENT_SOURCE_DIR}/motions"
    )


    # core already links optimizer; linking again is fine and explicit
    target_link_libraries(dbcbs PRIVATE idbastar::optimization)
    target_include_directories(dbcbs PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/dynoplan/include)

endif()

# ---------------- examples (optional) ----
if (DBCBS_BUILD_EXAMPLES)
    add_executable(main_ompl src/main_ompl.cpp)
    target_link_libraries(main_ompl PRIVATE dbcbs::core)

    add_executable(main_kcbs src/main_kcbs.cpp)
    target_link_libraries(main_kcbs PRIVATE dbcbs::core)

    add_executable(main_ompl_geometric src/main_ompl_geometric.cpp)
    target_link_libraries(main_ompl_geometric PRIVATE dbcbs::core)
endif()
