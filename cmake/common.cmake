# Build settings shared by localization_core and the projects that use it
# (gnss_imu_prediction, drone_localization). Include it right after
# project():
#
#   include(external/localization_core/cmake/common.cmake)
include_guard(GLOBAL)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Debug CACHE STRING "Build type" FORCE)
endif()

if(NOT DEFINED CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_compile_options(
  -Wall -Wextra -Wpedantic -Wunused -Wuninitialized -Wshadow -Wconversion -Wduplicated-cond -Wduplicated-branches
  -Wlogical-op -Wuseless-cast -Wsign-conversion -Wfloat-equal -Weffc++ -Wold-style-cast -Wvexing-parse -Wformat=2 -Wvla 
  -Wsuggest-override -Wsuggest-final-methods -Wsuggest-final-types -Wnon-virtual-dtor -Wctor-dtor-privacy
)

if(CMAKE_COMPILER_IS_GNUCXX)
  if(CMAKE_CXX_COMPILER_VERSION VERSION_EQUAL 14.1.1)
    add_compile_options(-Wnrvo)
  endif()
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(
    -Wweak-vtables -Wglobal-constructors -Wexit-time-destructors -Wused-but-marked-unused -Wcovered-switch-default
    -Wmissing-prototypes -Woverloaded-virtual -Wdelete-non-virtual-dtor
  )
endif()
