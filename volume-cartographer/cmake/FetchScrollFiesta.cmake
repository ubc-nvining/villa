# ScrollFiesta (github.com/Hob3rMallow/scrollfiesta_public) — external sheet
# detangling / mesh cleanup toolkit for scroll segmentations.
#
# VC3D never LINKS ScrollFiesta: binding happens at runtime via dlopen /
# LoadLibrary of the shared library and a single resolved symbol, sf_get_api,
# which returns a function-pointer table (see core/src/fiesta/FiestaBridge.cpp
# and include/scrollfiesta.h in the ScrollFiesta repo). FetchContent's job
# here is only to provide (a) the public header at compile time and (b) a
# built scrollfiesta.dll/.so staged into ${CMAKE_BINARY_DIR}/bin next to the
# executables. Swapping in a newer library later is "replace the DLL,
# restart" — sf_get_api(SCROLLFIESTA_ABI_VERSION) is the compatibility gate.
#
# Local development against a checkout (skips the network fetch):
#   cmake -DFETCHCONTENT_SOURCE_DIR_SCROLLFIESTA=D:/work/scrollfiesta_public ...

include(FetchContent)

set(VC_SCROLLFIESTA_TAG "v0.9.0" CACHE STRING
    "scrollfiesta_public git tag to fetch")

# Library-only embed: no CLI tools, no TIFF, no tests, no install rules.
set(SCROLLFIESTA_BUILD_TOOLS OFF)
set(SCROLLFIESTA_BUILD_TESTS OFF)
set(SCROLLFIESTA_WITH_TIFF   OFF)
set(SCROLLFIESTA_INSTALL     OFF)

# The canonical artifact is the shared library regardless of the host's
# library type (its RUNTIME output inherits CMAKE_RUNTIME_OUTPUT_DIRECTORY,
# landing in build/bin automatically).
set(_vc_saved_build_shared ${BUILD_SHARED_LIBS})
set(BUILD_SHARED_LIBS ON)
FetchContent_Declare(scrollfiesta
    GIT_REPOSITORY https://github.com/Hob3rMallow/scrollfiesta_public.git
    GIT_TAG        ${VC_SCROLLFIESTA_TAG}
    GIT_SHALLOW    TRUE)
FetchContent_MakeAvailable(scrollfiesta)
set(BUILD_SHARED_LIBS ${_vc_saved_build_shared})

# The one compile-time artifact consumers use: the public header.
set(VC_SCROLLFIESTA_INCLUDE_DIR ${scrollfiesta_SOURCE_DIR}/include)
