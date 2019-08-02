# CMakeLists.txt has to be located in the project folder and cmake has to be
# executed from 'project/build' with 'cmake ../'.
cmake_minimum_required(VERSION 2.6)

find_package(lib_manager)

lib_defaults()
define_module_info()

find_package(Rock)
rock_init(cpp_bagel_wrapper 0.1)

include(FindPkgConfig)

pkg_check_modules(PKGCONFIG REQUIRED
        mars_utils
        c_bagel
        configmaps
)
include_directories(${PKGCONFIG_INCLUDE_DIRS})
link_directories(${PKGCONFIG_LIBRARY_DIRS})
add_definitions(${PKGCONFIG_CFLAGS_OTHER})  #flags excluding the ones with -I

add_subdirectory(${PROJECT_SOURCE_DIR}/src)

enable_testing()
add_subdirectory(${PROJECT_SOURCE_DIR}/test)
