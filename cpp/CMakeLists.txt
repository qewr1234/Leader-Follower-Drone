cmake_minimum_required(VERSION 3.16)
project(mars_core LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
# 파이썬(numpy) 오라클과 수치를 맞추려면 fast-math 금지. -O2 로 충분하다.
# -ffp-contract=off: GCC 는 gnu++17 에서 a*b+c 를 FMA 로 묶는다(x86 는 -march 없이는 안 나오지만 Jetson/aarch64 는 기본).
# 파이썬(numpy)은 FMA 를 쓰지 않으므로 묶으면 1e-16 급 차이가 쌓인다 — 차등 검증을 위해 끈다.
set(CMAKE_CXX_FLAGS_RELEASE "-O2")
add_compile_options(-ffp-contract=off)

option(MARS_BUILD_PYTHON "pybind11 모듈 mars_core 빌드" ON)
option(MARS_BUILD_TESTS "gtest 빌드 (googletest 를 FetchContent 로 받는다)" ON)

add_library(mars_core_lib STATIC src/imm_ekf.cpp src/control.cpp src/mission.cpp)
target_include_directories(mars_core_lib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
set_target_properties(mars_core_lib PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_compile_options(mars_core_lib PRIVATE -Wall -Wextra)

if(MARS_BUILD_PYTHON)
  find_package(Python3 COMPONENTS Interpreter Development.Module REQUIRED)
  # pip 로 설치한 pybind11 의 CMake 디렉터리 (pybind11 --cmakedir)
  execute_process(COMMAND ${Python3_EXECUTABLE} -c "import pybind11; print(pybind11.get_cmake_dir())"
                  OUTPUT_VARIABLE _pb11_dir OUTPUT_STRIP_TRAILING_WHITESPACE ERROR_QUIET)
  if(_pb11_dir)
    list(APPEND CMAKE_PREFIX_PATH ${_pb11_dir})
  endif()
  find_package(pybind11 CONFIG REQUIRED)
  pybind11_add_module(mars_core src/bindings.cpp)
  target_link_libraries(mars_core PRIVATE mars_core_lib)
  # 저장소 루트에 놓아 `import mars_core` 가 바로 되게
  set_target_properties(mars_core PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/..)
endif()

if(MARS_BUILD_TESTS)
  include(FetchContent)
  FetchContent_Declare(googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG v1.14.0
    GIT_SHALLOW TRUE)
  set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
  FetchContent_MakeAvailable(googletest)
  enable_testing()
  foreach(_t imm_ekf control mission)
    add_executable(test_${_t} tests/test_${_t}.cpp)
    target_link_libraries(test_${_t} PRIVATE mars_core_lib GTest::gtest_main)
    add_test(NAME ${_t} COMMAND test_${_t})
  endforeach()
endif()
