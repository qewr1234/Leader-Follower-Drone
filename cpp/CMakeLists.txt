cmake_minimum_required(VERSION 3.16)
project(mars_core LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
# 파이썬(numpy) 오라클과 수치를 맞추려면 fast-math 금지. -O2 로 충분하다.
set(CMAKE_CXX_FLAGS_RELEASE "-O2")

option(MARS_BUILD_PYTHON "pybind11 모듈 mars_core 빌드" ON)
option(MARS_BUILD_TESTS "gtest 빌드 (googletest 를 FetchContent 로 받는다)" ON)

add_library(mars_core_lib STATIC src/imm_ekf.cpp)
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
  add_executable(test_imm_ekf tests/test_imm_ekf.cpp)
  target_link_libraries(test_imm_ekf PRIVATE mars_core_lib GTest::gtest_main)
  add_test(NAME imm_ekf COMMAND test_imm_ekf)
endif()
