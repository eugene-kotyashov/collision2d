cmake_minimum_required(VERSION 3.29)
project (Collision2D)

# GoogleTest requires at least C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/03597a01ee50ed33e9dfd640b249b4be3799d395.zip
)
# For Windows: Prevent overriding the parent project's compiler/linker settings
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

find_package(SDL2 REQUIRED)
add_executable(
    ${PROJECT_NAME} 
    src/animation.c
    src/geom_utils.c
)

target_link_libraries(${PROJECT_NAME} SDL2::SDL2 SDL2::SDL2main)

enable_testing()

add_executable(
  run_tests
  src/geom_utils.c
  src/tests.cc
)
target_link_libraries(
  run_tests
  GTest::gtest_main
)

include(GoogleTest)
gtest_discover_tests(run_tests)