cmake_minimum_required(VERSION 3.10)
set(CMAKE_BUILD_TYPE RELEASE)

project(cligotcha)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_CXX_FLAGS "-Wall -Wextra -pthread ")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -march=native -ftree-vectorizer-verbose=2 -ffast-math")
set(CMAKE_CXX_FLAGS_DEBUG "-ggdb -O0")


find_package( OpenCV REQUIRED )
find_package( Eigen3 REQUIRED NO_MODULE)

# Gotcha
add_executable(gotcha)
target_sources(gotcha PRIVATE src/gotcha.cpp
                    src/TPHandler.cpp
                    src/Densify.cpp
                    src/ALSC.cpp
                    src/alscworker.cpp
                    src/tpqueue.cpp
                    src/visitedmap.cpp
                    src/timer.cpp)

target_include_directories(gotcha PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(gotcha ${OpenCV_LIBS})
target_link_libraries(gotcha Eigen3::Eigen)

# TiePoint Generator:
add_executable(tpgen)
target_sources(tpgen PRIVATE src/tpgen.cpp
                    src/CIPdetector.cpp
                    src/ALSC.cpp
                    src/CMutualAdapt.cpp
                    src/CIPParamAdapt.cpp
                    src/CInterestPoint.cpp
                    src/CAutoAdapt.cpp
                    src/CAlscAdapt.cpp)

target_include_directories(tpgen PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)



target_link_libraries(tpgen ${OpenCV_LIBS})
target_link_libraries(tpgen Eigen3::Eigen)
