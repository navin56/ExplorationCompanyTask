cmake_minimum_required(VERSION 3.15)

project(TEC_Task)

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

set(MAIN_SRC 
    src/gnc.c)

set(LIB_SRC
    libSrc/threadLib.c
    libSrc/interfaceLib.c)

set(SUBMODULE_SRC
    submodules/npy/npy_array.c)

set(IMU_SRC
    src/sensors.c)

# All Warning bitte.
add_compile_options(-Wall -Wextra -pedantic -g -Og)

add_executable(GncMain ${LIB_SRC} ${SUBMODULE_SRC} ${MAIN_SRC})

add_executable(SensorsOut ${LIB_SRC} ${SUBMODULE_SRC} ${IMU_SRC})

target_include_directories(GncMain PRIVATE
                            ${PROJECT_SOURCE_DIR}/inc
                            ${PROJECT_SOURCE_DIR}/libInc
                            ${PROJECT_SOURCE_DIR}/submodules/npy/)

target_include_directories(SensorsOut PRIVATE
                            ${PROJECT_SOURCE_DIR}/inc
                            ${PROJECT_SOURCE_DIR}/libInc
                            ${PROJECT_SOURCE_DIR}/submodules/npy/)

target_link_libraries(GncMain PRIVATE Threads::Threads )
target_link_libraries(SensorsOut PRIVATE Threads::Threads)