cmake_minimum_required(VERSION 3.16)
project(txn-graph-engine LANGUAGES CXX)

# ---- C++17 standard -------------------------------------------
set(CMAKE_CXX_STANDARD          17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS        OFF)

# ---- Compile flags ---------------------------------------------
# -Wall -Wextra -O2 as specified; add -Wpedantic for stricter
# conformance checks.  Thread support (-pthread) is needed for
# the ThreadPool and ParallelExecutor.
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic -O2")

# ---- Executable ------------------------------------------------
add_executable(txn-graph-engine
    src/main.cpp
)

target_include_directories(txn-graph-engine
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Link pthreads — required by std::thread on Linux and macOS
find_package(Threads REQUIRED)
target_link_libraries(txn-graph-engine PRIVATE Threads::Threads)

# ---- Install (optional) ----------------------------------------
install(TARGETS txn-graph-engine RUNTIME DESTINATION bin)
