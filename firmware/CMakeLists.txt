# Host build of the firmware core: static library, unit tests and the
# software-in-the-loop shared library used by python/kalonda/swarm/sil.py.
# The ESP32 build lives in ports/esp32 (ESP-IDF) and compiles the same src/.
cmake_minimum_required(VERSION 3.16)
project(kalonda_swarm_firmware C)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

option(KS_SANITIZE "Build with AddressSanitizer and UBSan" OFF)
option(KS_WERROR "Treat warnings as errors" ON)

set(KS_WARNINGS -Wall -Wextra -Wpedantic -Wshadow -Wstrict-prototypes
    -Wmissing-prototypes -Wdouble-promotion -Wformat=2 -Wundef)
if(KS_WERROR)
  list(APPEND KS_WARNINGS -Werror)
endif()

if(KS_SANITIZE)
  add_compile_options(-fsanitize=address,undefined -fno-omit-frame-pointer -g -O1)
  add_link_options(-fsanitize=address,undefined)
endif()

file(GLOB KS_CORE_SRC CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/src/*.c)
add_library(ks_core STATIC ${KS_CORE_SRC})
target_include_directories(ks_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_compile_options(ks_core PRIVATE ${KS_WARNINGS})
target_link_libraries(ks_core PUBLIC m)
set_target_properties(ks_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_library(ks_sil SHARED ports/sil/ks_sil.c)
target_include_directories(ks_sil PUBLIC ports/sil)
target_compile_options(ks_sil PRIVATE ${KS_WARNINGS})
target_link_libraries(ks_sil PRIVATE ks_core)

enable_testing()
set(KS_TESTS crypto wire stats epoch geom consensus sense safety log negotiate node)
foreach(t ${KS_TESTS})
  add_executable(test_${t} test/test_${t}.c)
  target_include_directories(test_${t} PRIVATE test src)
  target_compile_options(test_${t} PRIVATE ${KS_WARNINGS} -Wno-missing-prototypes -Wno-double-promotion)
  target_link_libraries(test_${t} PRIVATE ks_core)
  add_test(NAME ${t} COMMAND test_${t})
endforeach()
