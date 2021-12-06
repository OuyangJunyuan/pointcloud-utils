find_package(GTest REQUIRED)
message(=============================================================)
message("-- option COMPILE_TEST switch on")
message("-- gtest header: ${GTEST_INCLUDE_DIR}")
message("-- gtest libraries: ${GTEST_BOTH_LIBRARIES}")
message(=============================================================)

include_directories(${GTEST_INCLUDE_DIR})

add_executable(test_pc test_pc.cpp )

target_link_libraries(test_pc
        PRIVATE
        pthread
        ${GTEST_BOTH_LIBRARIES}
        pc_utils
        )

target_compile_definitions(
        test_pc
        PRIVATE
        ROOT_PATH="${CMAKE_CURRENT_SOURCE_DIR}"
)

