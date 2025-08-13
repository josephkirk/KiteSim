# Test configuration
find_package(GTest QUIET)

if(NOT GTest_FOUND)
    # Download and build GoogleTest if not found
    include(FetchContent)
    FetchContent_Declare(
        googletest
        URL https://github.com/google/googletest/archive/03597a01ee50f33f9142fd2d6828e8e5ab0772ba.zip
    )
    FetchContent_MakeAvailable(googletest)
endif()

# Test executable
add_executable(kite_sim_tests
    test_math_types.cpp
    test_coordinate_transforms.cpp
    test_kite_state.cpp
    test_kite_geometry.cpp
    test_aero_coefficients.cpp
    test_wind_model.cpp
    test_aerodynamics.cpp
    test_tether.cpp
    test_main.cpp
)

target_link_libraries(kite_sim_tests 
    kite_simulation
    gtest
    gtest_main
)

# Add test to CTest
add_test(NAME kite_sim_unit_tests COMMAND kite_sim_tests)