#include "kite_sim/kite_geometry.h"
#include <gtest/gtest.h>
#include <cmath>
#include <vector>

using namespace kite_sim;

class AeroCoefficientsTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Set up common test data
        alpha_1d_ = {-10, -5, 0, 5, 10, 15, 20};
        cl_1d_ = {-0.5, 0.0, 0.5, 1.0, 1.2, 1.0, 0.8};
        cd_1d_ = {0.1, 0.08, 0.06, 0.08, 0.12, 0.18, 0.25};
        cm_1d_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        
        // 2D test data
        alpha_2d_ = {0, 10, 20};
        beta_2d_ = {-5, 0, 5};
        
        // CL table: rows = alpha, cols = beta
        cl_2d_ = {
            {0.4, 0.5, 0.6},  // alpha = 0
            {0.9, 1.0, 1.1},  // alpha = 10
            {0.7, 0.8, 0.9}   // alpha = 20
        };
        
        // 3D test data
        delta_3d_ = {-10, 0, 10};
        
        // CL table: [alpha][beta][delta]
        cl_3d_ = {
            {  // alpha = 0
                {0.3, 0.4, 0.5},  // beta = -5
                {0.4, 0.5, 0.6},  // beta = 0
                {0.5, 0.6, 0.7}   // beta = 5
            },
            {  // alpha = 10
                {0.8, 0.9, 1.0},  // beta = -5
                {0.9, 1.0, 1.1},  // beta = 0
                {1.0, 1.1, 1.2}   // beta = 5
            },
            {  // alpha = 20
                {0.6, 0.7, 0.8},  // beta = -5
                {0.7, 0.8, 0.9},  // beta = 0
                {0.8, 0.9, 1.0}   // beta = 5
            }
        };
    }
    
    // 1D test data
    std::vector<double> alpha_1d_, cl_1d_, cd_1d_, cm_1d_;
    
    // 2D test data
    std::vector<double> alpha_2d_, beta_2d_;
    std::vector<std::vector<double>> cl_2d_;
    
    // 3D test data
    std::vector<double> delta_3d_;
    std::vector<std::vector<std::vector<double>>> cl_3d_;
    
    const double tolerance_ = 1e-10;
};//
 Basic functionality tests
TEST_F(AeroCoefficientsTest, DefaultConstructor) {
    AeroCoefficients coeffs;
    
    EXPECT_EQ(coeffs.getCLTableType(), AeroCoefficients::TableType::NONE);
    EXPECT_EQ(coeffs.getCDTableType(), AeroCoefficients::TableType::NONE);
    EXPECT_EQ(coeffs.getCMTableType(), AeroCoefficients::TableType::NONE);
    EXPECT_FALSE(coeffs.hasData());
    EXPECT_TRUE(coeffs.isValid()); // Empty coefficients are considered valid
}

// 1D Table Tests
TEST_F(AeroCoefficientsTest, Set1DCoefficients) {
    AeroCoefficients coeffs;
    
    coeffs.setCL1D(alpha_1d_, cl_1d_);
    coeffs.setCD1D(alpha_1d_, cd_1d_);
    coeffs.setCM1D(alpha_1d_, cm_1d_);
    
    EXPECT_EQ(coeffs.getCLTableType(), AeroCoefficients::TableType::TABLE_1D);
    EXPECT_EQ(coeffs.getCDTableType(), AeroCoefficients::TableType::TABLE_1D);
    EXPECT_EQ(coeffs.getCMTableType(), AeroCoefficients::TableType::TABLE_1D);
    EXPECT_TRUE(coeffs.hasData());
    EXPECT_TRUE(coeffs.isValid());
}

TEST_F(AeroCoefficientsTest, Interpolation1D) {
    AeroCoefficients coeffs;
    coeffs.setCL1D(alpha_1d_, cl_1d_);
    coeffs.setCD1D(alpha_1d_, cd_1d_);
    coeffs.setCM1D(alpha_1d_, cm_1d_);
    
    // Test exact values
    EXPECT_NEAR(coeffs.getCL(0.0), 0.5, tolerance_);
    EXPECT_NEAR(coeffs.getCD(0.0), 0.06, tolerance_);
    EXPECT_NEAR(coeffs.getCM(0.0), 0.0, tolerance_);
    
    // Test interpolation
    EXPECT_NEAR(coeffs.getCL(2.5), 0.75, tolerance_); // Between 0° and 5°
    EXPECT_NEAR(coeffs.getCL(7.5), 1.1, tolerance_);  // Between 5° and 10°
    
    // Test extrapolation (should return boundary values)
    EXPECT_NEAR(coeffs.getCL(-15.0), -0.5, tolerance_); // Below range
    EXPECT_NEAR(coeffs.getCL(25.0), 0.8, tolerance_);   // Above range
}

// 2D Table Tests
TEST_F(AeroCoefficientsTest, Set2DCoefficients) {
    AeroCoefficients coeffs;
    
    coeffs.setCL2D(alpha_2d_, beta_2d_, cl_2d_);
    
    EXPECT_EQ(coeffs.getCLTableType(), AeroCoefficients::TableType::TABLE_2D);
    EXPECT_TRUE(coeffs.hasData());
    EXPECT_TRUE(coeffs.isValid());
}

TEST_F(AeroCoefficientsTest, Interpolation2D) {
    AeroCoefficients coeffs;
    coeffs.setCL2D(alpha_2d_, beta_2d_, cl_2d_);
    
    // Test exact values
    EXPECT_NEAR(coeffs.getCL(0.0, 0.0), 0.5, tolerance_);
    EXPECT_NEAR(coeffs.getCL(10.0, 0.0), 1.0, tolerance_);
    EXPECT_NEAR(coeffs.getCL(20.0, 5.0), 0.9, tolerance_);
    
    // Test bilinear interpolation
    EXPECT_NEAR(coeffs.getCL(5.0, 0.0), 0.75, tolerance_);   // Midpoint in alpha
    EXPECT_NEAR(coeffs.getCL(10.0, 2.5), 1.05, tolerance_);  // Midpoint in beta
    EXPECT_NEAR(coeffs.getCL(5.0, 2.5), 0.8, tolerance_);    // Midpoint in both
}

// 3D Table Tests
TEST_F(AeroCoefficientsTest, Set3DCoefficients) {
    AeroCoefficients coeffs;
    
    coeffs.setCL3D(alpha_2d_, beta_2d_, delta_3d_, cl_3d_);
    
    EXPECT_EQ(coeffs.getCLTableType(), AeroCoefficients::TableType::TABLE_3D);
    EXPECT_TRUE(coeffs.hasData());
    EXPECT_TRUE(coeffs.isValid());
}

TEST_F(AeroCoefficientsTest, Interpolation3D) {
    AeroCoefficients coeffs;
    coeffs.setCL3D(alpha_2d_, beta_2d_, delta_3d_, cl_3d_);
    
    // Test exact values
    EXPECT_NEAR(coeffs.getCL(0.0, 0.0, 0.0), 0.5, tolerance_);
    EXPECT_NEAR(coeffs.getCL(10.0, 0.0, 0.0), 1.0, tolerance_);
    EXPECT_NEAR(coeffs.getCL(20.0, 5.0, 10.0), 1.0, tolerance_);
    
    // Test trilinear interpolation
    EXPECT_NEAR(coeffs.getCL(5.0, 0.0, 0.0), 0.75, tolerance_);    // Midpoint in alpha
    EXPECT_NEAR(coeffs.getCL(10.0, 2.5, 0.0), 1.05, tolerance_);   // Midpoint in beta
    EXPECT_NEAR(coeffs.getCL(10.0, 0.0, 5.0), 1.05, tolerance_);   // Midpoint in delta
}// Va
lidation Tests
TEST_F(AeroCoefficientsTest, InvalidInputValidation) {
    AeroCoefficients coeffs;
    
    // Test empty vectors
    std::vector<double> empty_vec;
    coeffs.setCL1D(empty_vec, empty_vec);
    EXPECT_EQ(coeffs.getCLTableType(), AeroCoefficients::TableType::NONE);
    
    // Test mismatched sizes
    std::vector<double> short_vec = {0, 5};
    coeffs.setCL1D(alpha_1d_, short_vec);
    EXPECT_EQ(coeffs.getCLTableType(), AeroCoefficients::TableType::NONE);
    
    // Test non-monotonic alpha values
    std::vector<double> non_monotonic = {0, 10, 5, 15};
    std::vector<double> values = {0.5, 1.0, 0.8, 1.2};
    coeffs.setCL1D(non_monotonic, values);
    EXPECT_FALSE(coeffs.isValid());
}

TEST_F(AeroCoefficientsTest, BoundsChecking) {
    AeroCoefficients coeffs;
    coeffs.setCL1D(alpha_1d_, cl_1d_);
    
    auto alpha_bounds = coeffs.getAlphaBounds();
    EXPECT_EQ(alpha_bounds.first, -10.0);
    EXPECT_EQ(alpha_bounds.second, 20.0);
    
    EXPECT_TRUE(coeffs.isWithinBounds(0.0));
    EXPECT_TRUE(coeffs.isWithinBounds(10.0));
    EXPECT_FALSE(coeffs.isWithinBounds(-15.0));
    EXPECT_FALSE(coeffs.isWithinBounds(25.0));
}

TEST_F(AeroCoefficientsTest, BoundsChecking2D) {
    AeroCoefficients coeffs;
    coeffs.setCL2D(alpha_2d_, beta_2d_, cl_2d_);
    
    auto alpha_bounds = coeffs.getAlphaBounds();
    auto beta_bounds = coeffs.getBetaBounds();
    
    EXPECT_EQ(alpha_bounds.first, 0.0);
    EXPECT_EQ(alpha_bounds.second, 20.0);
    EXPECT_EQ(beta_bounds.first, -5.0);
    EXPECT_EQ(beta_bounds.second, 5.0);
    
    EXPECT_TRUE(coeffs.isWithinBounds(10.0, 0.0));
    EXPECT_FALSE(coeffs.isWithinBounds(25.0, 0.0));
    EXPECT_FALSE(coeffs.isWithinBounds(10.0, 10.0));
}

// Edge Cases and Error Handling
TEST_F(AeroCoefficientsTest, SinglePointTable) {
    AeroCoefficients coeffs;
    
    std::vector<double> single_alpha = {5.0};
    std::vector<double> single_cl = {1.0};
    
    coeffs.setCL1D(single_alpha, single_cl);
    
    EXPECT_TRUE(coeffs.isValid());
    EXPECT_NEAR(coeffs.getCL(0.0), 1.0, tolerance_);   // Should return single value
    EXPECT_NEAR(coeffs.getCL(5.0), 1.0, tolerance_);
    EXPECT_NEAR(coeffs.getCL(10.0), 1.0, tolerance_);
}

TEST_F(AeroCoefficientsTest, ExtrapolationBehavior) {
    AeroCoefficients coeffs;
    coeffs.setCL1D(alpha_1d_, cl_1d_);
    
    // Test that extrapolation returns boundary values
    double min_cl = coeffs.getCL(-100.0);  // Far below range
    double max_cl = coeffs.getCL(100.0);   // Far above range
    
    EXPECT_NEAR(min_cl, cl_1d_.front(), tolerance_);
    EXPECT_NEAR(max_cl, cl_1d_.back(), tolerance_);
}

TEST_F(AeroCoefficientsTest, MixedTableTypes) {
    AeroCoefficients coeffs;
    
    // Set different table types for different coefficients
    coeffs.setCL1D(alpha_1d_, cl_1d_);
    coeffs.setCD2D(alpha_2d_, beta_2d_, {{0.05, 0.06, 0.07}, {0.08, 0.09, 0.10}, {0.15, 0.16, 0.17}});
    
    EXPECT_EQ(coeffs.getCLTableType(), AeroCoefficients::TableType::TABLE_1D);
    EXPECT_EQ(coeffs.getCDTableType(), AeroCoefficients::TableType::TABLE_2D);
    EXPECT_EQ(coeffs.getCMTableType(), AeroCoefficients::TableType::NONE);
    
    // Test that each coefficient uses its appropriate table
    EXPECT_NEAR(coeffs.getCL(0.0), 0.5, tolerance_);      // 1D lookup
    EXPECT_NEAR(coeffs.getCD(0.0, 0.0), 0.06, tolerance_); // 2D lookup
    EXPECT_NEAR(coeffs.getCM(0.0), 0.0, tolerance_);       // No data, should return 0
}

// Backward Compatibility Tests
TEST_F(AeroCoefficientsTest, BackwardCompatibility) {
    AeroCoefficients coeffs;
    
    // Test that old interface still works
    coeffs.setCL(alpha_1d_, cl_1d_);
    coeffs.setCD(alpha_1d_, cd_1d_);
    coeffs.setCM(alpha_1d_, cm_1d_);
    
    EXPECT_EQ(coeffs.getCLTableType(), AeroCoefficients::TableType::TABLE_1D);
    EXPECT_TRUE(coeffs.hasData());
    EXPECT_TRUE(coeffs.isValid());
    
    // Test interpolation works the same way
    EXPECT_NEAR(coeffs.getCL(2.5), 0.75, tolerance_);
}// 
Performance and Accuracy Tests
TEST_F(AeroCoefficientsTest, InterpolationAccuracy) {
    AeroCoefficients coeffs;
    
    // Create a smooth function for testing: CL = 0.1 * alpha
    std::vector<double> alpha_smooth = {-20, -10, 0, 10, 20, 30};
    std::vector<double> cl_smooth;
    for (double alpha : alpha_smooth) {
        cl_smooth.push_back(0.1 * alpha);
    }
    
    coeffs.setCL1D(alpha_smooth, cl_smooth);
    
    // Test interpolation accuracy at various points
    for (double alpha = -15; alpha <= 25; alpha += 2.5) {
        double expected = 0.1 * alpha;
        double actual = coeffs.getCL(alpha);
        EXPECT_NEAR(actual, expected, 1e-6) << "Failed at alpha = " << alpha;
    }
}

TEST_F(AeroCoefficientsTest, BilinearInterpolationAccuracy) {
    AeroCoefficients coeffs;
    
    // Create a bilinear function: CL = 0.1 * alpha + 0.01 * beta
    std::vector<double> alpha_test = {0, 10, 20};
    std::vector<double> beta_test = {-10, 0, 10};
    std::vector<std::vector<double>> cl_bilinear(3, std::vector<double>(3));
    
    for (size_t i = 0; i < alpha_test.size(); ++i) {
        for (size_t j = 0; j < beta_test.size(); ++j) {
            cl_bilinear[i][j] = 0.1 * alpha_test[i] + 0.01 * beta_test[j];
        }
    }
    
    coeffs.setCL2D(alpha_test, beta_test, cl_bilinear);
    
    // Test interpolation at intermediate points
    double alpha = 5.0, beta = -5.0;
    double expected = 0.1 * alpha + 0.01 * beta;
    double actual = coeffs.getCL(alpha, beta);
    EXPECT_NEAR(actual, expected, 1e-10);
}

// Serialization Tests
TEST_F(AeroCoefficientsTest, JsonSerialization) {
    AeroCoefficients coeffs;
    coeffs.setCL1D(alpha_1d_, cl_1d_);
    coeffs.setCD1D(alpha_1d_, cd_1d_);
    coeffs.setCM1D(alpha_1d_, cm_1d_);
    
    std::string json = coeffs.toJson();
    
    // Check that JSON contains expected fields
    EXPECT_NE(json.find("cl_table_type"), std::string::npos);
    EXPECT_NE(json.find("cd_table_type"), std::string::npos);
    EXPECT_NE(json.find("cm_table_type"), std::string::npos);
    EXPECT_NE(json.find("alpha_points"), std::string::npos);
    EXPECT_NE(json.find("cl_1d"), std::string::npos);
    EXPECT_NE(json.find("cd_1d"), std::string::npos);
    EXPECT_NE(json.find("cm_1d"), std::string::npos);
    
    EXPECT_FALSE(json.empty());
}

// Stress Tests
TEST_F(AeroCoefficientsTest, LargeTableHandling) {
    AeroCoefficients coeffs;
    
    // Create a large 1D table
    std::vector<double> large_alpha;
    std::vector<double> large_cl;
    
    for (int i = -180; i <= 180; i += 1) {
        large_alpha.push_back(static_cast<double>(i));
        large_cl.push_back(std::sin(i * M_PI / 180.0)); // Sinusoidal CL curve
    }
    
    coeffs.setCL1D(large_alpha, large_cl);
    
    EXPECT_TRUE(coeffs.isValid());
    EXPECT_TRUE(coeffs.hasData());
    
    // Test interpolation still works
    double test_alpha = 45.5;
    double cl_value = coeffs.getCL(test_alpha);
    EXPECT_TRUE(std::isfinite(cl_value));
    EXPECT_GT(cl_value, 0.0); // Should be positive for 45.5 degrees
}

TEST_F(AeroCoefficientsTest, EdgeCaseValues) {
    AeroCoefficients coeffs;
    
    // Test with extreme values
    std::vector<double> extreme_alpha = {-1000, 0, 1000};
    std::vector<double> extreme_cl = {-10, 0, 10};
    
    coeffs.setCL1D(extreme_alpha, extreme_cl);
    
    EXPECT_TRUE(coeffs.isValid());
    
    // Test interpolation with extreme inputs
    EXPECT_NEAR(coeffs.getCL(0.0), 0.0, tolerance_);
    EXPECT_NEAR(coeffs.getCL(-500.0), -5.0, tolerance_); // Should interpolate
    EXPECT_NEAR(coeffs.getCL(500.0), 5.0, tolerance_);   // Should interpolate
}