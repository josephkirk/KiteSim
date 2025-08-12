#include "kite_sim/kite_geometry.h"
#include <gtest/gtest.h>
#include <cmath>
#include <cstdio>

using namespace kite_sim;

class KiteGeometryTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Set up test data
        wing_area_ = 12.0;
        aspect_ratio_ = 6.0;
        mass_ = 2.5;
        cg_ = Vector3(0.1, 0.0, 0.05);
        inertia_ = Matrix3x3::identity();
        inertia_ *= 0.5; // Scale identity matrix
    }

    double wing_area_;
    double aspect_ratio_;
    double mass_;
    Vector3 cg_;
    Matrix3x3 inertia_;
};

// MassProperties Tests
TEST_F(KiteGeometryTest, MassPropertiesDefaultConstructor) {
    MassProperties props;
    
    EXPECT_EQ(props.mass, 1.0);
    EXPECT_TRUE(props.center_of_gravity.isZero());
    EXPECT_TRUE(props.isValid());
}

TEST_F(KiteGeometryTest, MassPropertiesParameterizedConstructor) {
    MassProperties props(mass_, cg_, inertia_);
    
    EXPECT_EQ(props.mass, mass_);
    EXPECT_EQ(props.center_of_gravity.x(), cg_.x());
    EXPECT_EQ(props.center_of_gravity.y(), cg_.y());
    EXPECT_EQ(props.center_of_gravity.z(), cg_.z());
    EXPECT_TRUE(props.isValid());
}

TEST_F(KiteGeometryTest, MassPropertiesValidation) {
    // Valid properties
    MassProperties valid_props(mass_, cg_, inertia_);
    EXPECT_TRUE(valid_props.isValid());
    
    // Invalid mass (negative)
    MassProperties invalid_mass(-1.0, cg_, inertia_);
    EXPECT_FALSE(invalid_mass.isValid());
    
    // Invalid mass (zero)
    MassProperties zero_mass(0.0, cg_, inertia_);
    EXPECT_FALSE(zero_mass.isValid());
}

TEST_F(KiteGeometryTest, MassPropertiesJsonSerialization) {
    MassProperties props(mass_, cg_, inertia_);
    
    std::string json = props.toJson();
    EXPECT_FALSE(json.empty());
    EXPECT_NE(json.find("mass"), std::string::npos);
    EXPECT_NE(json.find("center_of_gravity"), std::string::npos);
    EXPECT_NE(json.find("inertia_tensor"), std::string::npos);
    
    // Basic deserialization test
    MassProperties deserialized;
    bool success = deserialized.fromJson(json);
    EXPECT_TRUE(success);
}

// AeroCoefficients Tests
TEST_F(KiteGeometryTest, AeroCoefficientsDefaultConstructor) {
    AeroCoefficients coeffs;
    
    EXPECT_FALSE(coeffs.hasData());
    EXPECT_FALSE(coeffs.isValid());
}

TEST_F(KiteGeometryTest, AeroCoefficientsSetAndGet) {
    AeroCoefficients coeffs;
    
    std::vector<double> alpha = {-10, -5, 0, 5, 10, 15};
    std::vector<double> cl = {-0.5, 0.0, 0.5, 1.0, 1.2, 1.0};
    std::vector<double> cd = {0.1, 0.08, 0.06, 0.08, 0.12, 0.18};
    std::vector<double> cm = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    
    coeffs.setCL(alpha, cl);
    coeffs.setCD(alpha, cd);
    coeffs.setCM(alpha, cm);
    
    EXPECT_TRUE(coeffs.hasData());
    EXPECT_TRUE(coeffs.isValid());
    
    // Test interpolation
    EXPECT_NEAR(coeffs.getCL(0.0), 0.5, 1e-10);
    EXPECT_NEAR(coeffs.getCL(2.5), 0.75, 1e-10); // Interpolated value
    EXPECT_NEAR(coeffs.getCD(0.0), 0.06, 1e-10);
    EXPECT_NEAR(coeffs.getCM(5.0), 0.0, 1e-10);
}

TEST_F(KiteGeometryTest, AeroCoefficientsInterpolation) {
    AeroCoefficients coeffs;
    
    std::vector<double> alpha = {0, 10, 20};
    std::vector<double> cl = {0.0, 1.0, 0.8};
    
    coeffs.setCL(alpha, cl);
    coeffs.setCD(alpha, {0.05, 0.1, 0.2});
    coeffs.setCM(alpha, {0.0, 0.0, 0.0});
    
    // Test interpolation at midpoint
    EXPECT_NEAR(coeffs.getCL(5.0), 0.5, 1e-10);
    EXPECT_NEAR(coeffs.getCL(15.0), 0.9, 1e-10);
    
    // Test extrapolation
    EXPECT_NEAR(coeffs.getCL(-5.0), 0.0, 1e-10); // Below range
    EXPECT_NEAR(coeffs.getCL(25.0), 0.8, 1e-10); // Above range
}

TEST_F(KiteGeometryTest, AeroCoefficientsJsonSerialization) {
    AeroCoefficients coeffs;
    
    std::vector<double> alpha = {0, 10};
    std::vector<double> cl = {0.0, 1.0};
    coeffs.setCL(alpha, cl);
    coeffs.setCD(alpha, {0.05, 0.1});
    coeffs.setCM(alpha, {0.0, 0.0});
    
    std::string json = coeffs.toJson();
    EXPECT_FALSE(json.empty());
    EXPECT_NE(json.find("alpha_points"), std::string::npos);
    EXPECT_NE(json.find("cl_values"), std::string::npos);
    EXPECT_NE(json.find("cd_values"), std::string::npos);
    EXPECT_NE(json.find("cm_values"), std::string::npos);
}

// KiteGeometry Tests
TEST_F(KiteGeometryTest, DefaultConstructor) {
    KiteGeometry kite;
    
    EXPECT_GT(kite.wingArea(), 0.0);
    EXPECT_GT(kite.aspectRatio(), 0.0);
    EXPECT_GT(kite.wingspan(), 0.0);
    EXPECT_GT(kite.meanChord(), 0.0);
    EXPECT_TRUE(kite.massProperties().isValid());
}

TEST_F(KiteGeometryTest, SettersAndGetters) {
    KiteGeometry kite;
    
    kite.setWingArea(wing_area_);
    kite.setAspectRatio(aspect_ratio_);
    
    EXPECT_EQ(kite.wingArea(), wing_area_);
    EXPECT_EQ(kite.aspectRatio(), aspect_ratio_);
    
    // Check that derived properties are updated
    double expected_wingspan = std::sqrt(wing_area_ * aspect_ratio_);
    double expected_chord = wing_area_ / expected_wingspan;
    
    EXPECT_NEAR(kite.wingspan(), expected_wingspan, 1e-10);
    EXPECT_NEAR(kite.meanChord(), expected_chord, 1e-10);
}

TEST_F(KiteGeometryTest, MassPropertiesAccess) {
    KiteGeometry kite;
    
    MassProperties props(mass_, cg_, inertia_);
    kite.setMassProperties(props);
    
    const MassProperties& retrieved = kite.massProperties();
    EXPECT_EQ(retrieved.mass, mass_);
    EXPECT_EQ(retrieved.center_of_gravity.x(), cg_.x());
}

TEST_F(KiteGeometryTest, AeroCoefficientsAccess) {
    KiteGeometry kite;
    
    AeroCoefficients coeffs;
    std::vector<double> alpha = {0, 10};
    std::vector<double> cl = {0.0, 1.0};
    coeffs.setCL(alpha, cl);
    coeffs.setCD(alpha, {0.05, 0.1});
    coeffs.setCM(alpha, {0.0, 0.0});
    
    kite.setAeroCoefficients(coeffs);
    
    const AeroCoefficients& retrieved = kite.aeroCoefficients();
    EXPECT_TRUE(retrieved.hasData());
    EXPECT_NEAR(retrieved.getCL(5.0), 0.5, 1e-10);
}

TEST_F(KiteGeometryTest, ReferencePoints) {
    KiteGeometry kite;
    
    Vector3 aero_center(0.25, 0.0, 0.0);
    Vector3 tether_point(0.0, 0.0, -0.1);
    
    kite.setAerodynamicCenter(aero_center);
    kite.setTetherAttachmentPoint(tether_point);
    
    EXPECT_EQ(kite.aerodynamicCenter().x(), aero_center.x());
    EXPECT_EQ(kite.tetherAttachmentPoint().z(), tether_point.z());
}

TEST_F(KiteGeometryTest, Validation) {
    KiteGeometry valid_kite = KiteGeometry::createDefaultKite();
    EXPECT_TRUE(valid_kite.isValid());
    
    std::vector<std::string> errors = valid_kite.getValidationErrors();
    EXPECT_TRUE(errors.empty());
    
    // Test invalid geometry
    KiteGeometry invalid_kite;
    invalid_kite.setWingArea(-1.0); // Invalid negative area
    EXPECT_FALSE(invalid_kite.isValid());
    
    errors = invalid_kite.getValidationErrors();
    EXPECT_FALSE(errors.empty());
}

TEST_F(KiteGeometryTest, JsonSerialization) {
    KiteGeometry kite = KiteGeometry::createDefaultKite();
    
    std::string json = kite.toJson();
    EXPECT_FALSE(json.empty());
    EXPECT_NE(json.find("wing_area"), std::string::npos);
    EXPECT_NE(json.find("aspect_ratio"), std::string::npos);
    EXPECT_NE(json.find("mass_properties"), std::string::npos);
    EXPECT_NE(json.find("aero_coefficients"), std::string::npos);
    
    // Basic deserialization test
    KiteGeometry deserialized;
    bool success = deserialized.fromJson(json);
    EXPECT_TRUE(success);
}

TEST_F(KiteGeometryTest, EqualityOperators) {
    KiteGeometry kite1 = KiteGeometry::createDefaultKite();
    KiteGeometry kite2 = KiteGeometry::createDefaultKite();
    KiteGeometry kite3 = KiteGeometry::createTestKite();
    
    EXPECT_EQ(kite1, kite2);
    EXPECT_NE(kite1, kite3);
    EXPECT_FALSE(kite1 != kite2);
    EXPECT_TRUE(kite1 != kite3);
}

TEST_F(KiteGeometryTest, DefaultConfigurations) {
    KiteGeometry default_kite = KiteGeometry::createDefaultKite();
    EXPECT_TRUE(default_kite.isValid());
    EXPECT_GT(default_kite.wingArea(), 0.0);
    EXPECT_TRUE(default_kite.aeroCoefficients().hasData());
    
    KiteGeometry test_kite = KiteGeometry::createTestKite();
    EXPECT_TRUE(test_kite.isValid());
    EXPECT_GT(test_kite.wingArea(), 0.0);
}

TEST_F(KiteGeometryTest, FileOperations) {
    KiteGeometry kite = KiteGeometry::createDefaultKite();
    
    // Test save to file
    std::string filename = "test_kite_config.json";
    bool save_success = kite.saveToFile(filename);
    EXPECT_TRUE(save_success);
    
    // Test load from file
    KiteGeometry loaded_kite;
    bool load_success = loaded_kite.loadFromFile(filename);
    EXPECT_TRUE(load_success);
    
    // Clean up test file
    std::remove(filename.c_str());
}