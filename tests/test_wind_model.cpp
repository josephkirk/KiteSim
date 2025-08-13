#include "kite_sim/wind_model.h"
#include <gtest/gtest.h>
#include <cmath>
#include <memory>

using namespace kite_sim;

class WindVectorTest : public ::testing::Test {
protected:
    void SetUp() override {}
    void TearDown() override {}
    
    // Helper function to compare doubles with tolerance
    bool isClose(double a, double b, double tolerance = 1e-6) {
        return std::abs(a - b) < tolerance;
    }
    
    // Helper function to compare vectors with tolerance
    bool isClose(const Vector3& a, const Vector3& b, double tolerance = 1e-6) {
        return isClose(a.x(), b.x(), tolerance) && 
               isClose(a.y(), b.y(), tolerance) && 
               isClose(a.z(), b.z(), tolerance);
    }
};

TEST_F(WindVectorTest, DefaultConstructor) {
    WindVector wind;
    EXPECT_TRUE(isClose(wind.velocity(), Vector3(0, 0, 0)));
    EXPECT_DOUBLE_EQ(wind.speed(), 0.0);
    EXPECT_TRUE(wind.isCalm());
}

TEST_F(WindVectorTest, VelocityConstructor) {
    Vector3 velocity(5.0, 3.0, 1.0);
    WindVector wind(velocity);
    
    EXPECT_TRUE(isClose(wind.velocity(), velocity));
    EXPECT_DOUBLE_EQ(wind.u(), 5.0);
    EXPECT_DOUBLE_EQ(wind.v(), 3.0);
    EXPECT_DOUBLE_EQ(wind.w(), 1.0);
}

TEST_F(WindVectorTest, ComponentConstructor) {
    WindVector wind(10.0, 5.0, 2.0);
    
    EXPECT_DOUBLE_EQ(wind.u(), 10.0);
    EXPECT_DOUBLE_EQ(wind.v(), 5.0);
    EXPECT_DOUBLE_EQ(wind.w(), 2.0);
    EXPECT_TRUE(isClose(wind.speed(), std::sqrt(10*10 + 5*5 + 2*2)));
}

TEST_F(WindVectorTest, SpeedAndDirectionCalculation) {
    // Test north wind (coming from north, blowing south)
    WindVector north_wind(0.0, 10.0, 0.0);
    EXPECT_DOUBLE_EQ(north_wind.speed(), 10.0);
    EXPECT_TRUE(isClose(north_wind.direction(), 0.0));  // 0 radians = North
    
    // Test east wind (coming from east, blowing west)
    WindVector east_wind(10.0, 0.0, 0.0);
    EXPECT_DOUBLE_EQ(east_wind.speed(), 10.0);
    EXPECT_TRUE(isClose(east_wind.direction(), M_PI/2));  // π/2 radians = East
    
    // Test south wind (coming from south, blowing north)
    WindVector south_wind(0.0, -10.0, 0.0);
    EXPECT_DOUBLE_EQ(south_wind.speed(), 10.0);
    EXPECT_TRUE(isClose(south_wind.direction(), M_PI));  // π radians = South
    
    // Test west wind (coming from west, blowing east)
    WindVector west_wind(-10.0, 0.0, 0.0);
    EXPECT_DOUBLE_EQ(west_wind.speed(), 10.0);
    EXPECT_TRUE(isClose(west_wind.direction(), 3*M_PI/2));  // 3π/2 radians = West
}

TEST_F(WindVectorTest, SetSpeedAndDirection) {
    WindVector wind;
    
    // Set north wind
    wind.setSpeedAndDirection(15.0, 0.0);
    EXPECT_DOUBLE_EQ(wind.speed(), 15.0);
    EXPECT_TRUE(isClose(wind.direction(), 0.0));
    EXPECT_TRUE(isClose(wind.u(), 0.0, 1e-10));
    EXPECT_DOUBLE_EQ(wind.v(), 15.0);
    
    // Set east wind
    wind.setSpeedAndDirection(20.0, M_PI/2);
    EXPECT_DOUBLE_EQ(wind.speed(), 20.0);
    EXPECT_TRUE(isClose(wind.direction(), M_PI/2));
    EXPECT_DOUBLE_EQ(wind.u(), 20.0);
    EXPECT_TRUE(isClose(wind.v(), 0.0, 1e-10));
}

TEST_F(WindVectorTest, InvalidSpeed) {
    WindVector wind;
    EXPECT_THROW(wind.setSpeedAndDirection(-5.0, 0.0), std::invalid_argument);
}

TEST_F(WindVectorTest, CalmWindDetection) {
    WindVector calm_wind(0.01, 0.01, 0.0);
    EXPECT_TRUE(calm_wind.isCalm(0.1));
    EXPECT_FALSE(calm_wind.isCalm(0.001));
    
    WindVector strong_wind(10.0, 5.0, 0.0);
    EXPECT_FALSE(strong_wind.isCalm());
}

TEST_F(WindVectorTest, WindRotation) {
    WindVector north_wind(0.0, 10.0, 0.0);
    
    // Rotate 90 degrees counterclockwise (should become west wind)
    WindVector rotated = north_wind.rotated(M_PI/2);
    EXPECT_TRUE(isClose(rotated.u(), -10.0, 1e-10));
    EXPECT_TRUE(isClose(rotated.v(), 0.0, 1e-10));
    EXPECT_DOUBLE_EQ(rotated.w(), 0.0);
}

class WindShearParametersTest : public ::testing::Test {};

TEST_F(WindShearParametersTest, DefaultParameters) {
    WindShearParameters params;
    EXPECT_TRUE(params.isValid());
    EXPECT_EQ(params.profile, WindShearProfile::UNIFORM);
}

TEST_F(WindShearParametersTest, PowerLawValidation) {
    WindShearParameters params;
    params.profile = WindShearProfile::POWER_LAW;
    params.power_law_exponent = 0.2;
    EXPECT_TRUE(params.isValid());
    
    params.power_law_exponent = -0.1;  // Invalid
    EXPECT_FALSE(params.isValid());
    
    params.power_law_exponent = 1.5;   // Invalid
    EXPECT_FALSE(params.isValid());
}

TEST_F(WindShearParametersTest, LogarithmicValidation) {
    WindShearParameters params;
    params.profile = WindShearProfile::LOGARITHMIC;
    params.reference_height = 10.0;
    params.roughness_length = 0.03;
    EXPECT_TRUE(params.isValid());
    
    params.roughness_length = 15.0;  // Greater than reference height
    EXPECT_FALSE(params.isValid());
    
    params.roughness_length = -0.1;  // Negative
    EXPECT_FALSE(params.isValid());
}

class UniformWindModelTest : public ::testing::Test {
protected:
    void SetUp() override {}
    void TearDown() override {}
    
    bool isClose(double a, double b, double tolerance = 1e-6) {
        return std::abs(a - b) < tolerance;
    }
};

TEST_F(UniformWindModelTest, DefaultConstructor) {
    UniformWindModel model;
    
    Vector3 position(0, 0, 10);  // 10m height
    WindVector wind = model.getWind(position);
    
    EXPECT_GT(wind.speed(), 0.0);  // Should have some default wind
    EXPECT_FALSE(model.hasWindShear());
}

TEST_F(UniformWindModelTest, SpeedDirectionConstructor) {
    UniformWindModel model(15.0, M_PI/4);  // 15 m/s from northeast
    
    Vector3 position(0, 0, 10);
    WindVector wind = model.getWind(position);
    
    EXPECT_DOUBLE_EQ(wind.speed(), 15.0);
    EXPECT_TRUE(isClose(wind.direction(), M_PI/4));
}

TEST_F(UniformWindModelTest, VelocityConstructor) {
    Vector3 wind_velocity(8.0, 6.0, 0.0);  // 10 m/s wind
    UniformWindModel model(wind_velocity);
    
    Vector3 position(0, 0, 10);
    WindVector wind = model.getWind(position);
    
    EXPECT_DOUBLE_EQ(wind.speed(), 10.0);
    EXPECT_DOUBLE_EQ(wind.u(), 8.0);
    EXPECT_DOUBLE_EQ(wind.v(), 6.0);
}

TEST_F(UniformWindModelTest, UniformWindField) {
    UniformWindModel model(12.0, 0.0);  // 12 m/s north wind
    
    // Test at different positions - should be uniform
    std::vector<Vector3> positions = {
        Vector3(0, 0, 10),
        Vector3(100, 200, 10),
        Vector3(-50, -100, 10)
    };
    
    for (const auto& pos : positions) {
        WindVector wind = model.getWind(pos);
        EXPECT_DOUBLE_EQ(wind.speed(), 12.0);
        EXPECT_TRUE(isClose(wind.direction(), 0.0));
    }
}

TEST_F(UniformWindModelTest, PowerLawWindShear) {
    UniformWindModel model(10.0, 0.0);  // 10 m/s north wind
    
    WindShearParameters shear_params;
    shear_params.profile = WindShearProfile::POWER_LAW;
    shear_params.reference_height = 10.0;
    shear_params.reference_speed = 10.0;
    shear_params.power_law_exponent = 0.2;
    
    model.setWindShear(shear_params);
    EXPECT_TRUE(model.hasWindShear());
    
    // Test at reference height - should match reference speed
    WindVector wind_ref = model.getWind(Vector3(0, 0, 10.0));
    EXPECT_TRUE(isClose(wind_ref.speed(), 10.0, 1e-3));
    
    // Test at double height - should be higher speed
    WindVector wind_high = model.getWind(Vector3(0, 0, 20.0));
    double expected_speed = 10.0 * std::pow(2.0, 0.2);  // 2^0.2 ≈ 1.149
    EXPECT_TRUE(isClose(wind_high.speed(), expected_speed, 1e-3));
    
    // Test at half height - should be lower speed
    WindVector wind_low = model.getWind(Vector3(0, 0, 5.0));
    expected_speed = 10.0 * std::pow(0.5, 0.2);  // 0.5^0.2 ≈ 0.871
    EXPECT_TRUE(isClose(wind_low.speed(), expected_speed, 1e-3));
}

TEST_F(UniformWindModelTest, LogarithmicWindShear) {
    UniformWindModel model(10.0, 0.0);  // 10 m/s north wind
    
    WindShearParameters shear_params;
    shear_params.profile = WindShearProfile::LOGARITHMIC;
    shear_params.reference_height = 10.0;
    shear_params.reference_speed = 10.0;
    shear_params.roughness_length = 0.03;
    
    model.setWindShear(shear_params);
    
    // Test at reference height
    WindVector wind_ref = model.getWind(Vector3(0, 0, 10.0));
    EXPECT_TRUE(isClose(wind_ref.speed(), 10.0, 1e-3));
    
    // Test at higher altitude
    WindVector wind_high = model.getWind(Vector3(0, 0, 50.0));
    double log_50 = std::log(50.0 / 0.03);
    double log_10 = std::log(10.0 / 0.03);
    double expected_speed = 10.0 * log_50 / log_10;
    EXPECT_TRUE(isClose(wind_high.speed(), expected_speed, 1e-3));
    EXPECT_GT(wind_high.speed(), 10.0);  // Should be higher than reference
}

TEST_F(UniformWindModelTest, GroundLevelWind) {
    UniformWindModel model(10.0, 0.0);
    
    WindShearParameters shear_params;
    shear_params.profile = WindShearProfile::POWER_LAW;
    shear_params.reference_height = 10.0;
    shear_params.reference_speed = 10.0;
    shear_params.power_law_exponent = 0.2;
    
    model.setWindShear(shear_params);
    
    // Test at ground level (z = 0)
    WindVector wind_ground = model.getWind(Vector3(0, 0, 0.0));
    EXPECT_LT(wind_ground.speed(), 2.0);  // Should be much reduced
    
    // Test below ground (z < 0)
    WindVector wind_below = model.getWind(Vector3(0, 0, -5.0));
    EXPECT_LT(wind_below.speed(), 2.0);  // Should be much reduced
}

TEST_F(UniformWindModelTest, InvalidShearParameters) {
    UniformWindModel model(10.0, 0.0);
    
    WindShearParameters invalid_params;
    invalid_params.profile = WindShearProfile::POWER_LAW;
    invalid_params.power_law_exponent = -0.1;  // Invalid
    
    EXPECT_THROW(model.setWindShear(invalid_params), std::invalid_argument);
}

class WindModelFactoryTest : public ::testing::Test {};

TEST_F(WindModelFactoryTest, CreateUniformWindModel) {
    auto model = createUniformWindModel(15.0, M_PI/3);
    EXPECT_NE(model, nullptr);
    
    WindVector wind = model->getWind(Vector3(0, 0, 10));
    EXPECT_DOUBLE_EQ(wind.speed(), 15.0);
}

TEST_F(WindModelFactoryTest, CreateUniformWindModelWithVelocity) {
    Vector3 velocity(12.0, 9.0, 0.0);
    auto model = createUniformWindModel(velocity);
    EXPECT_NE(model, nullptr);
    
    WindVector wind = model->getWind(Vector3(0, 0, 10));
    EXPECT_DOUBLE_EQ(wind.speed(), 15.0);  // sqrt(12^2 + 9^2) = 15
    EXPECT_DOUBLE_EQ(wind.u(), 12.0);
    EXPECT_DOUBLE_EQ(wind.v(), 9.0);
}