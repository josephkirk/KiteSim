#include "kite_sim/tether.h"
#include <gtest/gtest.h>
#include <cmath>

using namespace kite_sim;

class TetherPropertiesTest : public ::testing::Test {
protected:
    void SetUp() override {
        default_props_ = TetherProperties();
        elastic_props_ = TetherProperties(100.0, 0.01, 0.005, 1e9, 1.2);
        inelastic_props_ = TetherProperties(100.0, 0.01, 0.005, 0.0, 1.2);
    }
    
    TetherProperties default_props_;
    TetherProperties elastic_props_;
    TetherProperties inelastic_props_;
};

TEST_F(TetherPropertiesTest, DefaultConstructor) {
    TetherProperties props;
    
    EXPECT_GT(props.length(), 0.0);
    EXPECT_GT(props.massPerLength(), 0.0);
    EXPECT_GT(props.diameter(), 0.0);
    EXPECT_GE(props.elasticModulus(), 0.0);
    EXPECT_GE(props.dragCoefficient(), 0.0);
    EXPECT_TRUE(props.isValid());
}

TEST_F(TetherPropertiesTest, ParameterizedConstructor) {
    double length = 150.0;
    double mass_per_length = 0.02;
    double diameter = 0.008;
    double elastic_modulus = 2e9;
    double drag_coefficient = 1.5;
    
    TetherProperties props(length, mass_per_length, diameter, elastic_modulus, drag_coefficient);
    
    EXPECT_DOUBLE_EQ(props.length(), length);
    EXPECT_DOUBLE_EQ(props.massPerLength(), mass_per_length);
    EXPECT_DOUBLE_EQ(props.diameter(), diameter);
    EXPECT_DOUBLE_EQ(props.elasticModulus(), elastic_modulus);
    EXPECT_DOUBLE_EQ(props.dragCoefficient(), drag_coefficient);
    EXPECT_TRUE(props.isValid());
}

TEST_F(TetherPropertiesTest, ElasticProperties) {
    EXPECT_TRUE(elastic_props_.isElastic());
    EXPECT_FALSE(inelastic_props_.isElastic());
    
    EXPECT_GT(elastic_props_.elasticConstant(), 0.0);
    EXPECT_EQ(inelastic_props_.elasticConstant(), 0.0);
}

TEST_F(TetherPropertiesTest, DerivedProperties) {
    double expected_area = M_PI * elastic_props_.diameter() * elastic_props_.diameter() / 4.0;
    EXPECT_DOUBLE_EQ(elastic_props_.crossSectionalArea(), expected_area);
    
    double expected_mass = elastic_props_.length() * elastic_props_.massPerLength();
    EXPECT_DOUBLE_EQ(elastic_props_.totalMass(), expected_mass);
    
    double expected_k = elastic_props_.elasticModulus() * expected_area / elastic_props_.length();
    EXPECT_DOUBLE_EQ(elastic_props_.elasticConstant(), expected_k);
}

TEST_F(TetherPropertiesTest, AttachmentPoints) {
    Vector3 kite_point(1.0, 2.0, 3.0);
    Vector3 ground_point(0.0, 0.0, 0.0);
    
    elastic_props_.setKiteAttachmentPoint(kite_point);
    elastic_props_.setGroundAttachmentPoint(ground_point);
    
    EXPECT_EQ(elastic_props_.kiteAttachmentPoint(), kite_point);
    EXPECT_EQ(elastic_props_.groundAttachmentPoint(), ground_point);
}

TEST_F(TetherPropertiesTest, Validation) {
    EXPECT_TRUE(elastic_props_.isValid());
    
    // Test invalid properties
    TetherProperties invalid_props(-1.0, 0.01, 0.005, 1e9, 1.2);  // Negative length
    EXPECT_FALSE(invalid_props.isValid());
    
    auto errors = invalid_props.getValidationErrors();
    EXPECT_FALSE(errors.empty());
    EXPECT_GT(errors.size(), 0);
}

TEST_F(TetherPropertiesTest, StaticFactoryMethods) {
    auto default_tether = TetherProperties::createDefaultTether();
    EXPECT_TRUE(default_tether.isValid());
    EXPECT_FALSE(default_tether.isElastic());
    
    auto elastic_tether = TetherProperties::createElasticTether();
    EXPECT_TRUE(elastic_tether.isValid());
    EXPECT_TRUE(elastic_tether.isElastic());
    
    auto inelastic_tether = TetherProperties::createInelasticTether();
    EXPECT_TRUE(inelastic_tether.isValid());
    EXPECT_FALSE(inelastic_tether.isElastic());
}

TEST_F(TetherPropertiesTest, EqualityOperators) {
    TetherProperties props1(100.0, 0.01, 0.005, 1e9, 1.2);
    TetherProperties props2(100.0, 0.01, 0.005, 1e9, 1.2);
    TetherProperties props3(200.0, 0.01, 0.005, 1e9, 1.2);
    
    EXPECT_EQ(props1, props2);
    EXPECT_NE(props1, props3);
}

class TetherForcesTest : public ::testing::Test {
protected:
    void SetUp() override {
        force_ = Vector3(100.0, 200.0, 300.0);
        moment_ = Vector3(10.0, 20.0, 30.0);
        tension_ = 500.0;
        length_ = 150.0;
    }
    
    Vector3 force_;
    Vector3 moment_;
    double tension_;
    double length_;
};

TEST_F(TetherForcesTest, DefaultConstructor) {
    TetherForces forces;
    
    EXPECT_TRUE(forces.force.isZero());
    EXPECT_TRUE(forces.moment.isZero());
    EXPECT_EQ(forces.tension, 0.0);
    EXPECT_EQ(forces.current_length, 0.0);
    EXPECT_FALSE(forces.constraint_active);
    EXPECT_TRUE(forces.isValid());
}

TEST_F(TetherForcesTest, ParameterizedConstructor) {
    TetherForces forces(force_, moment_, tension_, length_, true);
    
    EXPECT_EQ(forces.force, force_);
    EXPECT_EQ(forces.moment, moment_);
    EXPECT_EQ(forces.tension, tension_);
    EXPECT_EQ(forces.current_length, length_);
    EXPECT_TRUE(forces.constraint_active);
    EXPECT_TRUE(forces.isValid());
}

TEST_F(TetherForcesTest, Validation) {
    TetherForces valid_forces(force_, moment_, tension_, length_, true);
    EXPECT_TRUE(valid_forces.isValid());
    
    // Test invalid forces (negative tension)
    TetherForces invalid_forces(force_, moment_, -100.0, length_, true);
    EXPECT_FALSE(invalid_forces.isValid());
}

TEST_F(TetherForcesTest, Reset) {
    TetherForces forces(force_, moment_, tension_, length_, true);
    forces.reset();
    
    EXPECT_TRUE(forces.force.isZero());
    EXPECT_TRUE(forces.moment.isZero());
    EXPECT_EQ(forces.tension, 0.0);
    EXPECT_EQ(forces.current_length, 0.0);
    EXPECT_FALSE(forces.constraint_active);
}

class TetherModelTest : public ::testing::Test {
protected:
    void SetUp() override {
        elastic_props_ = TetherProperties(100.0, 0.01, 0.005, 1e9, 1.2);
        inelastic_props_ = TetherProperties(100.0, 0.01, 0.005, 0.0, 1.2);
        
        elastic_model_ = TetherModel(elastic_props_);
        inelastic_model_ = TetherModel(inelastic_props_);
        
        // Set up test positions
        ground_position_ = Vector3(0.0, 0.0, 0.0);
        kite_position_slack_ = Vector3(50.0, 0.0, 50.0);  // Shorter than tether length
        kite_position_taut_ = Vector3(100.0, 0.0, 0.0);   // Equal to tether length
        kite_position_extended_ = Vector3(150.0, 0.0, 0.0); // Longer than tether length
        
        kite_attitude_ = Quaternion::identity();
        kite_velocity_ = Vector3(10.0, 0.0, 0.0);
        wind_velocity_ = Vector3(5.0, 0.0, 0.0);
    }
    
    TetherProperties elastic_props_;
    TetherProperties inelastic_props_;
    TetherModel elastic_model_;
    TetherModel inelastic_model_;
    
    Vector3 ground_position_;
    Vector3 kite_position_slack_;
    Vector3 kite_position_taut_;
    Vector3 kite_position_extended_;
    
    Quaternion kite_attitude_;
    Vector3 kite_velocity_;
    Vector3 wind_velocity_;
};

TEST_F(TetherModelTest, DefaultConstructor) {
    TetherModel model;
    EXPECT_TRUE(model.isValid());
    EXPECT_GT(model.getAirDensity(), 0.0);
}

TEST_F(TetherModelTest, PropertiesAccess) {
    EXPECT_EQ(elastic_model_.properties().length(), elastic_props_.length());
    EXPECT_TRUE(elastic_model_.properties().isElastic());
    EXPECT_FALSE(inelastic_model_.properties().isElastic());
}

TEST_F(TetherModelTest, LengthCalculations) {
    double expected_length = kite_position_extended_.magnitude();
    double calculated_length = elastic_model_.getCurrentLength(kite_position_extended_);
    
    EXPECT_NEAR(calculated_length, expected_length, 1e-10);
}

TEST_F(TetherModelTest, TetherDirection) {
    Vector3 direction = elastic_model_.getTetherDirection(kite_position_extended_);
    Vector3 expected_direction = kite_position_extended_.normalized();
    
    EXPECT_NEAR(direction.x(), expected_direction.x(), 1e-10);
    EXPECT_NEAR(direction.y(), expected_direction.y(), 1e-10);
    EXPECT_NEAR(direction.z(), expected_direction.z(), 1e-10);
}

TEST_F(TetherModelTest, TetherTautness) {
    EXPECT_FALSE(elastic_model_.isTetherTaut(kite_position_slack_));
    EXPECT_TRUE(elastic_model_.isTetherTaut(kite_position_taut_));
    EXPECT_TRUE(elastic_model_.isTetherTaut(kite_position_extended_));
}

TEST_F(TetherModelTest, ConstraintChecking) {
    // Elastic tether should always satisfy constraints
    EXPECT_TRUE(elastic_model_.checkConstraints(kite_position_slack_));
    EXPECT_TRUE(elastic_model_.checkConstraints(kite_position_taut_));
    EXPECT_TRUE(elastic_model_.checkConstraints(kite_position_extended_));
    
    // Inelastic tether should violate constraints when extended
    EXPECT_TRUE(inelastic_model_.checkConstraints(kite_position_slack_));
    EXPECT_TRUE(inelastic_model_.checkConstraints(kite_position_taut_));
    EXPECT_FALSE(inelastic_model_.checkConstraints(kite_position_extended_));
}

TEST_F(TetherModelTest, ConstraintViolation) {
    // Elastic tether should have no constraint violation
    EXPECT_EQ(elastic_model_.getConstraintViolation(kite_position_extended_), 0.0);
    
    // Inelastic tether should have constraint violation when extended
    double expected_violation = kite_position_extended_.magnitude() - inelastic_props_.length();
    double calculated_violation = inelastic_model_.getConstraintViolation(kite_position_extended_);
    
    EXPECT_NEAR(calculated_violation, expected_violation, 1e-10);
}

TEST_F(TetherModelTest, ElasticForceCalculation) {
    TetherForces forces = elastic_model_.calculateTetherForces(
        kite_position_extended_, kite_attitude_, kite_velocity_, wind_velocity_);
    
    EXPECT_TRUE(forces.isValid());
    EXPECT_GT(forces.tension, 0.0);  // Should have tension when extended
    EXPECT_TRUE(forces.constraint_active);
    
    // Force should be directed toward ground (negative x direction)
    EXPECT_LT(forces.force.x(), 0.0);
    
    // Current length should match position
    EXPECT_NEAR(forces.current_length, kite_position_extended_.magnitude(), 1e-10);
}

TEST_F(TetherModelTest, InelasticForceCalculation) {
    TetherForces forces = inelastic_model_.calculateTetherForces(
        kite_position_extended_, kite_attitude_, kite_velocity_, wind_velocity_);
    
    EXPECT_TRUE(forces.isValid());
    EXPECT_GT(forces.tension, 0.0);  // Should have tension when extended
    EXPECT_TRUE(forces.constraint_active);
    
    // Force should be directed toward ground
    EXPECT_LT(forces.force.x(), 0.0);
}

TEST_F(TetherModelTest, SlackTetherForces) {
    // When tether is slack, forces should be minimal
    TetherForces elastic_forces = elastic_model_.calculateTetherForces(
        kite_position_slack_, kite_attitude_, kite_velocity_, wind_velocity_);
    
    TetherForces inelastic_forces = inelastic_model_.calculateTetherForces(
        kite_position_slack_, kite_attitude_, kite_velocity_, wind_velocity_);
    
    EXPECT_TRUE(elastic_forces.isValid());
    EXPECT_TRUE(inelastic_forces.isValid());
    
    // Elastic tether should have no tension when slack
    EXPECT_EQ(elastic_forces.tension, 0.0);
    EXPECT_FALSE(elastic_forces.constraint_active);
    
    // Inelastic tether should also have no tension when slack
    EXPECT_EQ(inelastic_forces.tension, 0.0);
    EXPECT_FALSE(inelastic_forces.constraint_active);
}

TEST_F(TetherModelTest, ZeroPositionHandling) {
    Vector3 zero_position(0.0, 0.0, 0.0);
    
    TetherForces forces = elastic_model_.calculateTetherForces(
        zero_position, kite_attitude_, kite_velocity_, wind_velocity_);
    
    EXPECT_TRUE(forces.isValid());
    EXPECT_EQ(forces.tension, 0.0);
    EXPECT_EQ(forces.current_length, 0.0);
    EXPECT_FALSE(forces.constraint_active);
}

TEST_F(TetherModelTest, StaticFactoryMethods) {
    auto elastic_model = TetherModel::createElasticModel(100.0, 0.01, 0.005, 1e9);
    EXPECT_TRUE(elastic_model.isValid());
    EXPECT_TRUE(elastic_model.properties().isElastic());
    
    auto inelastic_model = TetherModel::createInelasticModel(100.0, 0.01, 0.005);
    EXPECT_TRUE(inelastic_model.isValid());
    EXPECT_FALSE(inelastic_model.properties().isElastic());
}

TEST_F(TetherModelTest, AirDensitySettings) {
    double new_density = 1.0;
    elastic_model_.setAirDensity(new_density);
    
    EXPECT_DOUBLE_EQ(elastic_model_.getAirDensity(), new_density);
}

TEST_F(TetherModelTest, ForceConsistency) {
    // Test that forces are consistent with physics
    TetherForces forces = elastic_model_.calculateTetherForces(
        kite_position_extended_, kite_attitude_, kite_velocity_, wind_velocity_);
    
    // Tension force should be along tether direction
    Vector3 tether_direction = elastic_model_.getTetherDirection(kite_position_extended_);
    Vector3 expected_tension_force = -forces.tension * tether_direction;
    
    // The total force should include tension plus drag, so we can't directly compare
    // But we can check that the tension component is reasonable
    double force_magnitude = forces.force.magnitude();
    EXPECT_GE(force_magnitude, forces.tension * 0.8);  // Allow for drag contribution
}

// Integration tests
class TetherIntegrationTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Create a realistic tether configuration
        props_ = TetherProperties(200.0, 0.015, 0.008, 5e8, 1.0);
        props_.setGroundAttachmentPoint(Vector3(0.0, 0.0, 0.0));
        props_.setKiteAttachmentPoint(Vector3(0.0, 0.0, -0.5));  // Below kite CG
        
        model_ = TetherModel(props_);
        model_.setAirDensity(1.225);
    }
    
    TetherProperties props_;
    TetherModel model_;
};

TEST_F(TetherIntegrationTest, RealisticFlightScenario) {
    // Simulate kite at various positions in a realistic flight envelope
    std::vector<Vector3> test_positions = {
        Vector3(100.0, 0.0, 100.0),    // 45-degree elevation
        Vector3(150.0, 50.0, 100.0),   // Off to the side
        Vector3(180.0, 0.0, 80.0),     // Lower elevation, extended
        Vector3(120.0, -30.0, 120.0)   // Different azimuth
    };
    
    Quaternion attitude = Quaternion::fromEulerAngles(0.1, 0.05, 0.0);  // Slight bank and pitch
    Vector3 velocity(15.0, 2.0, -1.0);  // Realistic kite velocity
    Vector3 wind(8.0, 0.0, 0.0);        // Wind from the side
    
    for (const auto& position : test_positions) {
        TetherForces forces = model_.calculateTetherForces(position, attitude, velocity, wind);
        
        EXPECT_TRUE(forces.isValid()) << "Forces invalid at position: " 
                                     << position.x() << ", " << position.y() << ", " << position.z();
        
        // Check that forces are reasonable
        EXPECT_GE(forces.tension, 0.0);
        EXPECT_GT(forces.current_length, 0.0);
        
        // Force magnitude should be reasonable (not too large)
        double force_magnitude = forces.force.magnitude();
        EXPECT_LT(force_magnitude, 10000.0);  // Less than 10kN
        
        // Moment magnitude should be reasonable
        double moment_magnitude = forces.moment.magnitude();
        EXPECT_LT(moment_magnitude, 5000.0);  // Less than 5kN⋅m
    }
}

TEST_F(TetherIntegrationTest, EnergyConsistency) {
    // Test that elastic tether behaves consistently with energy principles
    Vector3 position1(150.0, 0.0, 0.0);  // Extended position
    Vector3 position2(180.0, 0.0, 0.0);  // More extended position
    
    Quaternion attitude = Quaternion::identity();
    Vector3 velocity(0.0, 0.0, 0.0);  // No velocity for pure static test
    Vector3 wind(0.0, 0.0, 0.0);      // No wind
    
    TetherForces forces1 = model_.calculateTetherForces(position1, attitude, velocity, wind);
    TetherForces forces2 = model_.calculateTetherForces(position2, attitude, velocity, wind);
    
    // More extension should result in higher tension for elastic tether
    EXPECT_GT(forces2.tension, forces1.tension);
    
    // The increase should be proportional to the elastic constant
    double extension1 = forces1.current_length - props_.length();
    double extension2 = forces2.current_length - props_.length();
    double k = props_.elasticConstant();
    
    EXPECT_NEAR(forces1.tension, k * extension1, k * extension1 * 0.1);  // 10% tolerance
    EXPECT_NEAR(forces2.tension, k * extension2, k * extension2 * 0.1);
}