#include "kite_sim/kite_state.h"
#include <gtest/gtest.h>
#include <cmath>
#include <limits>

using namespace kite_sim;

class KiteStateTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Set up test data
        position_ = Vector3(10.0, 20.0, 30.0);
        velocity_ = Vector3(5.0, 0.0, -2.0);
        attitude_ = Quaternion::fromEulerAngles(0.1, 0.2, 0.3);
        angular_velocity_ = Vector3(0.01, 0.02, 0.03);
        time_ = 5.5;
    }

    Vector3 position_;
    Vector3 velocity_;
    Quaternion attitude_;
    Vector3 angular_velocity_;
    double time_;
};

TEST_F(KiteStateTest, DefaultConstructor) {
    KiteState state;
    
    EXPECT_TRUE(state.position().isZero());
    EXPECT_TRUE(state.velocity().isZero());
    EXPECT_TRUE(state.attitude().isUnit());
    EXPECT_TRUE(state.angularVelocity().isZero());
    EXPECT_EQ(state.time(), 0.0);
    EXPECT_TRUE(state.isValid());
}

TEST_F(KiteStateTest, ParameterizedConstructor) {
    KiteState state(position_, velocity_, attitude_, angular_velocity_, time_);
    
    EXPECT_EQ(state.position().x(), position_.x());
    EXPECT_EQ(state.position().y(), position_.y());
    EXPECT_EQ(state.position().z(), position_.z());
    
    EXPECT_EQ(state.velocity().x(), velocity_.x());
    EXPECT_EQ(state.velocity().y(), velocity_.y());
    EXPECT_EQ(state.velocity().z(), velocity_.z());
    
    EXPECT_TRUE(state.attitude().isUnit());
    
    EXPECT_EQ(state.angularVelocity().x(), angular_velocity_.x());
    EXPECT_EQ(state.angularVelocity().y(), angular_velocity_.y());
    EXPECT_EQ(state.angularVelocity().z(), angular_velocity_.z());
    
    EXPECT_EQ(state.time(), time_);
    EXPECT_TRUE(state.isValid());
}

TEST_F(KiteStateTest, CopyConstructor) {
    KiteState original(position_, velocity_, attitude_, angular_velocity_, time_);
    KiteState copy(original);
    
    EXPECT_EQ(copy, original);
    EXPECT_TRUE(copy.isValid());
}

TEST_F(KiteStateTest, AssignmentOperator) {
    KiteState original(position_, velocity_, attitude_, angular_velocity_, time_);
    KiteState assigned;
    
    assigned = original;
    
    EXPECT_EQ(assigned, original);
    EXPECT_TRUE(assigned.isValid());
}

TEST_F(KiteStateTest, SettersAndGetters) {
    KiteState state;
    
    state.setPosition(position_);
    state.setVelocity(velocity_);
    state.setAttitude(attitude_);
    state.setAngularVelocity(angular_velocity_);
    state.setTime(time_);
    
    EXPECT_EQ(state.position().x(), position_.x());
    EXPECT_EQ(state.velocity().y(), velocity_.y());
    EXPECT_TRUE(state.attitude().isUnit());
    EXPECT_EQ(state.angularVelocity().z(), angular_velocity_.z());
    EXPECT_EQ(state.time(), time_);
}

TEST_F(KiteStateTest, StateVectorConversion) {
    KiteState original(position_, velocity_, attitude_, angular_velocity_, time_);
    
    double state_vector[KiteState::getStateSize()];
    original.toStateVector(state_vector);
    
    KiteState reconstructed;
    reconstructed.fromStateVector(state_vector);
    reconstructed.setTime(time_); // Time is not part of state vector
    
    // Check that reconstruction is close to original
    const double tolerance = 1e-10;
    EXPECT_NEAR(reconstructed.position().x(), original.position().x(), tolerance);
    EXPECT_NEAR(reconstructed.position().y(), original.position().y(), tolerance);
    EXPECT_NEAR(reconstructed.position().z(), original.position().z(), tolerance);
    
    EXPECT_NEAR(reconstructed.velocity().x(), original.velocity().x(), tolerance);
    EXPECT_NEAR(reconstructed.velocity().y(), original.velocity().y(), tolerance);
    EXPECT_NEAR(reconstructed.velocity().z(), original.velocity().z(), tolerance);
    
    EXPECT_NEAR(reconstructed.attitude().w(), original.attitude().w(), tolerance);
    EXPECT_NEAR(reconstructed.attitude().x(), original.attitude().x(), tolerance);
    EXPECT_NEAR(reconstructed.attitude().y(), original.attitude().y(), tolerance);
    EXPECT_NEAR(reconstructed.attitude().z(), original.attitude().z(), tolerance);
    
    EXPECT_NEAR(reconstructed.angularVelocity().x(), original.angularVelocity().x(), tolerance);
    EXPECT_NEAR(reconstructed.angularVelocity().y(), original.angularVelocity().y(), tolerance);
    EXPECT_NEAR(reconstructed.angularVelocity().z(), original.angularVelocity().z(), tolerance);
}

TEST_F(KiteStateTest, StateSize) {
    EXPECT_EQ(KiteState::getStateSize(), 13);
}

TEST_F(KiteStateTest, Reset) {
    KiteState state(position_, velocity_, attitude_, angular_velocity_, time_);
    
    state.reset();
    
    EXPECT_TRUE(state.position().isZero());
    EXPECT_TRUE(state.velocity().isZero());
    EXPECT_TRUE(state.attitude().isUnit());
    EXPECT_TRUE(state.angularVelocity().isZero());
    EXPECT_EQ(state.time(), 0.0);
    EXPECT_TRUE(state.isValid());
}

TEST_F(KiteStateTest, Validation) {
    KiteState valid_state(position_, velocity_, attitude_, angular_velocity_, time_);
    EXPECT_TRUE(valid_state.isValid());
    
    // Test invalid position (NaN)
    KiteState invalid_pos;
    invalid_pos.setPosition(Vector3(std::nan(""), 0.0, 0.0));
    EXPECT_FALSE(invalid_pos.isValid());
    
    // Test invalid time (infinity)
    KiteState invalid_time;
    invalid_time.setTime(std::numeric_limits<double>::infinity());
    EXPECT_FALSE(invalid_time.isValid());
}

TEST_F(KiteStateTest, JsonSerialization) {
    KiteState original(position_, velocity_, attitude_, angular_velocity_, time_);
    
    std::string json = original.toJson();
    EXPECT_FALSE(json.empty());
    EXPECT_NE(json.find("position"), std::string::npos);
    EXPECT_NE(json.find("velocity"), std::string::npos);
    EXPECT_NE(json.find("attitude"), std::string::npos);
    EXPECT_NE(json.find("angular_velocity"), std::string::npos);
    EXPECT_NE(json.find("time"), std::string::npos);
    
    // Basic deserialization test (implementation is minimal)
    KiteState deserialized;
    bool success = deserialized.fromJson(json);
    EXPECT_TRUE(success);
}

TEST_F(KiteStateTest, EqualityOperators) {
    KiteState state1(position_, velocity_, attitude_, angular_velocity_, time_);
    KiteState state2(position_, velocity_, attitude_, angular_velocity_, time_);
    KiteState state3(Vector3(0, 0, 0), velocity_, attitude_, angular_velocity_, time_);
    
    EXPECT_EQ(state1, state2);
    EXPECT_NE(state1, state3);
    EXPECT_FALSE(state1 != state2);
    EXPECT_TRUE(state1 != state3);
}