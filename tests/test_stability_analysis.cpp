#include <gtest/gtest.h>
#include "kite_sim/stability_analysis.h"
#include "kite_sim/kite_geometry.h"
#include "kite_sim/aerodynamics.h"
#include "kite_sim/dynamics.h"
#include "kite_sim/wind_model.h"
#include "kite_sim/tether.h"
#include <memory>
#include <cmath>

using namespace kite_sim;

class StabilityAnalysisTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Create test geometry
        geometry_ = KiteGeometry::createDefaultKite();
        
        // Create modules
        aero_calc_ = std::make_shared<AerodynamicsCalculator>(geometry_);
        dynamics_engine_ = std::make_shared<DynamicsEngine>(IntegrationMethod::RUNGE_KUTTA_4);
        dynamics_engine_->setMassProperties(geometry_.massProperties());
        
        tether_model_ = std::make_shared<TetherModel>();
        tether_model_->setProperties(TetherProperties::createDefaultTether());
        
        wind_model_ = std::make_shared<UniformWindModel>(10.0, 0.0);  // 10 m/s wind
        
        // Create stability analyzer
        analyzer_ = std::make_unique<StabilityAnalyzer>();
        analyzer_->setKiteGeometry(geometry_);
        analyzer_->setAerodynamicsCalculator(aero_calc_);
        analyzer_->setDynamicsEngine(dynamics_engine_);
        analyzer_->setTetherModel(tether_model_);
        analyzer_->setWindModel(wind_model_);
        
        // Create test initial state
        initial_state_.setPosition(Vector3(0.0, 0.0, 80.0));  // 80m altitude
        initial_state_.setVelocity(Vector3(15.0, 0.0, 0.0));   // 15 m/s forward
        initial_state_.setAttitude(Quaternion(1.0, 0.0, 0.0, 0.0));  // Level
        initial_state_.setAngularVelocity(Vector3(0.0, 0.0, 0.0));
        initial_state_.setTime(0.0);
    }
    
    KiteGeometry geometry_;
    std::shared_ptr<AerodynamicsCalculator> aero_calc_;
    std::shared_ptr<DynamicsEngine> dynamics_engine_;
    std::shared_ptr<TetherModel> tether_model_;
    std::shared_ptr<WindModel> wind_model_;
    std::unique_ptr<StabilityAnalyzer> analyzer_;
    KiteState initial_state_;
};

// TrimCondition tests
TEST_F(StabilityAnalysisTest, TrimConditionValidation) {
    TrimCondition trim;
    EXPECT_FALSE(trim.isValid());  // Default constructed should be invalid
    
    trim.state = initial_state_;
    trim.controls = ControlInputs();
    trim.total_forces = Forces();
    trim.force_residual = 0.5;
    trim.moment_residual = 0.05;
    trim.trim_quality = 0.9;
    trim.converged = true;
    
    EXPECT_TRUE(trim.isValid());
    EXPECT_TRUE(trim.isTrimmed(1.0, 0.1));  // Within tolerance
    EXPECT_FALSE(trim.isTrimmed(0.1, 0.01)); // Outside tolerance
}

TEST_F(StabilityAnalysisTest, TrimConditionToString) {
    TrimCondition trim;
    trim.converged = true;
    trim.iterations = 25;
    trim.force_residual = 0.8;
    trim.moment_residual = 0.06;
    trim.trim_quality = 0.85;
    trim.controls.elevator = 0.1;
    
    std::string str = trim.toString();
    EXPECT_TRUE(str.find("Converged: Yes") != std::string::npos);
    EXPECT_TRUE(str.find("Iterations: 25") != std::string::npos);
    EXPECT_TRUE(str.find("Force Residual: 0.8") != std::string::npos);
}

// TrimParameters tests
TEST_F(StabilityAnalysisTest, TrimParametersValidation) {
    TrimParameters params;
    EXPECT_TRUE(params.isValid());  // Default should be valid
    
    params.force_tolerance = -1.0;  // Invalid
    EXPECT_FALSE(params.isValid());
    
    params.reset();
    EXPECT_TRUE(params.isValid());
    
    params.step_reduction = 1.5;  // Invalid (should be < 1.0)
    EXPECT_FALSE(params.isValid());
}

// StabilityDerivatives tests
TEST_F(StabilityAnalysisTest, StabilityDerivativesValidation) {
    StabilityDerivatives derivatives;
    EXPECT_TRUE(derivatives.isValid());  // Default should be valid
    
    derivatives.CL_alpha = std::numeric_limits<double>::infinity();
    EXPECT_FALSE(derivatives.isValid());
    
    derivatives.reset();
    EXPECT_TRUE(derivatives.isValid());
}

TEST_F(StabilityAnalysisTest, StabilityDerivativesToString) {
    StabilityDerivatives derivatives;
    derivatives.CL_alpha = 5.2;
    derivatives.CD_alpha = 0.8;
    derivatives.Cm_alpha = -1.5;
    derivatives.CL_elevator = 0.6;
    
    std::string str = derivatives.toString();
    EXPECT_TRUE(str.find("CL_alpha = 5.2") != std::string::npos);
    EXPECT_TRUE(str.find("CD_alpha = 0.8") != std::string::npos);
    EXPECT_TRUE(str.find("Cm_alpha = -1.5") != std::string::npos);
    EXPECT_TRUE(str.find("CL_elevator = 0.6") != std::string::npos);
}

// LinearizedSystem tests
TEST_F(StabilityAnalysisTest, LinearizedSystemValidation) {
    LinearizedSystem system;
    EXPECT_FALSE(system.isValid());  // Default should be invalid
    
    system.state_size = 13;
    system.control_size = 3;
    system.output_size = 13;
    system.A = Matrix::zeros(13, 13);
    system.B = Matrix::zeros(13, 3);
    system.C = Matrix::identity(13, 13);
    system.D = Matrix::zeros(13, 3);
    system.trim_state = initial_state_;
    system.trim_controls = ControlInputs();
    
    EXPECT_TRUE(system.isValid());
}

// FlightEnvelope tests
TEST_F(StabilityAnalysisTest, FlightEnvelopeValidation) {
    FlightEnvelope envelope;
    EXPECT_FALSE(envelope.isValid());  // Default should be invalid
    
    envelope.min_airspeed = 5.0;
    envelope.max_airspeed = 30.0;
    envelope.min_angle_of_attack = -0.2;
    envelope.max_angle_of_attack = 0.4;
    envelope.stall_angle = 0.3;
    envelope.max_lift_to_drag = 15.0;
    envelope.best_glide_speed = 18.0;
    envelope.min_sink_speed = 15.0;
    
    EXPECT_TRUE(envelope.isValid());
}

// StabilityAnalyzer configuration tests
TEST_F(StabilityAnalysisTest, AnalyzerConfiguration) {
    StabilityAnalyzer analyzer;
    EXPECT_FALSE(analyzer.isConfigured());
    
    auto errors = analyzer.getConfigurationErrors();
    EXPECT_FALSE(errors.empty());
    EXPECT_TRUE(std::find_if(errors.begin(), errors.end(),
        [](const std::string& error) { return error.find("geometry") != std::string::npos; }) != errors.end());
    
    // Configure step by step
    analyzer.setKiteGeometry(geometry_);
    analyzer.setAerodynamicsCalculator(aero_calc_);
    analyzer.setDynamicsEngine(dynamics_engine_);
    analyzer.setTetherModel(tether_model_);
    analyzer.setWindModel(wind_model_);
    
    EXPECT_TRUE(analyzer.isConfigured());
    EXPECT_TRUE(analyzer.getConfigurationErrors().empty());
}

TEST_F(StabilityAnalysisTest, AnalyzerConfigurationErrors) {
    StabilityAnalyzer analyzer;
    
    // Test null pointer handling
    EXPECT_THROW(analyzer.setAerodynamicsCalculator(nullptr), std::invalid_argument);
    EXPECT_THROW(analyzer.setDynamicsEngine(nullptr), std::invalid_argument);
    EXPECT_THROW(analyzer.setTetherModel(nullptr), std::invalid_argument);
    EXPECT_THROW(analyzer.setWindModel(nullptr), std::invalid_argument);
    
    // Test invalid geometry
    KiteGeometry invalid_geometry;  // Default constructed is invalid
    EXPECT_THROW(analyzer.setKiteGeometry(invalid_geometry), std::invalid_argument);
}

// Trim condition calculation tests
TEST_F(StabilityAnalysisTest, FindTrimConditionBasic) {
    ASSERT_TRUE(analyzer_->isConfigured());
    
    TrimCondition trim = analyzer_->findTrimCondition(initial_state_);
    
    EXPECT_TRUE(trim.isValid());
    EXPECT_TRUE(trim.state.isValid());
    EXPECT_TRUE(trim.controls.isValid());
    EXPECT_GE(trim.iterations, 0);
    EXPECT_LE(trim.iterations, 100);  // Should not exceed max iterations
    
    // Check that forces are reasonable (may not be perfectly trimmed due to simplified optimization)
    EXPECT_LT(trim.force_residual, 100.0);   // Should be reasonable
    EXPECT_LT(trim.moment_residual, 10.0);   // Should be reasonable
    EXPECT_GE(trim.trim_quality, 0.0);
    EXPECT_LE(trim.trim_quality, 1.0);
}

TEST_F(StabilityAnalysisTest, FindTrimConditionWithControls) {
    ASSERT_TRUE(analyzer_->isConfigured());
    
    ControlInputs initial_controls;
    initial_controls.elevator = 0.05;  // Small initial elevator deflection
    
    TrimCondition trim = analyzer_->findTrimCondition(initial_state_, initial_controls);
    
    EXPECT_TRUE(trim.isValid());
    EXPECT_TRUE(trim.state.isValid());
    EXPECT_TRUE(trim.controls.isValid());
}

TEST_F(StabilityAnalysisTest, FindTrimConditionConstrained) {
    ASSERT_TRUE(analyzer_->isConfigured());
    
    std::vector<std::string> fixed_variables = {"position", "attitude"};
    TrimCondition trim = analyzer_->findTrimConditionConstrained(initial_state_, fixed_variables);
    
    EXPECT_TRUE(trim.isValid());
    // Position and attitude should remain close to initial values (within optimization tolerance)
    EXPECT_LT((trim.state.position() - initial_state_.position()).magnitude(), 1.0);
}

TEST_F(StabilityAnalysisTest, FindMultipleTrimConditions) {
    ASSERT_TRUE(analyzer_->isConfigured());
    
    std::vector<KiteState> initial_guesses;
    
    // Create multiple initial guesses with different velocities
    for (double speed = 10.0; speed <= 20.0; speed += 5.0) {
        KiteState state = initial_state_;
        state.setVelocity(Vector3(speed, 0.0, 0.0));
        initial_guesses.push_back(state);
    }
    
    auto trim_conditions = analyzer_->findMultipleTrimConditions(initial_guesses);
    
    EXPECT_EQ(trim_conditions.size(), initial_guesses.size());
    
    for (const auto& trim : trim_conditions) {
        EXPECT_TRUE(trim.isValid());
    }
}

// Stability derivatives calculation tests
TEST_F(StabilityAnalysisTest, CalculateStabilityDerivatives) {
    ASSERT_TRUE(analyzer_->isConfigured());
    
    // First find a trim condition
    TrimCondition trim = analyzer_->findTrimCondition(initial_state_);
    ASSERT_TRUE(trim.isValid());
    
    StabilityDerivatives derivatives = analyzer_->calculateStabilityDerivatives(trim);
    
    EXPECT_TRUE(derivatives.isValid());
    
    // Check that key derivatives have reasonable values
    EXPECT_GT(derivatives.CL_alpha, 0.0);     // Lift should increase with angle of attack
    EXPECT_GT(derivatives.CD_alpha, 0.0);     // Drag should increase with angle of attack
    EXPECT_LT(derivatives.Cm_alpha, 0.0);     // Should be statically stable (negative Cm_alpha)
    EXPECT_GT(derivatives.CL_elevator, 0.0);  // Elevator should affect lift
    EXPECT_LT(derivatives.Cm_elevator, 0.0);  // Elevator should affect pitching moment
    
    // Rate derivatives should provide damping
    EXPECT_LT(derivatives.CL_q, 0.0);         // Pitch rate damping
    EXPECT_LT(derivatives.Cm_q, 0.0);         // Pitch damping
}

TEST_F(StabilityAnalysisTest, CalculateStabilityDerivativesWithPerturbation) {
    ASSERT_TRUE(analyzer_->isConfigured());
    
    TrimCondition trim = analyzer_->findTrimCondition(initial_state_);
    ASSERT_TRUE(trim.isValid());
    
    double perturbation_size = 1e-3;  // Smaller perturbation
    StabilityDerivatives derivatives = analyzer_->calculateStabilityDerivatives(trim, perturbation_size);
    
    EXPECT_TRUE(derivatives.isValid());
    
    // Results should be similar to default perturbation but may have different numerical accuracy
    EXPECT_GT(derivatives.CL_alpha, 0.0);
    EXPECT_GT(derivatives.CD_alpha, 0.0);
}

// Linearization tests
TEST_F(StabilityAnalysisTest, LinearizeAroundTrim) {
    ASSERT_TRUE(analyzer_->isConfigured());
    
    TrimCondition trim = analyzer_->findTrimCondition(initial_state_);
    ASSERT_TRUE(trim.isValid());
    
    LinearizedSystem system = analyzer_->linearizeAroundTrim(trim);
    
    EXPECT_TRUE(system.isValid());
    EXPECT_EQ(system.state_size, 13);
    EXPECT_EQ(system.control_size, 3);
    EXPECT_EQ(system.output_size, 13);
    
    // Check matrix dimensions
    EXPECT_EQ(system.A.rows(), 13);
    EXPECT_EQ(system.A.cols(), 13);
    EXPECT_EQ(system.B.rows(), 13);
    EXPECT_EQ(system.B.cols(), 3);
    EXPECT_EQ(system.C.rows(), 13);
    EXPECT_EQ(system.C.cols(), 13);
    EXPECT_EQ(system.D.rows(), 13);
    EXPECT_EQ(system.D.cols(), 3);
    
    // Trim point should match
    EXPECT_EQ(system.trim_state.time(), trim.state.time());
    EXPECT_NEAR(system.trim_controls.elevator, trim.controls.elevator, 1e-6);
}

// Flight envelope analysis tests
TEST_F(StabilityAnalysisTest, AnalyzeFlightEnvelope) {
    ASSERT_TRUE(analyzer_->isConfigured());
    
    // Use smaller ranges for faster testing
    double min_speed = 10.0, max_speed = 20.0, speed_step = 5.0;
    double min_alpha = -0.1, max_alpha = 0.2, alpha_step = 0.1;
    
    FlightEnvelope envelope = analyzer_->analyzeFlightEnvelope(
        min_speed, max_speed, speed_step, min_alpha, max_alpha, alpha_step);
    
    EXPECT_TRUE(envelope.isValid());
    EXPECT_GE(envelope.min_airspeed, 0.0);
    EXPECT_GT(envelope.max_airspeed, envelope.min_airspeed);
    EXPECT_LT(envelope.min_angle_of_attack, envelope.max_angle_of_attack);
    EXPECT_GE(envelope.stall_angle, envelope.min_angle_of_attack);
    EXPECT_LE(envelope.stall_angle, envelope.max_angle_of_attack);
    
    // Should have found some trim conditions
    EXPECT_GE(envelope.stable_trims.size() + envelope.unstable_trims.size(), 1);
    
    // Performance metrics should be reasonable
    EXPECT_GT(envelope.max_lift_to_drag, 1.0);
    EXPECT_GT(envelope.best_glide_speed, 0.0);
    EXPECT_GT(envelope.min_sink_speed, 0.0);
}

// Utility function tests
TEST_F(StabilityAnalysisTest, CalculateTrimQuality) {
    Forces zero_forces;
    zero_forces.force = Vector3(0.0, 0.0, 0.0);
    zero_forces.moment = Vector3(0.0, 0.0, 0.0);
    
    double quality = StabilityAnalyzer::calculateTrimQuality(zero_forces);
    EXPECT_NEAR(quality, 1.0, 1e-6);  // Perfect trim should have quality = 1
    
    Forces large_forces;
    large_forces.force = Vector3(100.0, 0.0, 0.0);
    large_forces.moment = Vector3(10.0, 0.0, 0.0);
    
    quality = StabilityAnalyzer::calculateTrimQuality(large_forces);
    EXPECT_LT(quality, 0.1);  // Large residuals should have low quality
}

TEST_F(StabilityAnalysisTest, IsStableEigenvalue) {
    std::complex<double> stable_eigenvalue(-1.0, 2.0);    // Negative real part
    std::complex<double> unstable_eigenvalue(0.5, -1.0);  // Positive real part
    std::complex<double> marginal_eigenvalue(0.0, 3.0);   // Zero real part
    
    EXPECT_TRUE(StabilityAnalyzer::isStableEigenvalue(stable_eigenvalue));
    EXPECT_FALSE(StabilityAnalyzer::isStableEigenvalue(unstable_eigenvalue));
    EXPECT_FALSE(StabilityAnalyzer::isStableEigenvalue(marginal_eigenvalue));
}

// Error handling tests
TEST_F(StabilityAnalysisTest, UnconfiguredAnalyzerErrors) {
    StabilityAnalyzer unconfigured_analyzer;
    
    EXPECT_THROW(unconfigured_analyzer.findTrimCondition(initial_state_), std::runtime_error);
    EXPECT_THROW(unconfigured_analyzer.findTrimConditionConstrained(initial_state_, {}), std::runtime_error);
    
    TrimCondition dummy_trim;
    dummy_trim.state = initial_state_;
    dummy_trim.controls = ControlInputs();
    dummy_trim.converged = true;
    
    EXPECT_THROW(unconfigured_analyzer.calculateStabilityDerivatives(dummy_trim), std::runtime_error);
    EXPECT_THROW(unconfigured_analyzer.linearizeAroundTrim(dummy_trim), std::runtime_error);
    EXPECT_THROW(unconfigured_analyzer.analyzeFlightEnvelope(), std::runtime_error);
}

TEST_F(StabilityAnalysisTest, InvalidTrimConditionErrors) {
    ASSERT_TRUE(analyzer_->isConfigured());
    
    TrimCondition invalid_trim;  // Default constructed is invalid
    
    EXPECT_THROW(analyzer_->calculateStabilityDerivatives(invalid_trim), std::invalid_argument);
    EXPECT_THROW(analyzer_->linearizeAroundTrim(invalid_trim), std::invalid_argument);
}

// Performance and accuracy tests
TEST_F(StabilityAnalysisTest, TrimAccuracyTest) {
    ASSERT_TRUE(analyzer_->isConfigured());
    
    // Set stricter trim parameters
    TrimParameters strict_params;
    strict_params.force_tolerance = 0.1;
    strict_params.moment_tolerance = 0.01;
    strict_params.max_iterations = 200;
    analyzer_->setTrimParameters(strict_params);
    
    TrimCondition trim = analyzer_->findTrimCondition(initial_state_);
    
    EXPECT_TRUE(trim.isValid());
    
    // If converged, should meet the strict tolerances
    if (trim.converged) {
        EXPECT_LE(trim.force_residual, strict_params.force_tolerance);
        EXPECT_LE(trim.moment_residual, strict_params.moment_tolerance);
        EXPECT_GT(trim.trim_quality, 0.9);  // High quality trim
    }
}

TEST_F(StabilityAnalysisTest, DerivativeConsistencyTest) {
    ASSERT_TRUE(analyzer_->isConfigured());
    
    TrimCondition trim = analyzer_->findTrimCondition(initial_state_);
    ASSERT_TRUE(trim.isValid());
    
    // Calculate derivatives with different perturbation sizes
    double perturbation1 = 1e-4;
    double perturbation2 = 1e-3;
    
    StabilityDerivatives derivatives1 = analyzer_->calculateStabilityDerivatives(trim, perturbation1);
    StabilityDerivatives derivatives2 = analyzer_->calculateStabilityDerivatives(trim, perturbation2);
    
    EXPECT_TRUE(derivatives1.isValid());
    EXPECT_TRUE(derivatives2.isValid());
    
    // Key derivatives should be consistent within reasonable tolerance
    double tolerance = 0.5;  // 50% tolerance for numerical derivatives
    EXPECT_NEAR(derivatives1.CL_alpha, derivatives2.CL_alpha, 
                std::abs(derivatives1.CL_alpha) * tolerance);
    EXPECT_NEAR(derivatives1.CD_alpha, derivatives2.CD_alpha, 
                std::abs(derivatives1.CD_alpha) * tolerance);
    EXPECT_NEAR(derivatives1.Cm_alpha, derivatives2.Cm_alpha, 
                std::abs(derivatives1.Cm_alpha) * tolerance);
}