#pragma once

#include "math_types.h"
#include "kite_state.h"
#include "kite_geometry.h"
#include "aerodynamics.h"
#include "dynamics.h"
#include "wind_model.h"
#include "tether.h"
#include <vector>
#include <functional>
#include <memory>

namespace kite_sim {

/**
 * @brief Trim condition parameters for equilibrium flight
 */
struct TrimCondition {
    KiteState state;                    // Equilibrium state
    ControlInputs controls;             // Control inputs for trim
    Forces total_forces;                // Forces at trim (should be near zero)
    
    // Trim quality metrics
    double force_residual = 0.0;        // Magnitude of residual forces (N)
    double moment_residual = 0.0;       // Magnitude of residual moments (N⋅m)
    double trim_quality = 0.0;          // Overall trim quality (0-1, 1 = perfect)
    
    // Convergence information
    int iterations = 0;                 // Number of iterations to converge
    bool converged = false;             // Whether trim calculation converged
    
    TrimCondition() = default;
    
    bool isValid() const;
    bool isTrimmed(double force_tolerance = 1.0, double moment_tolerance = 0.1) const;
    void reset();
    
    std::string toString() const;
};

/**
 * @brief Trim calculation parameters
 */
struct TrimParameters {
    // Convergence criteria
    double force_tolerance = 1.0;       // Force residual tolerance (N)
    double moment_tolerance = 0.1;      // Moment residual tolerance (N⋅m)
    int max_iterations = 100;           // Maximum iterations
    
    // Optimization parameters
    double step_size = 0.01;            // Initial step size for optimization
    double step_reduction = 0.5;        // Step size reduction factor
    double min_step_size = 1e-6;        // Minimum step size
    
    // Variable bounds
    double max_control_deflection = M_PI / 6;  // Maximum control deflection (30 deg)
    double max_attitude_change = M_PI / 4;     // Maximum attitude change (45 deg)
    double max_velocity_change = 20.0;         // Maximum velocity change (m/s)
    
    TrimParameters() = default;
    
    bool isValid() const;
    void reset();
};

/**
 * @brief Stability derivatives for linearized analysis
 */
struct StabilityDerivatives {
    // Force derivatives (per unit change)
    double CL_alpha = 0.0;      // ∂CL/∂α (per radian)
    double CL_beta = 0.0;       // ∂CL/∂β (per radian)
    double CL_p = 0.0;          // ∂CL/∂p (per radian/s)
    double CL_q = 0.0;          // ∂CL/∂q (per radian/s)
    double CL_r = 0.0;          // ∂CL/∂r (per radian/s)
    
    double CD_alpha = 0.0;      // ∂CD/∂α (per radian)
    double CD_beta = 0.0;       // ∂CD/∂β (per radian)
    
    double CY_beta = 0.0;       // ∂CY/∂β (per radian)
    double CY_p = 0.0;          // ∂CY/∂p (per radian/s)
    double CY_r = 0.0;          // ∂CY/∂r (per radian/s)
    
    // Moment derivatives
    double Cl_beta = 0.0;       // ∂Cl/∂β (per radian)
    double Cl_p = 0.0;          // ∂Cl/∂p (per radian/s)
    double Cl_r = 0.0;          // ∂Cl/∂r (per radian/s)
    
    double Cm_alpha = 0.0;      // ∂Cm/∂α (per radian)
    double Cm_q = 0.0;          // ∂Cm/∂q (per radian/s)
    
    double Cn_beta = 0.0;       // ∂Cn/∂β (per radian)
    double Cn_p = 0.0;          // ∂Cn/∂p (per radian/s)
    double Cn_r = 0.0;          // ∂Cn/∂r (per radian/s)
    
    // Control derivatives
    double CL_elevator = 0.0;   // ∂CL/∂δe (per radian)
    double CD_elevator = 0.0;   // ∂CD/∂δe (per radian)
    double Cm_elevator = 0.0;   // ∂Cm/∂δe (per radian)
    
    double CY_rudder = 0.0;     // ∂CY/∂δr (per radian)
    double Cn_rudder = 0.0;     // ∂Cn/∂δr (per radian)
    
    double Cl_aileron = 0.0;    // ∂Cl/∂δa (per radian)
    
    StabilityDerivatives() = default;
    
    bool isValid() const;
    void reset();
    
    std::string toString() const;
};

/**
 * @brief Linearized system matrices for stability analysis
 */
struct LinearizedSystem {
    Matrix A;                   // State matrix (n x n)
    Matrix B;                   // Control matrix (n x m)
    Matrix C;                   // Output matrix (p x n)
    Matrix D;                   // Feedthrough matrix (p x m)
    
    // System dimensions
    int state_size = 0;         // Number of state variables
    int control_size = 0;       // Number of control inputs
    int output_size = 0;        // Number of outputs
    
    // Linearization point
    KiteState trim_state;       // State around which system is linearized
    ControlInputs trim_controls; // Control inputs at linearization point
    
    LinearizedSystem() = default;
    
    bool isValid() const;
    void reset();
    
    // Eigenvalue analysis
    std::vector<std::complex<double>> getEigenvalues() const;
    bool isStable() const;
    
    std::string toString() const;
};

/**
 * @brief Flight envelope analysis results
 */
struct FlightEnvelope {
    // Operating limits
    double min_airspeed = 0.0;          // Minimum airspeed (m/s)
    double max_airspeed = 0.0;          // Maximum airspeed (m/s)
    double min_angle_of_attack = 0.0;   // Minimum AoA (rad)
    double max_angle_of_attack = 0.0;   // Maximum AoA (rad)
    double stall_angle = 0.0;           // Stall angle of attack (rad)
    
    // Stability boundaries
    std::vector<TrimCondition> stable_trims;    // Stable trim conditions
    std::vector<TrimCondition> unstable_trims;  // Unstable trim conditions
    
    // Performance metrics
    double max_lift_to_drag = 0.0;      // Maximum L/D ratio
    double best_glide_speed = 0.0;      // Speed for best glide ratio (m/s)
    double min_sink_speed = 0.0;        // Speed for minimum sink rate (m/s)
    
    FlightEnvelope() = default;
    
    bool isValid() const;
    void reset();
    
    std::string toString() const;
};

/**
 * @brief Stability and flight envelope analysis engine
 * 
 * Provides comprehensive stability analysis capabilities including:
 * - Trim condition calculation for equilibrium flight states
 * - Linearization around trim conditions for small perturbation analysis
 * - Stability derivative calculation using finite differences
 * - Flight envelope analysis to identify stable operating regions
 */
class StabilityAnalyzer {
public:
    // Constructor
    StabilityAnalyzer();
    
    // Configuration
    void setKiteGeometry(const KiteGeometry& geometry);
    void setAerodynamicsCalculator(std::shared_ptr<AerodynamicsCalculator> aero_calc);
    void setDynamicsEngine(std::shared_ptr<DynamicsEngine> dynamics_engine);
    void setTetherModel(std::shared_ptr<TetherModel> tether_model);
    void setWindModel(std::shared_ptr<WindModel> wind_model);
    
    void setTrimParameters(const TrimParameters& params) { trim_params_ = params; }
    const TrimParameters& getTrimParameters() const { return trim_params_; }
    
    // Trim condition calculation
    TrimCondition findTrimCondition(const KiteState& initial_guess,
                                   const ControlInputs& initial_controls = ControlInputs()) const;
    
    TrimCondition findTrimConditionConstrained(const KiteState& initial_guess,
                                              const std::vector<std::string>& fixed_variables,
                                              const ControlInputs& initial_controls = ControlInputs()) const;
    
    // Multiple trim conditions
    std::vector<TrimCondition> findMultipleTrimConditions(const std::vector<KiteState>& initial_guesses) const;
    
    // Stability derivative calculation
    StabilityDerivatives calculateStabilityDerivatives(const TrimCondition& trim_condition,
                                                      double perturbation_size = 1e-4) const;
    
    // Linearization
    LinearizedSystem linearizeAroundTrim(const TrimCondition& trim_condition,
                                        double perturbation_size = 1e-4) const;
    
    // Flight envelope analysis
    FlightEnvelope analyzeFlightEnvelope(double min_speed = 5.0, double max_speed = 50.0,
                                        double speed_step = 1.0,
                                        double min_alpha = -0.2, double max_alpha = 0.4,
                                        double alpha_step = 0.02) const;
    
    // Validation and configuration
    bool isConfigured() const;
    std::vector<std::string> getConfigurationErrors() const;
    
    // Utility functions
    static double calculateTrimQuality(const Forces& residual_forces);
    static bool isStableEigenvalue(const std::complex<double>& eigenvalue);
    
private:
    // Configuration
    KiteGeometry geometry_;
    std::shared_ptr<AerodynamicsCalculator> aero_calc_;
    std::shared_ptr<DynamicsEngine> dynamics_engine_;
    std::shared_ptr<TetherModel> tether_model_;
    std::shared_ptr<WindModel> wind_model_;
    
    TrimParameters trim_params_;
    
    bool geometry_set_;
    bool modules_configured_;
    
    // Internal calculation methods
    Forces calculateTotalForces(const KiteState& state, const ControlInputs& controls, double time = 0.0) const;
    
    // Trim calculation helpers
    TrimCondition optimizeTrimCondition(const KiteState& initial_state,
                                       const ControlInputs& initial_controls,
                                       const std::vector<std::string>& fixed_variables) const;
    
    std::vector<double> stateToOptimizationVector(const KiteState& state, const ControlInputs& controls,
                                                 const std::vector<std::string>& fixed_variables) const;
    
    std::pair<KiteState, ControlInputs> optimizationVectorToState(const std::vector<double>& x,
                                                                 const KiteState& reference_state,
                                                                 const ControlInputs& reference_controls,
                                                                 const std::vector<std::string>& fixed_variables) const;
    
    double evaluateTrimObjective(const std::vector<double>& x,
                                const KiteState& reference_state,
                                const ControlInputs& reference_controls,
                                const std::vector<std::string>& fixed_variables) const;
    
    // Finite difference calculations
    StabilityDerivatives calculateDerivativesFiniteDifference(const TrimCondition& trim,
                                                             double perturbation_size) const;
    
    AeroForces perturbState(const KiteState& state, const ControlInputs& controls,
                           const std::string& variable, double delta) const;
    
    // Linearization helpers
    Matrix calculateStateMatrix(const TrimCondition& trim, double perturbation_size) const;
    Matrix calculateControlMatrix(const TrimCondition& trim, double perturbation_size) const;
    
    // Flight envelope helpers
    std::vector<TrimCondition> scanFlightEnvelope(double min_speed, double max_speed, double speed_step,
                                                 double min_alpha, double max_alpha, double alpha_step) const;
    
    bool isStableTrimCondition(const TrimCondition& trim) const;
    
    // Validation helpers
    bool validateConfiguration() const;
    bool validateTrimCondition(const TrimCondition& trim) const;
    
    // Optimization constants
    static constexpr double DEFAULT_PERTURBATION = 1e-4;
    static constexpr double TRIM_FORCE_TOLERANCE = 1.0;     // N
    static constexpr double TRIM_MOMENT_TOLERANCE = 0.1;    // N⋅m
    static constexpr int MAX_TRIM_ITERATIONS = 100;
};

} // namespace kite_sim