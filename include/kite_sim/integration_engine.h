#pragma once

#include "math_types.h"
#include "kite_state.h"
#include <functional>
#include <memory>

namespace kite_sim {

/**
 * @brief Integration statistics for monitoring performance and accuracy
 */
struct IntegrationStats {
    size_t total_steps;           // Total number of integration steps taken
    size_t accepted_steps;        // Number of accepted steps
    size_t rejected_steps;        // Number of rejected steps due to error control
    double min_step_size;         // Minimum step size used
    double max_step_size;         // Maximum step size used
    double avg_step_size;         // Average step size
    double max_error;             // Maximum error encountered
    double total_time;            // Total integration time
    
    IntegrationStats() : total_steps(0), accepted_steps(0), rejected_steps(0),
                        min_step_size(0.0), max_step_size(0.0), avg_step_size(0.0),
                        max_error(0.0), total_time(0.0) {}
    
    void reset();
    double getStepAcceptanceRate() const;
};

/**
 * @brief Integration parameters for controlling adaptive stepping and error tolerance
 */
struct IntegrationParameters {
    double initial_step_size;     // Initial time step size (s)
    double min_step_size;         // Minimum allowed step size (s)
    double max_step_size;         // Maximum allowed step size (s)
    double absolute_tolerance;    // Absolute error tolerance
    double relative_tolerance;    // Relative error tolerance
    double safety_factor;         // Safety factor for step size adjustment (0.8-0.9)
    double step_increase_factor;  // Maximum factor for step size increase (1.5-2.0)
    double step_decrease_factor;  // Maximum factor for step size decrease (0.1-0.5)
    int max_iterations;           // Maximum iterations for adaptive stepping
    
    IntegrationParameters() 
        : initial_step_size(0.01)
        , min_step_size(1e-6)
        , max_step_size(0.1)
        , absolute_tolerance(1e-6)
        , relative_tolerance(1e-6)
        , safety_factor(0.85)
        , step_increase_factor(1.5)
        , step_decrease_factor(0.5)
        , max_iterations(100) {}
    
    bool isValid() const;
};

/**
 * @brief Derivative function type for integration
 * 
 * Function that calculates the derivative of the state vector at a given time and state.
 * Parameters: time, state_vector (input), derivative_vector (output)
 */
using DerivativeFunction = std::function<void(double, const double*, double*)>;

/**
 * @brief Integration method enumeration
 */
enum class IntegrationMethod {
    EULER,                    // Simple Euler method (1st order)
    RUNGE_KUTTA_4,           // Fixed-step 4th order Runge-Kutta
    RUNGE_KUTTA_45,          // Adaptive Runge-Kutta 4(5) with error control
    DORMAND_PRINCE_45        // Dormand-Prince 4(5) method (higher accuracy)
};

/**
 * @brief Advanced numerical integration engine with adaptive time stepping
 * 
 * Provides high-accuracy numerical integration with error control and adaptive
 * time stepping for kite simulation. Supports multiple integration methods
 * including fixed-step and adaptive algorithms.
 */
class IntegrationEngine {
public:
    // Constructor
    explicit IntegrationEngine(IntegrationMethod method = IntegrationMethod::RUNGE_KUTTA_45);
    
    // Copy constructor and assignment
    IntegrationEngine(const IntegrationEngine& other) = default;
    IntegrationEngine& operator=(const IntegrationEngine& other) = default;
    
    // Move constructor and assignment
    IntegrationEngine(IntegrationEngine&& other) noexcept = default;
    IntegrationEngine& operator=(IntegrationEngine&& other) noexcept = default;
    
    // Destructor
    ~IntegrationEngine() = default;
    
    // Configuration
    void setMethod(IntegrationMethod method) { method_ = method; }
    IntegrationMethod getMethod() const { return method_; }
    
    void setParameters(const IntegrationParameters& params);
    const IntegrationParameters& getParameters() const { return parameters_; }
    
    void setDerivativeFunction(const DerivativeFunction& func) { derivative_func_ = func; }
    
    // State vector integration
    bool integrate(double* state_vector, int state_size, 
                  double current_time, double target_time);
    
    bool integrateStep(double* state_vector, int state_size,
                      double current_time, double& step_size);
    
    // KiteState integration (convenience wrapper)
    bool integrate(KiteState& state, double target_time);
    bool integrateStep(KiteState& state, double& step_size);
    
    // Statistics and monitoring
    const IntegrationStats& getStats() const { return stats_; }
    void resetStats() { stats_.reset(); }
    
    // Error estimation
    double estimateError(const double* state1, const double* state2, 
                        int state_size) const;
    
    // Step size control
    double calculateOptimalStepSize(double current_step, double error, 
                                   int method_order) const;
    
    // Validation
    bool isConfigured() const;
    bool validateState(const double* state_vector, int state_size) const;
    
    // Utility
    void reset();
    
private:
    // Integration method
    IntegrationMethod method_;
    
    // Integration parameters
    IntegrationParameters parameters_;
    
    // Derivative function
    DerivativeFunction derivative_func_;
    
    // Statistics
    mutable IntegrationStats stats_;
    
    // Working arrays for integration
    mutable std::vector<double> k1_, k2_, k3_, k4_, k5_, k6_;
    mutable std::vector<double> temp_state_, error_estimate_;
    
    // Fixed-step integration methods
    void integrateEuler(double* state_vector, int state_size, 
                       double time, double step_size) const;
    
    void integrateRungeKutta4(double* state_vector, int state_size,
                             double time, double step_size) const;
    
    // Adaptive integration methods
    bool integrateRungeKutta45(double* state_vector, int state_size,
                              double time, double& step_size) const;
    
    bool integrateDormandPrince45(double* state_vector, int state_size,
                                 double time, double& step_size) const;
    
    // Butcher tableau coefficients for RK methods
    static const double RK45_A[6][5];
    static const double RK45_B[6];
    static const double RK45_B_STAR[6];
    static const double RK45_C[6];
    
    static const double DP45_A[6][5];
    static const double DP45_B[6];
    static const double DP45_B_STAR[6];
    static const double DP45_C[6];
    
    // Helper functions
    void ensureWorkingArrays(int state_size) const;
    void updateStats(double step_size, double error, bool accepted) const;
    bool checkStepSizeConstraints(double& step_size) const;
    
    // Error control
    double calculateErrorNorm(const double* error_vector, const double* state_vector,
                             int state_size) const;
    
    // State vector utilities for KiteState conversion
    void kiteStateToVector(const KiteState& state, double* vector) const;
    void vectorToKiteState(const double* vector, KiteState& state) const;
};

} // namespace kite_sim