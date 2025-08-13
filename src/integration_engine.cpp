#include "kite_sim/integration_engine.h"
#include <cmath>
#include <algorithm>
#include <stdexcept>
#include <limits>

namespace kite_sim {

// IntegrationStats implementation
void IntegrationStats::reset() {
    total_steps = 0;
    accepted_steps = 0;
    rejected_steps = 0;
    min_step_size = std::numeric_limits<double>::max();
    max_step_size = 0.0;
    avg_step_size = 0.0;
    max_error = 0.0;
    total_time = 0.0;
}

double IntegrationStats::getStepAcceptanceRate() const {
    return total_steps > 0 ? static_cast<double>(accepted_steps) / total_steps : 0.0;
}

// IntegrationParameters implementation
bool IntegrationParameters::isValid() const {
    return initial_step_size > 0.0 &&
           min_step_size > 0.0 &&
           max_step_size > min_step_size &&
           initial_step_size >= min_step_size &&
           initial_step_size <= max_step_size &&
           absolute_tolerance > 0.0 &&
           relative_tolerance > 0.0 &&
           safety_factor > 0.0 && safety_factor < 1.0 &&
           step_increase_factor > 1.0 &&
           step_decrease_factor > 0.0 && step_decrease_factor < 1.0 &&
           max_iterations > 0;
}

// Butcher tableau coefficients for Runge-Kutta 4(5) method
const double IntegrationEngine::RK45_A[6][5] = {
    {0.0, 0.0, 0.0, 0.0, 0.0},
    {1.0/4.0, 0.0, 0.0, 0.0, 0.0},
    {3.0/32.0, 9.0/32.0, 0.0, 0.0, 0.0},
    {1932.0/2197.0, -7200.0/2197.0, 7296.0/2197.0, 0.0, 0.0},
    {439.0/216.0, -8.0, 3680.0/513.0, -845.0/4104.0, 0.0},
    {-8.0/27.0, 2.0, -3544.0/2565.0, 1859.0/4104.0, -11.0/40.0}
};

const double IntegrationEngine::RK45_B[6] = {
    16.0/135.0, 0.0, 6656.0/12825.0, 28561.0/56430.0, -9.0/50.0, 2.0/55.0
};

const double IntegrationEngine::RK45_B_STAR[6] = {
    25.0/216.0, 0.0, 1408.0/2565.0, 2197.0/4104.0, -1.0/5.0, 0.0
};

const double IntegrationEngine::RK45_C[6] = {
    0.0, 1.0/4.0, 3.0/8.0, 12.0/13.0, 1.0, 1.0/2.0
};

// Dormand-Prince 4(5) coefficients (higher accuracy)
const double IntegrationEngine::DP45_A[6][5] = {
    {0.0, 0.0, 0.0, 0.0, 0.0},
    {1.0/5.0, 0.0, 0.0, 0.0, 0.0},
    {3.0/40.0, 9.0/40.0, 0.0, 0.0, 0.0},
    {44.0/45.0, -56.0/15.0, 32.0/9.0, 0.0, 0.0},
    {19372.0/6561.0, -25360.0/2187.0, 64448.0/6561.0, -212.0/729.0, 0.0},
    {9017.0/3168.0, -355.0/33.0, 46732.0/5247.0, 49.0/176.0, -5103.0/18656.0}
};

const double IntegrationEngine::DP45_B[6] = {
    35.0/384.0, 0.0, 500.0/1113.0, 125.0/192.0, -2187.0/6784.0, 11.0/84.0
};

const double IntegrationEngine::DP45_B_STAR[6] = {
    5179.0/57600.0, 0.0, 7571.0/16695.0, 393.0/640.0, -92097.0/339200.0, 187.0/2100.0, 1.0/40.0
};

const double IntegrationEngine::DP45_C[6] = {
    0.0, 1.0/5.0, 3.0/10.0, 4.0/5.0, 8.0/9.0, 1.0
};

// IntegrationEngine implementation
IntegrationEngine::IntegrationEngine(IntegrationMethod method)
    : method_(method)
    , parameters_()
    , derivative_func_(nullptr)
    , stats_() {
}

void IntegrationEngine::setParameters(const IntegrationParameters& params) {
    if (!params.isValid()) {
        throw std::invalid_argument("Invalid integration parameters");
    }
    parameters_ = params;
}

bool IntegrationEngine::integrate(double* state_vector, int state_size,
                                 double current_time, double target_time) {
    if (!isConfigured()) {
        throw std::runtime_error("IntegrationEngine not properly configured");
    }
    
    if (!state_vector || state_size <= 0) {
        throw std::invalid_argument("Invalid state vector or size");
    }
    
    if (target_time <= current_time) {
        return true; // No integration needed
    }
    
    if (!validateState(state_vector, state_size)) {
        throw std::invalid_argument("Invalid initial state");
    }
    
    double time = current_time;
    double step_size = parameters_.initial_step_size;
    
    // Ensure working arrays are properly sized
    ensureWorkingArrays(state_size);
    
    while (time < target_time) {
        // Adjust step size if we would overshoot
        if (time + step_size > target_time) {
            step_size = target_time - time;
        }
        
        bool success = false;
        int iterations = 0;
        
        // Adaptive stepping loop
        while (!success && iterations < parameters_.max_iterations) {
            success = integrateStep(state_vector, state_size, time, step_size);
            iterations++;
            
            if (!success) {
                // Step was rejected, step_size has been reduced
                continue;
            }
        }
        
        if (!success) {
            return false; // Failed to converge
        }
        
        time += step_size;
        
        // Update statistics
        stats_.total_time = time - current_time;
    }
    
    return true;
}

bool IntegrationEngine::integrateStep(double* state_vector, int state_size,
                                     double current_time, double& step_size) {
    if (!checkStepSizeConstraints(step_size)) {
        return false;
    }
    
    switch (method_) {
        case IntegrationMethod::EULER:
            integrateEuler(state_vector, state_size, current_time, step_size);
            updateStats(step_size, 0.0, true); // Euler always accepts
            return true;
            
        case IntegrationMethod::RUNGE_KUTTA_4:
            integrateRungeKutta4(state_vector, state_size, current_time, step_size);
            updateStats(step_size, 0.0, true); // Fixed step always accepts
            return true;
            
        case IntegrationMethod::RUNGE_KUTTA_45:
            return integrateRungeKutta45(state_vector, state_size, current_time, step_size);
            
        case IntegrationMethod::DORMAND_PRINCE_45:
            return integrateDormandPrince45(state_vector, state_size, current_time, step_size);
            
        default:
            throw std::runtime_error("Unknown integration method");
    }
}

bool IntegrationEngine::integrate(KiteState& state, double target_time) {
    constexpr int state_size = KiteState::getStateSize();
    std::vector<double> state_vector(state_size);
    
    kiteStateToVector(state, state_vector.data());
    
    bool success = integrate(state_vector.data(), state_size, state.time(), target_time);
    
    if (success) {
        vectorToKiteState(state_vector.data(), state);
        state.setTime(target_time);
    }
    
    return success;
}

bool IntegrationEngine::integrateStep(KiteState& state, double& step_size) {
    constexpr int state_size = KiteState::getStateSize();
    std::vector<double> state_vector(state_size);
    
    kiteStateToVector(state, state_vector.data());
    
    double current_time = state.time();
    bool success = integrateStep(state_vector.data(), state_size, current_time, step_size);
    
    if (success) {
        vectorToKiteState(state_vector.data(), state);
        state.setTime(current_time + step_size);
    }
    
    return success;
}

void IntegrationEngine::integrateEuler(double* state_vector, int state_size,
                                      double time, double step_size) const {
    // Simple Euler: y_{n+1} = y_n + h * f(t_n, y_n)
    
    derivative_func_(time, state_vector, k1_.data());
    
    for (int i = 0; i < state_size; ++i) {
        state_vector[i] += step_size * k1_[i];
    }
}

void IntegrationEngine::integrateRungeKutta4(double* state_vector, int state_size,
                                            double time, double step_size) const {
    // Classical 4th order Runge-Kutta
    
    // k1 = f(t, y)
    derivative_func_(time, state_vector, k1_.data());
    
    // k2 = f(t + h/2, y + h*k1/2)
    for (int i = 0; i < state_size; ++i) {
        temp_state_[i] = state_vector[i] + 0.5 * step_size * k1_[i];
    }
    derivative_func_(time + 0.5 * step_size, temp_state_.data(), k2_.data());
    
    // k3 = f(t + h/2, y + h*k2/2)
    for (int i = 0; i < state_size; ++i) {
        temp_state_[i] = state_vector[i] + 0.5 * step_size * k2_[i];
    }
    derivative_func_(time + 0.5 * step_size, temp_state_.data(), k3_.data());
    
    // k4 = f(t + h, y + h*k3)
    for (int i = 0; i < state_size; ++i) {
        temp_state_[i] = state_vector[i] + step_size * k3_[i];
    }
    derivative_func_(time + step_size, temp_state_.data(), k4_.data());
    
    // y_{n+1} = y_n + h/6 * (k1 + 2*k2 + 2*k3 + k4)
    for (int i = 0; i < state_size; ++i) {
        state_vector[i] += step_size / 6.0 * (k1_[i] + 2.0 * k2_[i] + 2.0 * k3_[i] + k4_[i]);
    }
}

bool IntegrationEngine::integrateRungeKutta45(double* state_vector, int state_size,
                                             double time, double& step_size) const {
    // Runge-Kutta 4(5) with error control
    
    // Store original state
    std::copy(state_vector, state_vector + state_size, temp_state_.data());
    
    // Calculate k1
    derivative_func_(time, state_vector, k1_.data());
    
    // Calculate k2
    for (int i = 0; i < state_size; ++i) {
        temp_state_[i] = state_vector[i] + step_size * RK45_A[1][0] * k1_[i];
    }
    derivative_func_(time + RK45_C[1] * step_size, temp_state_.data(), k2_.data());
    
    // Calculate k3
    for (int i = 0; i < state_size; ++i) {
        temp_state_[i] = state_vector[i] + step_size * (RK45_A[2][0] * k1_[i] + RK45_A[2][1] * k2_[i]);
    }
    derivative_func_(time + RK45_C[2] * step_size, temp_state_.data(), k3_.data());
    
    // Calculate k4
    for (int i = 0; i < state_size; ++i) {
        temp_state_[i] = state_vector[i] + step_size * (RK45_A[3][0] * k1_[i] + 
                                                       RK45_A[3][1] * k2_[i] + 
                                                       RK45_A[3][2] * k3_[i]);
    }
    derivative_func_(time + RK45_C[3] * step_size, temp_state_.data(), k4_.data());
    
    // Calculate k5
    for (int i = 0; i < state_size; ++i) {
        temp_state_[i] = state_vector[i] + step_size * (RK45_A[4][0] * k1_[i] + 
                                                       RK45_A[4][1] * k2_[i] + 
                                                       RK45_A[4][2] * k3_[i] + 
                                                       RK45_A[4][3] * k4_[i]);
    }
    derivative_func_(time + RK45_C[4] * step_size, temp_state_.data(), k5_.data());
    
    // Calculate k6
    for (int i = 0; i < state_size; ++i) {
        temp_state_[i] = state_vector[i] + step_size * (RK45_A[5][0] * k1_[i] + 
                                                       RK45_A[5][1] * k2_[i] + 
                                                       RK45_A[5][2] * k3_[i] + 
                                                       RK45_A[5][3] * k4_[i] + 
                                                       RK45_A[5][4] * k5_[i]);
    }
    derivative_func_(time + RK45_C[5] * step_size, temp_state_.data(), k6_.data());
    
    // Calculate 4th order solution
    for (int i = 0; i < state_size; ++i) {
        temp_state_[i] = state_vector[i] + step_size * (RK45_B_STAR[0] * k1_[i] + 
                                                       RK45_B_STAR[1] * k2_[i] + 
                                                       RK45_B_STAR[2] * k3_[i] + 
                                                       RK45_B_STAR[3] * k4_[i] + 
                                                       RK45_B_STAR[4] * k5_[i] + 
                                                       RK45_B_STAR[5] * k6_[i]);
    }
    
    // Calculate 5th order solution
    for (int i = 0; i < state_size; ++i) {
        error_estimate_[i] = state_vector[i] + step_size * (RK45_B[0] * k1_[i] + 
                                                           RK45_B[1] * k2_[i] + 
                                                           RK45_B[2] * k3_[i] + 
                                                           RK45_B[3] * k4_[i] + 
                                                           RK45_B[4] * k5_[i] + 
                                                           RK45_B[5] * k6_[i]);
    }
    
    // Calculate error estimate
    double error = estimateError(temp_state_.data(), error_estimate_.data(), state_size);
    
    // Check if step is acceptable
    double tolerance = std::max(parameters_.absolute_tolerance, 
                               parameters_.relative_tolerance * 
                               calculateErrorNorm(state_vector, state_vector, state_size));
    
    if (error <= tolerance) {
        // Accept step - use 5th order solution
        std::copy(error_estimate_.data(), error_estimate_.data() + state_size, state_vector);
        
        // Calculate optimal step size for next step
        double new_step = calculateOptimalStepSize(step_size, error, 5);
        step_size = std::min(new_step, parameters_.step_increase_factor * step_size);
        
        updateStats(step_size, error, true);
        return true;
    } else {
        // Reject step
        double new_step = calculateOptimalStepSize(step_size, error, 4);
        step_size = std::max(new_step, parameters_.step_decrease_factor * step_size);
        
        updateStats(step_size, error, false);
        return false;
    }
}

bool IntegrationEngine::integrateDormandPrince45(double* state_vector, int state_size,
                                                 double time, double& step_size) const {
    // Similar implementation to RK45 but with Dormand-Prince coefficients
    // This provides better error estimation and stability
    
    // Store original state
    std::copy(state_vector, state_vector + state_size, temp_state_.data());
    
    // Calculate stages k1 through k6 using DP45 coefficients
    derivative_func_(time, state_vector, k1_.data());
    
    for (int i = 0; i < state_size; ++i) {
        temp_state_[i] = state_vector[i] + step_size * DP45_A[1][0] * k1_[i];
    }
    derivative_func_(time + DP45_C[1] * step_size, temp_state_.data(), k2_.data());
    
    for (int i = 0; i < state_size; ++i) {
        temp_state_[i] = state_vector[i] + step_size * (DP45_A[2][0] * k1_[i] + DP45_A[2][1] * k2_[i]);
    }
    derivative_func_(time + DP45_C[2] * step_size, temp_state_.data(), k3_.data());
    
    for (int i = 0; i < state_size; ++i) {
        temp_state_[i] = state_vector[i] + step_size * (DP45_A[3][0] * k1_[i] + 
                                                       DP45_A[3][1] * k2_[i] + 
                                                       DP45_A[3][2] * k3_[i]);
    }
    derivative_func_(time + DP45_C[3] * step_size, temp_state_.data(), k4_.data());
    
    for (int i = 0; i < state_size; ++i) {
        temp_state_[i] = state_vector[i] + step_size * (DP45_A[4][0] * k1_[i] + 
                                                       DP45_A[4][1] * k2_[i] + 
                                                       DP45_A[4][2] * k3_[i] + 
                                                       DP45_A[4][3] * k4_[i]);
    }
    derivative_func_(time + DP45_C[4] * step_size, temp_state_.data(), k5_.data());
    
    for (int i = 0; i < state_size; ++i) {
        temp_state_[i] = state_vector[i] + step_size * (DP45_A[5][0] * k1_[i] + 
                                                       DP45_A[5][1] * k2_[i] + 
                                                       DP45_A[5][2] * k3_[i] + 
                                                       DP45_A[5][3] * k4_[i] + 
                                                       DP45_A[5][4] * k5_[i]);
    }
    derivative_func_(time + DP45_C[5] * step_size, temp_state_.data(), k6_.data());
    
    // Calculate 5th order solution
    for (int i = 0; i < state_size; ++i) {
        error_estimate_[i] = state_vector[i] + step_size * (DP45_B[0] * k1_[i] + 
                                                           DP45_B[1] * k2_[i] + 
                                                           DP45_B[2] * k3_[i] + 
                                                           DP45_B[3] * k4_[i] + 
                                                           DP45_B[4] * k5_[i] + 
                                                           DP45_B[5] * k6_[i]);
    }
    
    // Calculate error estimate (difference between 4th and 5th order)
    for (int i = 0; i < state_size; ++i) {
        temp_state_[i] = step_size * ((DP45_B[0] - DP45_B_STAR[0]) * k1_[i] + 
                                     (DP45_B[1] - DP45_B_STAR[1]) * k2_[i] + 
                                     (DP45_B[2] - DP45_B_STAR[2]) * k3_[i] + 
                                     (DP45_B[3] - DP45_B_STAR[3]) * k4_[i] + 
                                     (DP45_B[4] - DP45_B_STAR[4]) * k5_[i] + 
                                     (DP45_B[5] - DP45_B_STAR[5]) * k6_[i]);
    }
    
    double error = calculateErrorNorm(temp_state_.data(), error_estimate_.data(), state_size);
    
    // Check if step is acceptable
    double tolerance = std::max(parameters_.absolute_tolerance, 
                               parameters_.relative_tolerance * 
                               calculateErrorNorm(state_vector, state_vector, state_size));
    
    if (error <= tolerance) {
        // Accept step
        std::copy(error_estimate_.data(), error_estimate_.data() + state_size, state_vector);
        
        double new_step = calculateOptimalStepSize(step_size, error, 5);
        step_size = std::min(new_step, parameters_.step_increase_factor * step_size);
        
        updateStats(step_size, error, true);
        return true;
    } else {
        // Reject step
        double new_step = calculateOptimalStepSize(step_size, error, 4);
        step_size = std::max(new_step, parameters_.step_decrease_factor * step_size);
        
        updateStats(step_size, error, false);
        return false;
    }
}

double IntegrationEngine::estimateError(const double* state1, const double* state2, 
                                       int state_size) const {
    return calculateErrorNorm(state1, state2, state_size);
}

double IntegrationEngine::calculateOptimalStepSize(double current_step, double error, 
                                                  int method_order) const {
    if (error <= 0.0) {
        return current_step * parameters_.step_increase_factor;
    }
    
    double tolerance = std::max(parameters_.absolute_tolerance, 
                               parameters_.relative_tolerance);
    
    double factor = parameters_.safety_factor * std::pow(tolerance / error, 1.0 / method_order);
    
    return current_step * std::max(parameters_.step_decrease_factor, 
                                  std::min(parameters_.step_increase_factor, factor));
}

bool IntegrationEngine::isConfigured() const {
    return derivative_func_ != nullptr && parameters_.isValid();
}

bool IntegrationEngine::validateState(const double* state_vector, int state_size) const {
    if (!state_vector || state_size <= 0) {
        return false;
    }
    
    for (int i = 0; i < state_size; ++i) {
        if (!std::isfinite(state_vector[i])) {
            return false;
        }
    }
    
    return true;
}

void IntegrationEngine::reset() {
    stats_.reset();
    k1_.clear();
    k2_.clear();
    k3_.clear();
    k4_.clear();
    k5_.clear();
    k6_.clear();
    temp_state_.clear();
    error_estimate_.clear();
}

void IntegrationEngine::ensureWorkingArrays(int state_size) const {
    if (k1_.size() != static_cast<size_t>(state_size)) {
        k1_.resize(state_size);
        k2_.resize(state_size);
        k3_.resize(state_size);
        k4_.resize(state_size);
        k5_.resize(state_size);
        k6_.resize(state_size);
        temp_state_.resize(state_size);
        error_estimate_.resize(state_size);
    }
}

void IntegrationEngine::updateStats(double step_size, double error, bool accepted) const {
    stats_.total_steps++;
    
    if (accepted) {
        stats_.accepted_steps++;
    } else {
        stats_.rejected_steps++;
    }
    
    stats_.min_step_size = std::min(stats_.min_step_size, step_size);
    stats_.max_step_size = std::max(stats_.max_step_size, step_size);
    stats_.max_error = std::max(stats_.max_error, error);
    
    // Update average step size
    if (stats_.accepted_steps > 0) {
        stats_.avg_step_size = (stats_.avg_step_size * (stats_.accepted_steps - 1) + step_size) / 
                              stats_.accepted_steps;
    }
}

bool IntegrationEngine::checkStepSizeConstraints(double& step_size) const {
    if (step_size < parameters_.min_step_size) {
        step_size = parameters_.min_step_size;
        return step_size > 0.0;
    }
    
    if (step_size > parameters_.max_step_size) {
        step_size = parameters_.max_step_size;
    }
    
    return std::isfinite(step_size) && step_size > 0.0;
}

double IntegrationEngine::calculateErrorNorm(const double* error_vector, 
                                            const double* state_vector,
                                            int state_size) const {
    double norm = 0.0;
    
    for (int i = 0; i < state_size; ++i) {
        double scale = std::max(parameters_.absolute_tolerance,
                               parameters_.relative_tolerance * std::abs(state_vector[i]));
        double normalized_error = std::abs(error_vector[i]) / scale;
        norm += normalized_error * normalized_error;
    }
    
    return std::sqrt(norm / state_size);
}

void IntegrationEngine::kiteStateToVector(const KiteState& state, double* vector) const {
    state.toStateVector(vector);
}

void IntegrationEngine::vectorToKiteState(const double* vector, KiteState& state) const {
    state.fromStateVector(vector);
}

} // namespace kite_sim