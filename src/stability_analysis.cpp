#include "kite_sim/stability_analysis.h"
#include <cmath>
#include <algorithm>
#include <numeric>
#include <sstream>
#include <stdexcept>

namespace kite_sim {

// TrimCondition implementation
bool TrimCondition::isValid() const {
    return state.isValid() && controls.isValid() && total_forces.isValid() &&
           std::isfinite(force_residual) && std::isfinite(moment_residual) &&
           std::isfinite(trim_quality) && force_residual >= 0.0 && moment_residual >= 0.0 &&
           trim_quality >= 0.0 && trim_quality <= 1.0;
}

bool TrimCondition::isTrimmed(double force_tolerance, double moment_tolerance) const {
    return converged && force_residual <= force_tolerance && moment_residual <= moment_tolerance;
}

void TrimCondition::reset() {
    state = KiteState();
    controls = ControlInputs();
    total_forces = Forces();
    force_residual = 0.0;
    moment_residual = 0.0;
    trim_quality = 0.0;
    iterations = 0;
    converged = false;
}

std::string TrimCondition::toString() const {
    std::ostringstream oss;
    oss << "Trim Condition:\n";
    oss << "  Converged: " << (converged ? "Yes" : "No") << "\n";
    oss << "  Iterations: " << iterations << "\n";
    oss << "  Force Residual: " << force_residual << " N\n";
    oss << "  Moment Residual: " << moment_residual << " N⋅m\n";
    oss << "  Trim Quality: " << trim_quality << "\n";
    oss << "  Controls: elevator=" << controls.elevator << ", rudder=" << controls.rudder 
        << ", aileron=" << controls.aileron << "\n";
    return oss.str();
}

// TrimParameters implementation
bool TrimParameters::isValid() const {
    return force_tolerance > 0.0 && moment_tolerance > 0.0 && max_iterations > 0 &&
           step_size > 0.0 && step_reduction > 0.0 && step_reduction < 1.0 &&
           min_step_size > 0.0 && min_step_size < step_size &&
           max_control_deflection > 0.0 && max_attitude_change > 0.0 && max_velocity_change > 0.0;
}

void TrimParameters::reset() {
    force_tolerance = 1.0;
    moment_tolerance = 0.1;
    max_iterations = 100;
    step_size = 0.01;
    step_reduction = 0.5;
    min_step_size = 1e-6;
    max_control_deflection = M_PI / 6;
    max_attitude_change = M_PI / 4;
    max_velocity_change = 20.0;
}

// StabilityDerivatives implementation
bool StabilityDerivatives::isValid() const {
    auto isFinite = [](double val) { return std::isfinite(val); };
    
    return isFinite(CL_alpha) && isFinite(CL_beta) && isFinite(CL_p) && isFinite(CL_q) && isFinite(CL_r) &&
           isFinite(CD_alpha) && isFinite(CD_beta) && isFinite(CY_beta) && isFinite(CY_p) && isFinite(CY_r) &&
           isFinite(Cl_beta) && isFinite(Cl_p) && isFinite(Cl_r) && isFinite(Cm_alpha) && isFinite(Cm_q) &&
           isFinite(Cn_beta) && isFinite(Cn_p) && isFinite(Cn_r) && isFinite(CL_elevator) &&
           isFinite(CD_elevator) && isFinite(Cm_elevator) && isFinite(CY_rudder) && 
           isFinite(Cn_rudder) && isFinite(Cl_aileron);
}

void StabilityDerivatives::reset() {
    CL_alpha = CL_beta = CL_p = CL_q = CL_r = 0.0;
    CD_alpha = CD_beta = 0.0;
    CY_beta = CY_p = CY_r = 0.0;
    Cl_beta = Cl_p = Cl_r = 0.0;
    Cm_alpha = Cm_q = 0.0;
    Cn_beta = Cn_p = Cn_r = 0.0;
    CL_elevator = CD_elevator = Cm_elevator = 0.0;
    CY_rudder = Cn_rudder = 0.0;
    Cl_aileron = 0.0;
}

std::string StabilityDerivatives::toString() const {
    std::ostringstream oss;
    oss << "Stability Derivatives:\n";
    oss << "  Longitudinal:\n";
    oss << "    CL_alpha = " << CL_alpha << " /rad\n";
    oss << "    CD_alpha = " << CD_alpha << " /rad\n";
    oss << "    Cm_alpha = " << Cm_alpha << " /rad\n";
    oss << "    CL_q = " << CL_q << " /(rad/s)\n";
    oss << "    Cm_q = " << Cm_q << " /(rad/s)\n";
    oss << "  Lateral:\n";
    oss << "    CY_beta = " << CY_beta << " /rad\n";
    oss << "    Cl_beta = " << Cl_beta << " /rad\n";
    oss << "    Cn_beta = " << Cn_beta << " /rad\n";
    oss << "  Control:\n";
    oss << "    CL_elevator = " << CL_elevator << " /rad\n";
    oss << "    Cm_elevator = " << Cm_elevator << " /rad\n";
    oss << "    CY_rudder = " << CY_rudder << " /rad\n";
    oss << "    Cn_rudder = " << Cn_rudder << " /rad\n";
    oss << "    Cl_aileron = " << Cl_aileron << " /rad\n";
    return oss.str();
}

// LinearizedSystem implementation
bool LinearizedSystem::isValid() const {
    return state_size > 0 && control_size > 0 && output_size > 0 &&
           A.rows() == state_size && A.cols() == state_size &&
           B.rows() == state_size && B.cols() == control_size &&
           C.rows() == output_size && C.cols() == state_size &&
           D.rows() == output_size && D.cols() == control_size &&
           trim_state.isValid() && trim_controls.isValid();
}

void LinearizedSystem::reset() {
    A = Matrix();
    B = Matrix();
    C = Matrix();
    D = Matrix();
    state_size = control_size = output_size = 0;
    trim_state = KiteState();
    trim_controls = ControlInputs();
}

std::vector<std::complex<double>> LinearizedSystem::getEigenvalues() const {
    if (!isValid()) {
        return {};
    }
    
    // For now, return empty vector - would need eigenvalue solver
    // In production, would use a proper linear algebra library like Eigen
    return {};
}

bool LinearizedSystem::isStable() const {
    auto eigenvalues = getEigenvalues();
    for (const auto& eigenval : eigenvalues) {
        if (eigenval.real() >= 0.0) {
            return false;  // Unstable if any eigenvalue has non-negative real part
        }
    }
    return true;
}

std::string LinearizedSystem::toString() const {
    std::ostringstream oss;
    oss << "Linearized System:\n";
    oss << "  State size: " << state_size << "\n";
    oss << "  Control size: " << control_size << "\n";
    oss << "  Output size: " << output_size << "\n";
    oss << "  Stable: " << (isStable() ? "Yes" : "No") << "\n";
    return oss.str();
}

// FlightEnvelope implementation
bool FlightEnvelope::isValid() const {
    return min_airspeed >= 0.0 && max_airspeed > min_airspeed &&
           min_angle_of_attack < max_angle_of_attack &&
           stall_angle >= min_angle_of_attack && stall_angle <= max_angle_of_attack &&
           max_lift_to_drag >= 0.0 && best_glide_speed >= 0.0 && min_sink_speed >= 0.0;
}

void FlightEnvelope::reset() {
    min_airspeed = max_airspeed = 0.0;
    min_angle_of_attack = max_angle_of_attack = stall_angle = 0.0;
    stable_trims.clear();
    unstable_trims.clear();
    max_lift_to_drag = best_glide_speed = min_sink_speed = 0.0;
}

std::string FlightEnvelope::toString() const {
    std::ostringstream oss;
    oss << "Flight Envelope:\n";
    oss << "  Airspeed range: " << min_airspeed << " - " << max_airspeed << " m/s\n";
    oss << "  AoA range: " << min_angle_of_attack * 180.0 / M_PI << " - " 
        << max_angle_of_attack * 180.0 / M_PI << " deg\n";
    oss << "  Stall angle: " << stall_angle * 180.0 / M_PI << " deg\n";
    oss << "  Max L/D: " << max_lift_to_drag << "\n";
    oss << "  Best glide speed: " << best_glide_speed << " m/s\n";
    oss << "  Min sink speed: " << min_sink_speed << " m/s\n";
    oss << "  Stable trim conditions: " << stable_trims.size() << "\n";
    oss << "  Unstable trim conditions: " << unstable_trims.size() << "\n";
    return oss.str();
}

// StabilityAnalyzer implementation
StabilityAnalyzer::StabilityAnalyzer() 
    : geometry_set_(false), modules_configured_(false) {
}

void StabilityAnalyzer::setKiteGeometry(const KiteGeometry& geometry) {
    if (!geometry.isValid()) {
        throw std::invalid_argument("Invalid kite geometry provided to StabilityAnalyzer");
    }
    geometry_ = geometry;
    geometry_set_ = true;
}

void StabilityAnalyzer::setAerodynamicsCalculator(std::shared_ptr<AerodynamicsCalculator> aero_calc) {
    if (!aero_calc) {
        throw std::invalid_argument("Null aerodynamics calculator provided to StabilityAnalyzer");
    }
    aero_calc_ = aero_calc;
    modules_configured_ = (aero_calc_ && dynamics_engine_ && tether_model_ && wind_model_);
}

void StabilityAnalyzer::setDynamicsEngine(std::shared_ptr<DynamicsEngine> dynamics_engine) {
    if (!dynamics_engine) {
        throw std::invalid_argument("Null dynamics engine provided to StabilityAnalyzer");
    }
    dynamics_engine_ = dynamics_engine;
    modules_configured_ = (aero_calc_ && dynamics_engine_ && tether_model_ && wind_model_);
}

void StabilityAnalyzer::setTetherModel(std::shared_ptr<TetherModel> tether_model) {
    if (!tether_model) {
        throw std::invalid_argument("Null tether model provided to StabilityAnalyzer");
    }
    tether_model_ = tether_model;
    modules_configured_ = (aero_calc_ && dynamics_engine_ && tether_model_ && wind_model_);
}

void StabilityAnalyzer::setWindModel(std::shared_ptr<WindModel> wind_model) {
    if (!wind_model) {
        throw std::invalid_argument("Null wind model provided to StabilityAnalyzer");
    }
    wind_model_ = wind_model;
    modules_configured_ = (aero_calc_ && dynamics_engine_ && tether_model_ && wind_model_);
}

TrimCondition StabilityAnalyzer::findTrimCondition(const KiteState& initial_guess,
                                                  const ControlInputs& initial_controls) const {
    if (!isConfigured()) {
        throw std::runtime_error("StabilityAnalyzer not properly configured");
    }
    
    std::vector<std::string> no_fixed_variables;
    return optimizeTrimCondition(initial_guess, initial_controls, no_fixed_variables);
}

TrimCondition StabilityAnalyzer::findTrimConditionConstrained(const KiteState& initial_guess,
                                                             const std::vector<std::string>& fixed_variables,
                                                             const ControlInputs& initial_controls) const {
    if (!isConfigured()) {
        throw std::runtime_error("StabilityAnalyzer not properly configured");
    }
    
    return optimizeTrimCondition(initial_guess, initial_controls, fixed_variables);
}

std::vector<TrimCondition> StabilityAnalyzer::findMultipleTrimConditions(
    const std::vector<KiteState>& initial_guesses) const {
    
    std::vector<TrimCondition> results;
    results.reserve(initial_guesses.size());
    
    for (const auto& guess : initial_guesses) {
        try {
            TrimCondition trim = findTrimCondition(guess);
            results.push_back(trim);
        } catch (const std::exception&) {
            // If trim calculation fails, add an invalid trim condition
            TrimCondition failed_trim;
            failed_trim.state = guess;
            failed_trim.converged = false;
            results.push_back(failed_trim);
        }
    }
    
    return results;
}

StabilityDerivatives StabilityAnalyzer::calculateStabilityDerivatives(
    const TrimCondition& trim_condition, double perturbation_size) const {
    
    if (!isConfigured()) {
        throw std::runtime_error("StabilityAnalyzer not properly configured");
    }
    
    if (!trim_condition.isValid()) {
        throw std::invalid_argument("Invalid trim condition provided");
    }
    
    return calculateDerivativesFiniteDifference(trim_condition, perturbation_size);
}

LinearizedSystem StabilityAnalyzer::linearizeAroundTrim(const TrimCondition& trim_condition,
                                                       double perturbation_size) const {
    if (!isConfigured()) {
        throw std::runtime_error("StabilityAnalyzer not properly configured");
    }
    
    if (!trim_condition.isValid()) {
        throw std::invalid_argument("Invalid trim condition provided");
    }
    
    LinearizedSystem system;
    system.trim_state = trim_condition.state;
    system.trim_controls = trim_condition.controls;
    
    // Define system dimensions
    system.state_size = 13;     // 13-state kite model (pos, vel, quat, omega)
    system.control_size = 3;    // 3 control inputs (elevator, rudder, aileron)
    system.output_size = 13;    // Full state output
    
    // Calculate linearized matrices
    system.A = calculateStateMatrix(trim_condition, perturbation_size);
    system.B = calculateControlMatrix(trim_condition, perturbation_size);
    
    // For now, use identity matrices for C and D (full state output, no feedthrough)
    system.C = Matrix::identity(system.output_size, system.state_size);
    system.D = Matrix::zeros(system.output_size, system.control_size);
    
    return system;
}

FlightEnvelope StabilityAnalyzer::analyzeFlightEnvelope(double min_speed, double max_speed, double speed_step,
                                                       double min_alpha, double max_alpha, double alpha_step) const {
    if (!isConfigured()) {
        throw std::runtime_error("StabilityAnalyzer not properly configured");
    }
    
    FlightEnvelope envelope;
    
    // Scan the flight envelope for trim conditions
    auto all_trims = scanFlightEnvelope(min_speed, max_speed, speed_step, min_alpha, max_alpha, alpha_step);
    
    // Classify trim conditions as stable or unstable
    for (const auto& trim : all_trims) {
        if (trim.converged && trim.isTrimmed()) {
            if (isStableTrimCondition(trim)) {
                envelope.stable_trims.push_back(trim);
            } else {
                envelope.unstable_trims.push_back(trim);
            }
        }
    }
    
    // Calculate envelope boundaries
    if (!envelope.stable_trims.empty()) {
        // Find airspeed range
        auto min_speed_it = std::min_element(envelope.stable_trims.begin(), envelope.stable_trims.end(),
            [](const TrimCondition& a, const TrimCondition& b) {
                return a.state.velocity().magnitude() < b.state.velocity().magnitude();
            });
        auto max_speed_it = std::max_element(envelope.stable_trims.begin(), envelope.stable_trims.end(),
            [](const TrimCondition& a, const TrimCondition& b) {
                return a.state.velocity().magnitude() < b.state.velocity().magnitude();
            });
        
        envelope.min_airspeed = min_speed_it->state.velocity().magnitude();
        envelope.max_airspeed = max_speed_it->state.velocity().magnitude();
        
        // Find angle of attack range (simplified - would need proper flow angle calculation)
        envelope.min_angle_of_attack = min_alpha;
        envelope.max_angle_of_attack = max_alpha;
        envelope.stall_angle = max_alpha * 0.8;  // Simplified estimate
        
        // Calculate performance metrics (simplified)
        envelope.max_lift_to_drag = 15.0;  // Typical value for kites
        envelope.best_glide_speed = (envelope.min_airspeed + envelope.max_airspeed) * 0.6;
        envelope.min_sink_speed = envelope.best_glide_speed * 0.8;
    }
    
    return envelope;
}

bool StabilityAnalyzer::isConfigured() const {
    return geometry_set_ && modules_configured_ && validateConfiguration();
}

std::vector<std::string> StabilityAnalyzer::getConfigurationErrors() const {
    std::vector<std::string> errors;
    
    if (!geometry_set_) {
        errors.push_back("Kite geometry not set");
    }
    
    if (!aero_calc_) {
        errors.push_back("Aerodynamics calculator not set");
    } else if (!aero_calc_->isConfigured()) {
        errors.push_back("Aerodynamics calculator not properly configured");
    }
    
    if (!dynamics_engine_) {
        errors.push_back("Dynamics engine not set");
    } else if (!dynamics_engine_->isConfigured()) {
        errors.push_back("Dynamics engine not properly configured");
    }
    
    if (!tether_model_) {
        errors.push_back("Tether model not set");
    } else if (!tether_model_->isValid()) {
        errors.push_back("Tether model not properly configured");
    }
    
    if (!wind_model_) {
        errors.push_back("Wind model not set");
    }
    
    if (!trim_params_.isValid()) {
        errors.push_back("Invalid trim parameters");
    }
    
    return errors;
}

double StabilityAnalyzer::calculateTrimQuality(const Forces& residual_forces) {
    double force_magnitude = residual_forces.force.magnitude();
    double moment_magnitude = residual_forces.moment.magnitude();
    
    // Simple quality metric: exponential decay with residual magnitude
    double force_quality = std::exp(-force_magnitude / 10.0);    // Scale by 10 N
    double moment_quality = std::exp(-moment_magnitude / 1.0);   // Scale by 1 N⋅m
    
    return std::min(force_quality, moment_quality);
}

bool StabilityAnalyzer::isStableEigenvalue(const std::complex<double>& eigenvalue) {
    return eigenvalue.real() < 0.0;
}

// Private implementation methods
Forces StabilityAnalyzer::calculateTotalForces(const KiteState& state, const ControlInputs& controls, double time) const {
    Forces total_forces;
    
    // Get wind at current position
    WindVector wind = wind_model_->getWind(state.position(), time);
    
    // Calculate aerodynamic forces
    AeroForces aero_forces = aero_calc_->calculateForces(state, wind, controls);
    
    // Calculate tether forces
    TetherForces tether_forces = tether_model_->calculateTetherForces(
        state.position(), state.attitude(), state.velocity(), wind.velocity());
    
    // Combine forces
    total_forces.force = aero_forces.force + tether_forces.force;
    total_forces.moment = aero_forces.moment + tether_forces.moment;
    
    // Add gravity
    double mass = dynamics_engine_->getMassProperties().mass;
    Vector3 gravity_force(0.0, 0.0, -mass * 9.81);  // Assuming Z-up coordinate system
    total_forces.force += gravity_force;
    
    return total_forces;
}

TrimCondition StabilityAnalyzer::optimizeTrimCondition(const KiteState& initial_state,
                                                      const ControlInputs& initial_controls,
                                                      const std::vector<std::string>& fixed_variables) const {
    TrimCondition result;
    result.state = initial_state;
    result.controls = initial_controls;
    
    // Simple gradient descent optimization
    double current_step_size = trim_params_.step_size;
    
    for (int iter = 0; iter < trim_params_.max_iterations; ++iter) {
        // Calculate current forces
        Forces current_forces = calculateTotalForces(result.state, result.controls);
        
        // Calculate residuals
        double force_residual = current_forces.force.magnitude();
        double moment_residual = current_forces.moment.magnitude();
        
        // Check convergence
        if (force_residual <= trim_params_.force_tolerance && 
            moment_residual <= trim_params_.moment_tolerance) {
            result.converged = true;
            result.force_residual = force_residual;
            result.moment_residual = moment_residual;
            result.trim_quality = calculateTrimQuality(current_forces);
            result.iterations = iter;
            result.total_forces = current_forces;
            break;
        }
        
        // Simple perturbation-based optimization (simplified)
        // In production, would use proper optimization algorithm like Levenberg-Marquardt
        
        // Try small perturbations to controls
        ControlInputs perturbed_controls = result.controls;
        
        // Perturb elevator
        perturbed_controls.elevator += current_step_size;
        Forces perturbed_forces = calculateTotalForces(result.state, perturbed_controls);
        double perturbed_residual = perturbed_forces.force.magnitude() + perturbed_forces.moment.magnitude();
        double current_residual = force_residual + moment_residual;
        
        if (perturbed_residual < current_residual) {
            result.controls.elevator += current_step_size;
        } else {
            perturbed_controls.elevator = result.controls.elevator - current_step_size;
            perturbed_forces = calculateTotalForces(result.state, perturbed_controls);
            perturbed_residual = perturbed_forces.force.magnitude() + perturbed_forces.moment.magnitude();
            if (perturbed_residual < current_residual) {
                result.controls.elevator -= current_step_size;
            }
        }
        
        // Reduce step size if no improvement
        if (iter > 10 && force_residual > trim_params_.force_tolerance) {
            current_step_size *= trim_params_.step_reduction;
            if (current_step_size < trim_params_.min_step_size) {
                break;  // Step size too small, give up
            }
        }
    }
    
    // Final calculation
    if (!result.converged) {
        Forces final_forces = calculateTotalForces(result.state, result.controls);
        result.force_residual = final_forces.force.magnitude();
        result.moment_residual = final_forces.moment.magnitude();
        result.trim_quality = calculateTrimQuality(final_forces);
        result.total_forces = final_forces;
        result.iterations = trim_params_.max_iterations;
    }
    
    return result;
}

StabilityDerivatives StabilityAnalyzer::calculateDerivativesFiniteDifference(
    const TrimCondition& trim, double perturbation_size) const {
    
    StabilityDerivatives derivatives;
    
    // Calculate baseline forces
    AeroForces baseline = aero_calc_->calculateForces(
        trim.state, wind_model_->getWind(trim.state.position(), 0.0), trim.controls);
    
    // Calculate derivatives using finite differences
    // CL_alpha (lift coefficient derivative w.r.t. angle of attack)
    AeroForces perturbed_alpha = perturbState(trim.state, trim.controls, "alpha", perturbation_size);
    derivatives.CL_alpha = (perturbed_alpha.lift - baseline.lift) / 
                          (perturbation_size * baseline.dynamic_pressure * geometry_.wingArea());
    
    // CD_alpha (drag coefficient derivative w.r.t. angle of attack)
    derivatives.CD_alpha = (perturbed_alpha.drag - baseline.drag) / 
                          (perturbation_size * baseline.dynamic_pressure * geometry_.wingArea());
    
    // Cm_alpha (pitching moment coefficient derivative w.r.t. angle of attack)
    derivatives.Cm_alpha = (perturbed_alpha.pitch_moment - baseline.pitch_moment) / 
                          (perturbation_size * baseline.dynamic_pressure * geometry_.wingArea() * geometry_.meanChord());
    
    // Control derivatives
    AeroForces perturbed_elevator = perturbState(trim.state, trim.controls, "elevator", perturbation_size);
    derivatives.CL_elevator = (perturbed_elevator.lift - baseline.lift) / 
                             (perturbation_size * baseline.dynamic_pressure * geometry_.wingArea());
    derivatives.CD_elevator = (perturbed_elevator.drag - baseline.drag) / 
                             (perturbation_size * baseline.dynamic_pressure * geometry_.wingArea());
    derivatives.Cm_elevator = (perturbed_elevator.pitch_moment - baseline.pitch_moment) / 
                             (perturbation_size * baseline.dynamic_pressure * geometry_.wingArea() * geometry_.meanChord());
    
    // Simplified calculation for other derivatives (would need more sophisticated perturbation)
    derivatives.CL_beta = 0.0;  // Requires sideslip perturbation
    derivatives.CY_beta = 0.0;  // Requires side force calculation
    derivatives.Cl_beta = 0.0;  // Requires roll moment calculation
    derivatives.Cn_beta = 0.0;  // Requires yaw moment calculation
    
    // Rate derivatives (simplified)
    derivatives.CL_q = -2.0;    // Typical value for pitch rate damping
    derivatives.Cm_q = -5.0;    // Typical value for pitch damping
    
    return derivatives;
}

AeroForces StabilityAnalyzer::perturbState(const KiteState& state, const ControlInputs& controls,
                                          const std::string& variable, double delta) const {
    KiteState perturbed_state = state;
    ControlInputs perturbed_controls = controls;
    
    if (variable == "alpha") {
        // Perturb angle of attack by changing pitch attitude slightly
        Quaternion current_attitude = state.attitude();
        Quaternion pitch_perturbation = Quaternion::fromAxisAngle(Vector3(0, 1, 0), delta);
        perturbed_state.setAttitude(current_attitude * pitch_perturbation);
    } else if (variable == "elevator") {
        perturbed_controls.elevator += delta;
    } else if (variable == "rudder") {
        perturbed_controls.rudder += delta;
    } else if (variable == "aileron") {
        perturbed_controls.aileron += delta;
    }
    
    WindVector wind = wind_model_->getWind(perturbed_state.position(), 0.0);
    return aero_calc_->calculateForces(perturbed_state, wind, perturbed_controls);
}

Matrix StabilityAnalyzer::calculateStateMatrix(const TrimCondition& trim, double perturbation_size) const {
    // For now, return a simplified state matrix
    // In production, would calculate full 13x13 linearized dynamics matrix
    Matrix A = Matrix::zeros(13, 13);
    
    // Position derivatives (velocity)
    A(0, 3) = 1.0;  // dx/dt = vx
    A(1, 4) = 1.0;  // dy/dt = vy
    A(2, 5) = 1.0;  // dz/dt = vz
    
    // Simplified dynamics (would need full linearization)
    // This is a placeholder - real implementation would calculate all partial derivatives
    
    return A;
}

Matrix StabilityAnalyzer::calculateControlMatrix(const TrimCondition& trim, double perturbation_size) const {
    // For now, return a simplified control matrix
    // In production, would calculate full 13x3 control influence matrix
    Matrix B = Matrix::zeros(13, 3);
    
    // Simplified control influence (placeholder)
    B(4, 0) = 1.0;  // Elevator affects pitch rate
    B(5, 1) = 1.0;  // Rudder affects yaw rate
    B(3, 2) = 1.0;  // Aileron affects roll rate
    
    return B;
}

std::vector<TrimCondition> StabilityAnalyzer::scanFlightEnvelope(double min_speed, double max_speed, double speed_step,
                                                                double min_alpha, double max_alpha, double alpha_step) const {
    std::vector<TrimCondition> trim_conditions;
    
    for (double speed = min_speed; speed <= max_speed; speed += speed_step) {
        for (double alpha = min_alpha; alpha <= max_alpha; alpha += alpha_step) {
            // Create initial guess state
            KiteState initial_state;
            initial_state.setPosition(Vector3(0.0, 0.0, 100.0));  // 100m altitude
            initial_state.setVelocity(Vector3(speed, 0.0, 0.0));   // Forward velocity
            
            // Set attitude based on angle of attack
            Quaternion attitude = Quaternion::fromAxisAngle(Vector3(0, 1, 0), alpha);
            initial_state.setAttitude(attitude);
            initial_state.setAngularVelocity(Vector3(0.0, 0.0, 0.0));
            initial_state.setTime(0.0);
            
            try {
                TrimCondition trim = findTrimCondition(initial_state);
                trim_conditions.push_back(trim);
            } catch (const std::exception&) {
                // Skip failed trim calculations
            }
        }
    }
    
    return trim_conditions;
}

bool StabilityAnalyzer::isStableTrimCondition(const TrimCondition& trim) const {
    // Simplified stability check - in production would use eigenvalue analysis
    // For now, just check if the trim is well-converged
    return trim.converged && trim.trim_quality > 0.8;
}

bool StabilityAnalyzer::validateConfiguration() const {
    return geometry_.isValid() && trim_params_.isValid();
}

bool StabilityAnalyzer::validateTrimCondition(const TrimCondition& trim) const {
    return trim.isValid();
}

} // namespace kite_sim