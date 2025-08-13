#include "kite_sim/kite_simulator.h"
#include "kite_sim/constants.h"
#include <fstream>
#include <sstream>
#include <iomanip>
#include <algorithm>
#include <cmath>

namespace kite_sim {

// SimulationConfig implementation
bool SimulationConfig::isValid() const {
    return time_step > 0.0 && 
           simulation_duration > 0.0 && 
           start_time >= 0.0 &&
           logging_interval > 0.0 &&
           integration_params.isValid();
}

std::string SimulationConfig::toJson() const {
    std::ostringstream oss;
    oss << "{\n";
    oss << "  \"time_step\": " << time_step << ",\n";
    oss << "  \"simulation_duration\": " << simulation_duration << ",\n";
    oss << "  \"start_time\": " << start_time << ",\n";
    oss << "  \"integration_method\": " << static_cast<int>(integration_method) << ",\n";
    oss << "  \"enable_data_logging\": " << (enable_data_logging ? "true" : "false") << ",\n";
    oss << "  \"logging_interval\": " << logging_interval << ",\n";
    oss << "  \"output_file\": \"" << output_file << "\"\n";
    oss << "}";
    return oss.str();
}

bool SimulationConfig::fromJson(const std::string& json_str) {
    // Simple JSON parsing - in production would use proper JSON library
    // For now, just return true and use defaults
    return true;
}

// SimulationResults implementation
void SimulationResults::reset() {
    success = false;
    actual_duration = 0.0;
    final_time = 0.0;
    total_steps = 0;
    average_step_size = 0.0;
    min_step_size = 0.0;
    max_step_size = 0.0;
    error_message = "";
    integration_stats.reset();
}

std::string SimulationResults::toString() const {
    std::ostringstream oss;
    oss << "Simulation Results:\n";
    oss << "  Success: " << (success ? "Yes" : "No") << "\n";
    oss << "  Duration: " << actual_duration << " s\n";
    oss << "  Final Time: " << final_time << " s\n";
    oss << "  Total Steps: " << total_steps << "\n";
    oss << "  Average Step Size: " << average_step_size << " s\n";
    oss << "  Min Step Size: " << min_step_size << " s\n";
    oss << "  Max Step Size: " << max_step_size << " s\n";
    if (!error_message.empty()) {
        oss << "  Error: " << error_message << "\n";
    }
    return oss.str();
}

// KiteSimulator implementation
KiteSimulator::KiteSimulator() 
    : config_()
    , initial_state_()
    , is_running_(false)
    , stop_requested_(false)
    , next_log_time_(0.0)
    , callback_(nullptr) {
    initializeModules();
}

KiteSimulator::KiteSimulator(const SimulationConfig& config)
    : config_(config)
    , initial_state_()
    , is_running_(false)
    , stop_requested_(false)
    , next_log_time_(0.0)
    , callback_(nullptr) {
    initializeModules();
}

void KiteSimulator::initializeModules() {
    dynamics_engine_ = std::make_unique<DynamicsEngine>(IntegrationMethod::RUNGE_KUTTA_4);
    aero_calculator_ = std::make_unique<AerodynamicsCalculator>();
    tether_model_ = std::make_unique<TetherModel>();
    wind_model_ = std::make_unique<UniformWindModel>(10.0, 0.0);  // Default 10 m/s wind
    integration_engine_ = std::make_unique<IntegrationEngine>(config_.integration_method);
    
    configureIntegration();
}

void KiteSimulator::configureIntegration() {
    if (integration_engine_) {
        integration_engine_->setParameters(config_.integration_params);
        
        // Set up derivative function
        auto derivative_func = [this](double time, const double* state_vector, double* derivative_vector) {
            this->calculateDerivative(time, state_vector, derivative_vector);
        };
        integration_engine_->setDerivativeFunction(derivative_func);
    }
}

void KiteSimulator::setKiteGeometry(const KiteGeometry& geometry) {
    if (aero_calculator_) {
        aero_calculator_->setGeometry(geometry);
    }
    if (dynamics_engine_) {
        dynamics_engine_->setMassProperties(geometry.massProperties());
    }
}

const KiteGeometry& KiteSimulator::getKiteGeometry() const {
    if (!aero_calculator_) {
        throw std::runtime_error("Aerodynamics calculator not initialized");
    }
    return aero_calculator_->getGeometry();
}

void KiteSimulator::setTetherProperties(const TetherProperties& properties) {
    if (tether_model_) {
        tether_model_->setProperties(properties);
    }
}

const TetherProperties& KiteSimulator::getTetherProperties() const {
    if (!tether_model_) {
        throw std::runtime_error("Tether model not initialized");
    }
    return tether_model_->properties();
}

void KiteSimulator::setWindModel(std::unique_ptr<WindModel> wind_model) {
    wind_model_ = std::move(wind_model);
}

SimulationResults KiteSimulator::run() {
    return run(initial_state_);
}

SimulationResults KiteSimulator::run(const KiteState& initial_state) {
    return runWithCallback(callback_);
}

SimulationResults KiteSimulator::runWithCallback(const SimulationCallback& callback) {
    last_results_.reset();
    
    // Validate configuration
    if (!isConfigured()) {
        auto errors = getConfigurationErrors();
        std::string error_msg = "Configuration errors: ";
        for (const auto& error : errors) {
            error_msg += error + "; ";
        }
        handleSimulationError(error_msg);
        return last_results_;
    }
    
    // Initialize simulation
    current_state_ = initial_state_;
    current_state_.setTime(config_.start_time);
    is_running_ = true;
    stop_requested_ = false;
    next_log_time_ = config_.start_time;
    
    // Clear previous data
    logged_states_.clear();
    logged_forces_.clear();
    logged_times_.clear();
    
    // Log initial state
    if (config_.enable_data_logging) {
        updateForces(current_state_, current_state_.time());
        logData(current_state_.time(), current_state_, total_forces_);
    }
    
    double target_time = config_.start_time + config_.simulation_duration;
    double current_time = current_state_.time();
    
    try {
        while (current_time < target_time && !stop_requested_) {
            // Calculate next step target time
            double step_target = std::min(current_time + config_.time_step, target_time);
            
            // Perform simulation step
            if (!performSimulationStep(step_target)) {
                handleSimulationError("Integration step failed");
                break;
            }
            
            current_time = current_state_.time();
            
            // Call user callback if provided
            if (callback) {
                updateForces(current_state_, current_time);
                if (!callback(current_time, current_state_, total_forces_, 
                             aero_forces_, tether_forces_)) {
                    // User requested early termination
                    break;
                }
            }
            
            // Log data if needed
            if (config_.enable_data_logging && current_time >= next_log_time_) {
                updateForces(current_state_, current_time);
                logData(current_time, current_state_, total_forces_);
                next_log_time_ += config_.logging_interval;
            }
        }
        
        // Simulation completed successfully
        updateResults(true);
        
    } catch (const std::exception& e) {
        handleSimulationError(std::string("Exception during simulation: ") + e.what());
    }
    
    is_running_ = false;
    
    // Write data to file if requested
    if (config_.enable_data_logging && !config_.output_file.empty()) {
        writeDataToFile();
    }
    
    return last_results_;
}

bool KiteSimulator::step(double target_time) {
    if (!isConfigured()) {
        return false;
    }
    
    if (!is_running_) {
        current_state_ = initial_state_;
        is_running_ = true;
    }
    
    return performSimulationStep(target_time);
}

bool KiteSimulator::step() {
    double target_time = current_state_.time() + config_.time_step;
    return step(target_time);
}

void KiteSimulator::reset() {
    current_state_ = initial_state_;
    is_running_ = false;
    stop_requested_ = false;
    next_log_time_ = config_.start_time;
    last_results_.reset();
    
    logged_states_.clear();
    logged_forces_.clear();
    logged_times_.clear();
    
    if (integration_engine_) {
        integration_engine_->resetStats();
    }
}

Forces KiteSimulator::calculateTotalForces(const KiteState& state, double time) {
    Forces total_forces;
    
    // Get wind at current position
    WindVector wind = wind_model_->getWind(state.position(), time);
    
    // Calculate aerodynamic forces
    aero_forces_ = aero_calculator_->calculateForces(state, wind, control_inputs_);
    
    // Calculate tether forces
    tether_forces_ = tether_model_->calculateTetherForces(
        state.position(), state.attitude(), state.velocity(), wind.velocity());
    
    // Combine forces
    total_forces.force = aero_forces_.force + tether_forces_.force;
    total_forces.moment = aero_forces_.moment + tether_forces_.moment;
    
    // Add gravity
    double mass = dynamics_engine_->getMassProperties().mass;
    Vector3 gravity_force(0.0, 0.0, -mass * 9.81);  // Assuming Z-up coordinate system
    total_forces.force += gravity_force;
    
    return total_forces;
}

void KiteSimulator::updateForces(const KiteState& state, double time) {
    total_forces_ = calculateTotalForces(state, time);
}

bool KiteSimulator::performSimulationStep(double target_time) {
    try {
        // Update forces for current state
        updateForces(current_state_, current_state_.time());
        
        // Perform integration step
        double step_size = target_time - current_state_.time();
        bool success = integration_engine_->integrateStep(current_state_, step_size);
        
        if (!success) {
            return false;
        }
        
        // Validate the new state
        if (!current_state_.isValid()) {
            handleSimulationError("Invalid state after integration step");
            return false;
        }
        
        return true;
        
    } catch (const std::exception& e) {
        handleSimulationError(std::string("Error in simulation step: ") + e.what());
        return false;
    }
}

void KiteSimulator::calculateDerivative(double time, const double* state_vector, double* derivative_vector) {
    // Convert state vector to KiteState
    KiteState temp_state;
    temp_state.fromStateVector(state_vector);
    temp_state.setTime(time);
    
    // Calculate forces
    Forces forces = calculateTotalForces(temp_state, time);
    
    // Calculate state derivative using dynamics engine
    auto derivative = dynamics_engine_->calculateDerivative(temp_state, forces);
    
    // Convert derivative to array
    derivative.toArray(derivative_vector);
}

void KiteSimulator::logData(double time, const KiteState& state, const Forces& forces) {
    logged_times_.push_back(time);
    logged_states_.push_back(state);
    logged_forces_.push_back(forces);
}

void KiteSimulator::writeDataToFile() const {
    if (config_.output_file.empty() || logged_states_.empty()) {
        return;
    }
    
    std::ofstream file(config_.output_file);
    if (!file.is_open()) {
        return;
    }
    
    // Write header
    file << "time,pos_x,pos_y,pos_z,vel_x,vel_y,vel_z,";
    file << "quat_w,quat_x,quat_y,quat_z,omega_x,omega_y,omega_z,";
    file << "force_x,force_y,force_z,moment_x,moment_y,moment_z\n";
    
    // Write data
    file << std::fixed << std::setprecision(6);
    for (size_t i = 0; i < logged_states_.size(); ++i) {
        const auto& state = logged_states_[i];
        const auto& forces = logged_forces_[i];
        double time = logged_times_[i];
        
        file << time << ",";
        file << state.position().x() << "," << state.position().y() << "," << state.position().z() << ",";
        file << state.velocity().x() << "," << state.velocity().y() << "," << state.velocity().z() << ",";
        file << state.attitude().w() << "," << state.attitude().x() << "," 
             << state.attitude().y() << "," << state.attitude().z() << ",";
        file << state.angularVelocity().x() << "," << state.angularVelocity().y() << "," 
             << state.angularVelocity().z() << ",";
        file << forces.force.x() << "," << forces.force.y() << "," << forces.force.z() << ",";
        file << forces.moment.x() << "," << forces.moment.y() << "," << forces.moment.z() << "\n";
    }
    
    file.close();
}

bool KiteSimulator::isConfigured() const {
    return validateConfiguration() && validateModules() && validateInitialState();
}

std::vector<std::string> KiteSimulator::getConfigurationErrors() const {
    std::vector<std::string> errors;
    
    if (!config_.isValid()) {
        errors.push_back("Invalid simulation configuration");
    }
    
    if (!aero_calculator_ || !aero_calculator_->isConfigured()) {
        errors.push_back("Aerodynamics calculator not properly configured");
    }
    
    if (!dynamics_engine_ || !dynamics_engine_->isConfigured()) {
        errors.push_back("Dynamics engine not properly configured");
    }
    
    if (!tether_model_ || !tether_model_->isValid()) {
        errors.push_back("Tether model not properly configured");
    }
    
    if (!wind_model_) {
        errors.push_back("Wind model not set");
    }
    
    if (!integration_engine_ || !integration_engine_->isConfigured()) {
        errors.push_back("Integration engine not properly configured");
    }
    
    if (!initial_state_.isValid()) {
        errors.push_back("Invalid initial state");
    }
    
    return errors;
}

bool KiteSimulator::validateConfiguration() const {
    return config_.isValid();
}

bool KiteSimulator::validateInitialState() const {
    return initial_state_.isValid();
}

bool KiteSimulator::validateModules() const {
    return aero_calculator_ && aero_calculator_->isConfigured() &&
           dynamics_engine_ && dynamics_engine_->isConfigured() &&
           tether_model_ && tether_model_->isValid() &&
           wind_model_ &&
           integration_engine_ && integration_engine_->isConfigured();
}

void KiteSimulator::handleSimulationError(const std::string& error_message) {
    updateResults(false, error_message);
    is_running_ = false;
}

void KiteSimulator::updateResults(bool success, const std::string& error_message) {
    last_results_.success = success;
    last_results_.actual_duration = current_state_.time() - config_.start_time;
    last_results_.final_time = current_state_.time();
    last_results_.error_message = error_message;
    
    if (integration_engine_) {
        last_results_.integration_stats = integration_engine_->getStats();
        last_results_.total_steps = last_results_.integration_stats.total_steps;
        last_results_.average_step_size = last_results_.integration_stats.avg_step_size;
        last_results_.min_step_size = last_results_.integration_stats.min_step_size;
        last_results_.max_step_size = last_results_.integration_stats.max_step_size;
    }
}

// Factory methods
KiteSimulator KiteSimulator::createBasicSimulator(const KiteGeometry& geometry,
                                                  const TetherProperties& tether,
                                                  double wind_speed,
                                                  double wind_direction) {
    KiteSimulator simulator;
    
    // Set geometry and tether
    simulator.setKiteGeometry(geometry);
    simulator.setTetherProperties(tether);
    
    // Create wind model
    auto wind_model = std::make_unique<UniformWindModel>(wind_speed, wind_direction);
    simulator.setWindModel(std::move(wind_model));
    
    // Set reasonable initial state
    KiteState initial_state;
    initial_state.setPosition(Vector3(0.0, 0.0, tether.length() * 0.8));  // Start at 80% of tether length height
    initial_state.setVelocity(Vector3(wind_speed * 0.5, 0.0, 0.0));       // Start with some forward velocity
    initial_state.setAttitude(Quaternion(1.0, 0.0, 0.0, 0.0));           // Level attitude
    initial_state.setAngularVelocity(Vector3(0.0, 0.0, 0.0));            // No initial rotation
    initial_state.setTime(0.0);
    
    simulator.setInitialState(initial_state);
    
    return simulator;
}

KiteSimulator KiteSimulator::createFromConfigFile(const std::string& config_file) {
    // For now, return a basic simulator
    // In production, would parse JSON config file
    KiteGeometry geometry = KiteGeometry::createDefaultKite();
    TetherProperties tether = TetherProperties::createDefaultTether();
    
    return createBasicSimulator(geometry, tether);
}

} // namespace kite_sim