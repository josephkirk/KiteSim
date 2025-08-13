#pragma once

#include "kite_state.h"
#include "kite_geometry.h"
#include "dynamics.h"
#include "aerodynamics.h"
#include "wind_model.h"
#include "tether.h"
#include "integration_engine.h"
#include <memory>
#include <functional>
#include <string>
#include <vector>

namespace kite_sim {

/**
 * @brief Simulation configuration parameters
 */
struct SimulationConfig {
    double time_step = 0.01;           // Fixed time step for output (s)
    double simulation_duration = 60.0;  // Total simulation time (s)
    double start_time = 0.0;           // Simulation start time (s)
    
    // Integration parameters
    IntegrationMethod integration_method = IntegrationMethod::RUNGE_KUTTA_45;
    IntegrationParameters integration_params;
    
    // Output configuration
    bool enable_data_logging = true;
    double logging_interval = 0.1;    // Data logging interval (s)
    std::string output_file = "";     // Output file path (empty = no file output)
    
    // Validation
    bool isValid() const;
    
    // Serialization
    std::string toJson() const;
    bool fromJson(const std::string& json_str);
};

/**
 * @brief Simulation results and statistics
 */
struct SimulationResults {
    bool success = false;
    double actual_duration = 0.0;
    double final_time = 0.0;
    size_t total_steps = 0;
    double average_step_size = 0.0;
    double min_step_size = 0.0;
    double max_step_size = 0.0;
    std::string error_message = "";
    
    // Integration statistics
    IntegrationStats integration_stats;
    
    void reset();
    std::string toString() const;
};

/**
 * @brief Callback function type for real-time data streaming
 * 
 * Called at each simulation step with current state and forces.
 * Return false to stop simulation early.
 */
using SimulationCallback = std::function<bool(double time, const KiteState& state, 
                                            const Forces& total_forces, 
                                            const AeroForces& aero_forces,
                                            const TetherForces& tether_forces)>;

/**
 * @brief Main kite simulation engine that orchestrates all modules
 * 
 * The KiteSimulator class provides the main interface for running kite simulations.
 * It coordinates the aerodynamics, dynamics, tether mechanics, wind modeling, and
 * numerical integration modules to provide complete 6-DOF kite flight simulation.
 */
class KiteSimulator {
public:
    // Constructors
    KiteSimulator();
    explicit KiteSimulator(const SimulationConfig& config);
    
    // Copy constructor and assignment (deleted - use move semantics)
    KiteSimulator(const KiteSimulator& other) = delete;
    KiteSimulator& operator=(const KiteSimulator& other) = delete;
    
    // Move constructor and assignment
    KiteSimulator(KiteSimulator&& other) noexcept = default;
    KiteSimulator& operator=(KiteSimulator&& other) noexcept = default;
    
    // Destructor
    ~KiteSimulator() = default;
    
    // Configuration
    void setConfig(const SimulationConfig& config) { config_ = config; }
    const SimulationConfig& getConfig() const { return config_; }
    
    void setKiteGeometry(const KiteGeometry& geometry);
    const KiteGeometry& getKiteGeometry() const;
    
    void setTetherProperties(const TetherProperties& properties);
    const TetherProperties& getTetherProperties() const;
    
    void setWindModel(std::unique_ptr<WindModel> wind_model);
    const WindModel* getWindModel() const { return wind_model_.get(); }
    
    void setInitialState(const KiteState& initial_state) { initial_state_ = initial_state; }
    const KiteState& getInitialState() const { return initial_state_; }
    
    // Simulation control
    SimulationResults run();
    SimulationResults run(const KiteState& initial_state);
    SimulationResults runWithCallback(const SimulationCallback& callback);
    
    bool step(double target_time);
    bool step();  // Single step with automatic time increment
    
    void reset();
    void stop() { stop_requested_ = true; }
    
    // State access
    const KiteState& getCurrentState() const { return current_state_; }
    double getCurrentTime() const { return current_state_.time(); }
    bool isRunning() const { return is_running_; }
    
    // Force access (from last simulation step)
    const Forces& getTotalForces() const { return total_forces_; }
    const AeroForces& getAeroForces() const { return aero_forces_; }
    const TetherForces& getTetherForces() const { return tether_forces_; }
    
    // Results access
    const SimulationResults& getLastResults() const { return last_results_; }
    
    // Data logging
    void enableDataLogging(bool enable) { config_.enable_data_logging = enable; }
    void setDataLoggingInterval(double interval) { config_.logging_interval = interval; }
    void setOutputFile(const std::string& filename) { config_.output_file = filename; }
    
    // Callback management
    void setCallback(const SimulationCallback& callback) { callback_ = callback; }
    void clearCallback() { callback_ = nullptr; }
    
    // Validation
    bool isConfigured() const;
    std::vector<std::string> getConfigurationErrors() const;
    
    // Factory methods for common configurations
    static KiteSimulator createBasicSimulator(const KiteGeometry& geometry,
                                             const TetherProperties& tether,
                                             double wind_speed = 10.0,
                                             double wind_direction = 0.0);
    
    static KiteSimulator createFromConfigFile(const std::string& config_file);

private:
    // Configuration
    SimulationConfig config_;
    KiteState initial_state_;
    
    // Simulation modules
    std::unique_ptr<DynamicsEngine> dynamics_engine_;
    std::unique_ptr<AerodynamicsCalculator> aero_calculator_;
    std::unique_ptr<TetherModel> tether_model_;
    std::unique_ptr<WindModel> wind_model_;
    std::unique_ptr<IntegrationEngine> integration_engine_;
    
    // Current simulation state
    KiteState current_state_;
    Forces total_forces_;
    AeroForces aero_forces_;
    TetherForces tether_forces_;
    ControlInputs control_inputs_;  // Currently zero, for future extension
    
    // Simulation control
    bool is_running_;
    bool stop_requested_;
    double next_log_time_;
    
    // Results and statistics
    SimulationResults last_results_;
    
    // Callback
    SimulationCallback callback_;
    
    // Data logging
    std::vector<KiteState> logged_states_;
    std::vector<Forces> logged_forces_;
    std::vector<double> logged_times_;
    
    // Internal methods
    void initializeModules();
    void configureIntegration();
    
    Forces calculateTotalForces(const KiteState& state, double time);
    void updateForces(const KiteState& state, double time);
    
    bool performSimulationStep(double target_time);
    void logData(double time, const KiteState& state, const Forces& forces);
    void writeDataToFile() const;
    
    // Derivative function for integration engine
    void calculateDerivative(double time, const double* state_vector, double* derivative_vector);
    
    // Validation helpers
    bool validateConfiguration() const;
    bool validateInitialState() const;
    bool validateModules() const;
    
    // Error handling
    void handleSimulationError(const std::string& error_message);
    void updateResults(bool success, const std::string& error_message = "");
};

} // namespace kite_sim