#pragma once

#include "math_types.h"
#include <string>
#include <vector>
#include <map>

namespace kite_sim {

/**
 * @brief Mass properties of the kite
 */
struct MassProperties {
    double mass;                    // Total mass (kg)
    Vector3 center_of_gravity;      // CG position in body coordinates (m)
    Matrix3x3 inertia_tensor;       // Inertia tensor about CG (kg⋅m²)
    
    MassProperties();
    MassProperties(double m, const Vector3& cg, const Matrix3x3& inertia);
    
    bool isValid() const;
    std::string toJson() const;
    bool fromJson(const std::string& json_str);
};

/**
 * @brief Aerodynamic coefficient lookup table with support for 1D, 2D, and 3D tables
 * 
 * Supports coefficient tables as functions of:
 * - 1D: angle of attack (α) only
 * - 2D: angle of attack (α) and sideslip angle (β)
 * - 3D: angle of attack (α), sideslip angle (β), and control deflection (δ)
 */
class AeroCoefficients {
public:
    enum class TableType {
        NONE,
        TABLE_1D,  // f(α)
        TABLE_2D,  // f(α, β)
        TABLE_3D   // f(α, β, δ)
    };
    
    AeroCoefficients();
    
    // 1D coefficient tables (angle of attack only)
    void setCL1D(const std::vector<double>& alpha_deg, const std::vector<double>& cl);
    void setCD1D(const std::vector<double>& alpha_deg, const std::vector<double>& cd);
    void setCM1D(const std::vector<double>& alpha_deg, const std::vector<double>& cm);
    
    // 2D coefficient tables (angle of attack and sideslip)
    void setCL2D(const std::vector<double>& alpha_deg, const std::vector<double>& beta_deg,
                 const std::vector<std::vector<double>>& cl_table);
    void setCD2D(const std::vector<double>& alpha_deg, const std::vector<double>& beta_deg,
                 const std::vector<std::vector<double>>& cd_table);
    void setCM2D(const std::vector<double>& alpha_deg, const std::vector<double>& beta_deg,
                 const std::vector<std::vector<double>>& cm_table);
    
    // 3D coefficient tables (angle of attack, sideslip, and control deflection)
    void setCL3D(const std::vector<double>& alpha_deg, const std::vector<double>& beta_deg,
                 const std::vector<double>& delta_deg,
                 const std::vector<std::vector<std::vector<double>>>& cl_table);
    void setCD3D(const std::vector<double>& alpha_deg, const std::vector<double>& beta_deg,
                 const std::vector<double>& delta_deg,
                 const std::vector<std::vector<std::vector<double>>>& cd_table);
    void setCM3D(const std::vector<double>& alpha_deg, const std::vector<double>& beta_deg,
                 const std::vector<double>& delta_deg,
                 const std::vector<std::vector<std::vector<double>>>& cm_table);
    
    // Coefficient lookup with interpolation
    double getCL(double alpha_deg, double beta_deg = 0.0, double delta_deg = 0.0) const;
    double getCD(double alpha_deg, double beta_deg = 0.0, double delta_deg = 0.0) const;
    double getCM(double alpha_deg, double beta_deg = 0.0, double delta_deg = 0.0) const;
    
    // Table type queries
    TableType getCLTableType() const { return cl_table_type_; }
    TableType getCDTableType() const { return cd_table_type_; }
    TableType getCMTableType() const { return cm_table_type_; }
    
    // Validation
    bool isValid() const;
    bool hasData() const;
    bool isWithinBounds(double alpha_deg, double beta_deg = 0.0, double delta_deg = 0.0) const;
    
    // Bounds information
    std::pair<double, double> getAlphaBounds() const;
    std::pair<double, double> getBetaBounds() const;
    std::pair<double, double> getDeltaBounds() const;
    
    // Serialization
    std::string toJson() const;
    bool fromJson(const std::string& json_str);
    
    // Backward compatibility methods (delegate to 1D versions)
    void setCL(const std::vector<double>& alpha_deg, const std::vector<double>& cl) {
        setCL1D(alpha_deg, cl);
    }
    void setCD(const std::vector<double>& alpha_deg, const std::vector<double>& cd) {
        setCD1D(alpha_deg, cd);
    }
    void setCM(const std::vector<double>& alpha_deg, const std::vector<double>& cm) {
        setCM1D(alpha_deg, cm);
    }
    
private:
    // 1D table data
    std::vector<double> alpha_points_;
    std::vector<double> cl_1d_, cd_1d_, cm_1d_;
    
    // 2D table data
    std::vector<double> beta_points_;
    std::vector<std::vector<double>> cl_2d_, cd_2d_, cm_2d_;
    
    // 3D table data
    std::vector<double> delta_points_;
    std::vector<std::vector<std::vector<double>>> cl_3d_, cd_3d_, cm_3d_;
    
    // Table type tracking
    TableType cl_table_type_, cd_table_type_, cm_table_type_;
    
    // Interpolation methods
    double interpolate1D(const std::vector<double>& x_vals, 
                        const std::vector<double>& y_vals, 
                        double x) const;
    
    double interpolate2D(const std::vector<double>& x_vals,
                        const std::vector<double>& y_vals,
                        const std::vector<std::vector<double>>& z_table,
                        double x, double y) const;
    
    double interpolate3D(const std::vector<double>& x_vals,
                        const std::vector<double>& y_vals,
                        const std::vector<double>& z_vals,
                        const std::vector<std::vector<std::vector<double>>>& w_table,
                        double x, double y, double z) const;
    
    // Validation helpers
    bool isMonotonic(const std::vector<double>& values) const;
    bool validateTable1D(const std::vector<double>& x_vals, const std::vector<double>& y_vals) const;
    bool validateTable2D(const std::vector<double>& x_vals, const std::vector<double>& y_vals,
                        const std::vector<std::vector<double>>& z_table) const;
    bool validateTable3D(const std::vector<double>& x_vals, const std::vector<double>& y_vals,
                        const std::vector<double>& z_vals,
                        const std::vector<std::vector<std::vector<double>>>& w_table) const;
    
    // Bounds checking
    bool isInBounds(double value, const std::vector<double>& bounds) const;
    
    // Helper to find interpolation indices and weights
    struct InterpolationData {
        size_t lower_index;
        size_t upper_index;
        double weight;
        bool extrapolating;
    };
    
    InterpolationData findInterpolationData(const std::vector<double>& values, double target) const;
};

/**
 * @brief Complete geometric and aerodynamic properties of a kite
 * 
 * Contains all physical parameters needed for kite simulation including
 * mass properties, aerodynamic coefficients, and geometric dimensions.
 */
class KiteGeometry {
public:
    // Constructors
    KiteGeometry();
    explicit KiteGeometry(const std::string& config_file);
    
    // Copy constructor and assignment
    KiteGeometry(const KiteGeometry& other) = default;
    KiteGeometry& operator=(const KiteGeometry& other) = default;
    
    // Move constructor and assignment
    KiteGeometry(KiteGeometry&& other) noexcept = default;
    KiteGeometry& operator=(KiteGeometry&& other) noexcept = default;
    
    // Destructor
    ~KiteGeometry() = default;
    
    // Wing geometry properties
    double wingArea() const { return wing_area_; }
    void setWingArea(double area) { wing_area_ = area; }
    
    double aspectRatio() const { return aspect_ratio_; }
    void setAspectRatio(double ar) { aspect_ratio_ = ar; }
    
    double wingspan() const { return wingspan_; }
    void setWingspan(double span) { wingspan_ = span; }
    
    double meanChord() const { return mean_chord_; }
    void setMeanChord(double chord) { mean_chord_ = chord; }
    
    // Mass properties access
    MassProperties& massProperties() { return mass_props_; }
    const MassProperties& massProperties() const { return mass_props_; }
    void setMassProperties(const MassProperties& props) { mass_props_ = props; }
    
    // Aerodynamic coefficients access
    AeroCoefficients& aeroCoefficients() { return aero_coeffs_; }
    const AeroCoefficients& aeroCoefficients() const { return aero_coeffs_; }
    void setAeroCoefficients(const AeroCoefficients& coeffs) { aero_coeffs_ = coeffs; }
    
    // Reference points (in body coordinates)
    Vector3 aerodynamicCenter() const { return aero_center_; }
    void setAerodynamicCenter(const Vector3& ac) { aero_center_ = ac; }
    
    Vector3 tetherAttachmentPoint() const { return tether_attachment_; }
    void setTetherAttachmentPoint(const Vector3& tap) { tether_attachment_ = tap; }
    
    // Validation
    bool isValid() const;
    std::vector<std::string> getValidationErrors() const;
    
    // Configuration management
    bool loadFromFile(const std::string& filename);
    bool saveToFile(const std::string& filename) const;
    
    // Serialization
    std::string toJson() const;
    bool fromJson(const std::string& json_str);
    
    // Comparison operators
    bool operator==(const KiteGeometry& other) const;
    bool operator!=(const KiteGeometry& other) const;
    
    // Default configurations
    static KiteGeometry createDefaultKite();
    static KiteGeometry createTestKite();

private:
    // Wing geometry
    double wing_area_;          // Wing area (m²)
    double aspect_ratio_;       // Aspect ratio (dimensionless)
    double wingspan_;           // Wing span (m)
    double mean_chord_;         // Mean aerodynamic chord (m)
    
    // Mass and inertia properties
    MassProperties mass_props_;
    
    // Aerodynamic properties
    AeroCoefficients aero_coeffs_;
    
    // Reference points
    Vector3 aero_center_;       // Aerodynamic center in body coordinates (m)
    Vector3 tether_attachment_; // Tether attachment point in body coordinates (m)
    
    // Validation helpers
    bool isWingGeometryValid() const;
    bool areMassPropertiesValid() const;
    bool areAeroCoefficientsValid() const;
    bool areReferencePointsValid() const;
    
    // Consistency checks
    void updateDerivedProperties();
    bool checkGeometricConsistency() const;
};

} // namespace kite_sim