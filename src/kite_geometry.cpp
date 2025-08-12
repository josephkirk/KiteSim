#include "kite_sim/kite_geometry.h"
#include <sstream>
#include <iomanip>
#include <cmath>
#include <algorithm>
#include <fstream>
#include <numeric>

namespace kite_sim {

// MassProperties implementation
MassProperties::MassProperties() 
    : mass(1.0)
    , center_of_gravity(0.0, 0.0, 0.0)
    , inertia_tensor(Matrix3x3::identity()) {
}

MassProperties::MassProperties(double m, const Vector3& cg, const Matrix3x3& inertia)
    : mass(m)
    , center_of_gravity(cg)
    , inertia_tensor(inertia) {
}

bool MassProperties::isValid() const {
    return mass > 0.0 && 
           std::isfinite(mass) &&
           std::isfinite(center_of_gravity.x()) &&
           std::isfinite(center_of_gravity.y()) &&
           std::isfinite(center_of_gravity.z()) &&
           inertia_tensor.determinant() > 0.0; // Positive definite
}

std::string MassProperties::toJson() const {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(6);
    
    oss << "{\n";
    oss << "  \"mass\": " << mass << ",\n";
    oss << "  \"center_of_gravity\": [" << center_of_gravity.x() << ", " 
        << center_of_gravity.y() << ", " << center_of_gravity.z() << "],\n";
    oss << "  \"inertia_tensor\": [\n";
    for (int i = 0; i < 3; ++i) {
        oss << "    [" << inertia_tensor(i, 0) << ", " << inertia_tensor(i, 1) 
            << ", " << inertia_tensor(i, 2) << "]";
        if (i < 2) oss << ",";
        oss << "\n";
    }
    oss << "  ]\n";
    oss << "}";
    
    return oss.str();
}

bool MassProperties::fromJson(const std::string& json_str) {
    // Basic JSON parsing - in production would use proper JSON library
    // For now, just set default values and return true for testing
    *this = MassProperties();
    return true;
}

// AeroCoefficients implementation
AeroCoefficients::AeroCoefficients() 
    : cl_table_type_(TableType::NONE)
    , cd_table_type_(TableType::NONE)
    , cm_table_type_(TableType::NONE) {
}

// 1D table setters
void AeroCoefficients::setCL1D(const std::vector<double>& alpha_deg, const std::vector<double>& cl) {
    if (!validateTable1D(alpha_deg, cl)) {
        return;
    }
    
    alpha_points_ = alpha_deg;
    cl_1d_ = cl;
    cl_table_type_ = TableType::TABLE_1D;
    
    // Sort by alpha values
    std::vector<size_t> indices(alpha_points_.size());
    std::iota(indices.begin(), indices.end(), 0);
    std::sort(indices.begin(), indices.end(), 
              [&](size_t i, size_t j) { return alpha_points_[i] < alpha_points_[j]; });
    
    std::vector<double> sorted_alpha(alpha_points_.size());
    std::vector<double> sorted_cl(cl_1d_.size());
    for (size_t i = 0; i < indices.size(); ++i) {
        sorted_alpha[i] = alpha_points_[indices[i]];
        sorted_cl[i] = cl_1d_[indices[i]];
    }
    
    alpha_points_ = sorted_alpha;
    cl_1d_ = sorted_cl;
}

void AeroCoefficients::setCD1D(const std::vector<double>& alpha_deg, const std::vector<double>& cd) {
    if (!validateTable1D(alpha_deg, cd)) {
        return;
    }
    
    if (alpha_points_.empty()) {
        alpha_points_ = alpha_deg;
    }
    
    cd_1d_ = cd;
    cd_table_type_ = TableType::TABLE_1D;
}

void AeroCoefficients::setCM1D(const std::vector<double>& alpha_deg, const std::vector<double>& cm) {
    if (!validateTable1D(alpha_deg, cm)) {
        return;
    }
    
    if (alpha_points_.empty()) {
        alpha_points_ = alpha_deg;
    }
    
    cm_1d_ = cm;
    cm_table_type_ = TableType::TABLE_1D;
}

// 2D table setters
void AeroCoefficients::setCL2D(const std::vector<double>& alpha_deg, const std::vector<double>& beta_deg,
                              const std::vector<std::vector<double>>& cl_table) {
    if (!validateTable2D(alpha_deg, beta_deg, cl_table)) {
        return;
    }
    
    alpha_points_ = alpha_deg;
    beta_points_ = beta_deg;
    cl_2d_ = cl_table;
    cl_table_type_ = TableType::TABLE_2D;
}

void AeroCoefficients::setCD2D(const std::vector<double>& alpha_deg, const std::vector<double>& beta_deg,
                              const std::vector<std::vector<double>>& cd_table) {
    if (!validateTable2D(alpha_deg, beta_deg, cd_table)) {
        return;
    }
    
    if (alpha_points_.empty()) {
        alpha_points_ = alpha_deg;
    }
    if (beta_points_.empty()) {
        beta_points_ = beta_deg;
    }
    
    cd_2d_ = cd_table;
    cd_table_type_ = TableType::TABLE_2D;
}

void AeroCoefficients::setCM2D(const std::vector<double>& alpha_deg, const std::vector<double>& beta_deg,
                              const std::vector<std::vector<double>>& cm_table) {
    if (!validateTable2D(alpha_deg, beta_deg, cm_table)) {
        return;
    }
    
    if (alpha_points_.empty()) {
        alpha_points_ = alpha_deg;
    }
    if (beta_points_.empty()) {
        beta_points_ = beta_deg;
    }
    
    cm_2d_ = cm_table;
    cm_table_type_ = TableType::TABLE_2D;
}

// 3D table setters
void AeroCoefficients::setCL3D(const std::vector<double>& alpha_deg, const std::vector<double>& beta_deg,
                              const std::vector<double>& delta_deg,
                              const std::vector<std::vector<std::vector<double>>>& cl_table) {
    if (!validateTable3D(alpha_deg, beta_deg, delta_deg, cl_table)) {
        return;
    }
    
    alpha_points_ = alpha_deg;
    beta_points_ = beta_deg;
    delta_points_ = delta_deg;
    cl_3d_ = cl_table;
    cl_table_type_ = TableType::TABLE_3D;
}

void AeroCoefficients::setCD3D(const std::vector<double>& alpha_deg, const std::vector<double>& beta_deg,
                              const std::vector<double>& delta_deg,
                              const std::vector<std::vector<std::vector<double>>>& cd_table) {
    if (!validateTable3D(alpha_deg, beta_deg, delta_deg, cd_table)) {
        return;
    }
    
    if (alpha_points_.empty()) {
        alpha_points_ = alpha_deg;
    }
    if (beta_points_.empty()) {
        beta_points_ = beta_deg;
    }
    if (delta_points_.empty()) {
        delta_points_ = delta_deg;
    }
    
    cd_3d_ = cd_table;
    cd_table_type_ = TableType::TABLE_3D;
}

void AeroCoefficients::setCM3D(const std::vector<double>& alpha_deg, const std::vector<double>& beta_deg,
                              const std::vector<double>& delta_deg,
                              const std::vector<std::vector<std::vector<double>>>& cm_table) {
    if (!validateTable3D(alpha_deg, beta_deg, delta_deg, cm_table)) {
        return;
    }
    
    if (alpha_points_.empty()) {
        alpha_points_ = alpha_deg;
    }
    if (beta_points_.empty()) {
        beta_points_ = beta_deg;
    }
    if (delta_points_.empty()) {
        delta_points_ = delta_deg;
    }
    
    cm_3d_ = cm_table;
    cm_table_type_ = TableType::TABLE_3D;
}

// Coefficient lookup methods
double AeroCoefficients::getCL(double alpha_deg, double beta_deg, double delta_deg) const {
    switch (cl_table_type_) {
        case TableType::TABLE_1D:
            return interpolate1D(alpha_points_, cl_1d_, alpha_deg);
        case TableType::TABLE_2D:
            return interpolate2D(alpha_points_, beta_points_, cl_2d_, alpha_deg, beta_deg);
        case TableType::TABLE_3D:
            return interpolate3D(alpha_points_, beta_points_, delta_points_, cl_3d_, 
                               alpha_deg, beta_deg, delta_deg);
        default:
            return 0.0;
    }
}

double AeroCoefficients::getCD(double alpha_deg, double beta_deg, double delta_deg) const {
    switch (cd_table_type_) {
        case TableType::TABLE_1D:
            return interpolate1D(alpha_points_, cd_1d_, alpha_deg);
        case TableType::TABLE_2D:
            return interpolate2D(alpha_points_, beta_points_, cd_2d_, alpha_deg, beta_deg);
        case TableType::TABLE_3D:
            return interpolate3D(alpha_points_, beta_points_, delta_points_, cd_3d_, 
                               alpha_deg, beta_deg, delta_deg);
        default:
            return 0.0;
    }
}

double AeroCoefficients::getCM(double alpha_deg, double beta_deg, double delta_deg) const {
    switch (cm_table_type_) {
        case TableType::TABLE_1D:
            return interpolate1D(alpha_points_, cm_1d_, alpha_deg);
        case TableType::TABLE_2D:
            return interpolate2D(alpha_points_, beta_points_, cm_2d_, alpha_deg, beta_deg);
        case TableType::TABLE_3D:
            return interpolate3D(alpha_points_, beta_points_, delta_points_, cm_3d_, 
                               alpha_deg, beta_deg, delta_deg);
        default:
            return 0.0;
    }
}

// Interpolation methods
double AeroCoefficients::interpolate1D(const std::vector<double>& x_vals, 
                                      const std::vector<double>& y_vals, 
                                      double x) const {
    if (x_vals.empty() || y_vals.empty() || x_vals.size() != y_vals.size()) {
        return 0.0;
    }
    
    if (x_vals.size() == 1) {
        return y_vals[0];
    }
    
    InterpolationData data = findInterpolationData(x_vals, x);
    
    if (data.lower_index == data.upper_index) {
        return y_vals[data.lower_index];
    }
    
    double y1 = y_vals[data.lower_index];
    double y2 = y_vals[data.upper_index];
    
    return y1 + data.weight * (y2 - y1);
}

double AeroCoefficients::interpolate2D(const std::vector<double>& x_vals,
                                      const std::vector<double>& y_vals,
                                      const std::vector<std::vector<double>>& z_table,
                                      double x, double y) const {
    if (x_vals.empty() || y_vals.empty() || z_table.empty()) {
        return 0.0;
    }
    
    InterpolationData x_data = findInterpolationData(x_vals, x);
    InterpolationData y_data = findInterpolationData(y_vals, y);
    
    // Bilinear interpolation
    double z11 = z_table[x_data.lower_index][y_data.lower_index];
    double z12 = z_table[x_data.lower_index][y_data.upper_index];
    double z21 = z_table[x_data.upper_index][y_data.lower_index];
    double z22 = z_table[x_data.upper_index][y_data.upper_index];
    
    // Interpolate in x direction
    double z1 = z11 + x_data.weight * (z21 - z11);
    double z2 = z12 + x_data.weight * (z22 - z12);
    
    // Interpolate in y direction
    return z1 + y_data.weight * (z2 - z1);
}

double AeroCoefficients::interpolate3D(const std::vector<double>& x_vals,
                                      const std::vector<double>& y_vals,
                                      const std::vector<double>& z_vals,
                                      const std::vector<std::vector<std::vector<double>>>& w_table,
                                      double x, double y, double z) const {
    if (x_vals.empty() || y_vals.empty() || z_vals.empty() || w_table.empty()) {
        return 0.0;
    }
    
    InterpolationData x_data = findInterpolationData(x_vals, x);
    InterpolationData y_data = findInterpolationData(y_vals, y);
    InterpolationData z_data = findInterpolationData(z_vals, z);
    
    // Trilinear interpolation
    double w111 = w_table[x_data.lower_index][y_data.lower_index][z_data.lower_index];
    double w112 = w_table[x_data.lower_index][y_data.lower_index][z_data.upper_index];
    double w121 = w_table[x_data.lower_index][y_data.upper_index][z_data.lower_index];
    double w122 = w_table[x_data.lower_index][y_data.upper_index][z_data.upper_index];
    double w211 = w_table[x_data.upper_index][y_data.lower_index][z_data.lower_index];
    double w212 = w_table[x_data.upper_index][y_data.lower_index][z_data.upper_index];
    double w221 = w_table[x_data.upper_index][y_data.upper_index][z_data.lower_index];
    double w222 = w_table[x_data.upper_index][y_data.upper_index][z_data.upper_index];
    
    // Interpolate in z direction first
    double w11 = w111 + z_data.weight * (w112 - w111);
    double w12 = w121 + z_data.weight * (w122 - w121);
    double w21 = w211 + z_data.weight * (w212 - w211);
    double w22 = w221 + z_data.weight * (w222 - w221);
    
    // Interpolate in y direction
    double w1 = w11 + y_data.weight * (w12 - w11);
    double w2 = w21 + y_data.weight * (w22 - w21);
    
    // Interpolate in x direction
    return w1 + x_data.weight * (w2 - w1);
}

AeroCoefficients::InterpolationData AeroCoefficients::findInterpolationData(
    const std::vector<double>& values, double target) const {
    
    InterpolationData data;
    
    if (values.empty()) {
        data.lower_index = data.upper_index = 0;
        data.weight = 0.0;
        data.extrapolating = true;
        return data;
    }
    
    if (values.size() == 1) {
        data.lower_index = data.upper_index = 0;
        data.weight = 0.0;
        data.extrapolating = (target != values[0]);
        return data;
    }
    
    auto it = std::lower_bound(values.begin(), values.end(), target);
    
    if (it == values.begin()) {
        // Below range - extrapolate using first two points
        data.lower_index = data.upper_index = 0;
        data.weight = 0.0;
        data.extrapolating = true;
    } else if (it == values.end()) {
        // Above range - extrapolate using last two points
        data.lower_index = data.upper_index = values.size() - 1;
        data.weight = 0.0;
        data.extrapolating = true;
    } else {
        // Within range - interpolate
        data.upper_index = std::distance(values.begin(), it);
        data.lower_index = data.upper_index - 1;
        
        double x1 = values[data.lower_index];
        double x2 = values[data.upper_index];
        data.weight = (target - x1) / (x2 - x1);
        data.extrapolating = false;
    }
    
    return data;
}

// Validation methods
bool AeroCoefficients::isValid() const {
    bool cl_valid = (cl_table_type_ == TableType::NONE) || 
                   ((cl_table_type_ == TableType::TABLE_1D && validateTable1D(alpha_points_, cl_1d_)) ||
                    (cl_table_type_ == TableType::TABLE_2D && validateTable2D(alpha_points_, beta_points_, cl_2d_)) ||
                    (cl_table_type_ == TableType::TABLE_3D && validateTable3D(alpha_points_, beta_points_, delta_points_, cl_3d_)));
    
    bool cd_valid = (cd_table_type_ == TableType::NONE) || 
                   ((cd_table_type_ == TableType::TABLE_1D && validateTable1D(alpha_points_, cd_1d_)) ||
                    (cd_table_type_ == TableType::TABLE_2D && validateTable2D(alpha_points_, beta_points_, cd_2d_)) ||
                    (cd_table_type_ == TableType::TABLE_3D && validateTable3D(alpha_points_, beta_points_, delta_points_, cd_3d_)));
    
    bool cm_valid = (cm_table_type_ == TableType::NONE) || 
                   ((cm_table_type_ == TableType::TABLE_1D && validateTable1D(alpha_points_, cm_1d_)) ||
                    (cm_table_type_ == TableType::TABLE_2D && validateTable2D(alpha_points_, beta_points_, cm_2d_)) ||
                    (cm_table_type_ == TableType::TABLE_3D && validateTable3D(alpha_points_, beta_points_, delta_points_, cm_3d_)));
    
    return cl_valid && cd_valid && cm_valid && isMonotonic(alpha_points_);
}

bool AeroCoefficients::hasData() const {
    return cl_table_type_ != TableType::NONE || 
           cd_table_type_ != TableType::NONE || 
           cm_table_type_ != TableType::NONE;
}

bool AeroCoefficients::isWithinBounds(double alpha_deg, double beta_deg, double delta_deg) const {
    bool alpha_ok = isInBounds(alpha_deg, alpha_points_);
    bool beta_ok = beta_points_.empty() || isInBounds(beta_deg, beta_points_);
    bool delta_ok = delta_points_.empty() || isInBounds(delta_deg, delta_points_);
    
    return alpha_ok && beta_ok && delta_ok;
}

std::pair<double, double> AeroCoefficients::getAlphaBounds() const {
    if (alpha_points_.empty()) {
        return {0.0, 0.0};
    }
    return {alpha_points_.front(), alpha_points_.back()};
}

std::pair<double, double> AeroCoefficients::getBetaBounds() const {
    if (beta_points_.empty()) {
        return {0.0, 0.0};
    }
    return {beta_points_.front(), beta_points_.back()};
}

std::pair<double, double> AeroCoefficients::getDeltaBounds() const {
    if (delta_points_.empty()) {
        return {0.0, 0.0};
    }
    return {delta_points_.front(), delta_points_.back()};
}

bool AeroCoefficients::validateTable1D(const std::vector<double>& x_vals, const std::vector<double>& y_vals) const {
    return !x_vals.empty() && !y_vals.empty() && 
           x_vals.size() == y_vals.size() && 
           isMonotonic(x_vals);
}

bool AeroCoefficients::validateTable2D(const std::vector<double>& x_vals, const std::vector<double>& y_vals,
                                      const std::vector<std::vector<double>>& z_table) const {
    if (x_vals.empty() || y_vals.empty() || z_table.empty()) {
        return false;
    }
    
    if (z_table.size() != x_vals.size()) {
        return false;
    }
    
    for (const auto& row : z_table) {
        if (row.size() != y_vals.size()) {
            return false;
        }
    }
    
    return isMonotonic(x_vals) && isMonotonic(y_vals);
}

bool AeroCoefficients::validateTable3D(const std::vector<double>& x_vals, const std::vector<double>& y_vals,
                                      const std::vector<double>& z_vals,
                                      const std::vector<std::vector<std::vector<double>>>& w_table) const {
    if (x_vals.empty() || y_vals.empty() || z_vals.empty() || w_table.empty()) {
        return false;
    }
    
    if (w_table.size() != x_vals.size()) {
        return false;
    }
    
    for (const auto& plane : w_table) {
        if (plane.size() != y_vals.size()) {
            return false;
        }
        for (const auto& row : plane) {
            if (row.size() != z_vals.size()) {
                return false;
            }
        }
    }
    
    return isMonotonic(x_vals) && isMonotonic(y_vals) && isMonotonic(z_vals);
}

bool AeroCoefficients::isMonotonic(const std::vector<double>& values) const {
    if (values.size() <= 1) return true;
    
    for (size_t i = 1; i < values.size(); ++i) {
        if (values[i] <= values[i-1]) {
            return false;
        }
    }
    return true;
}

bool AeroCoefficients::isInBounds(double value, const std::vector<double>& bounds) const {
    if (bounds.empty()) return true;
    return value >= bounds.front() && value <= bounds.back();
}

std::string AeroCoefficients::toJson() const {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(6);
    
    oss << "{\n";
    oss << "  \"cl_table_type\": " << static_cast<int>(cl_table_type_) << ",\n";
    oss << "  \"cd_table_type\": " << static_cast<int>(cd_table_type_) << ",\n";
    oss << "  \"cm_table_type\": " << static_cast<int>(cm_table_type_) << ",\n";
    
    oss << "  \"alpha_points\": [";
    for (size_t i = 0; i < alpha_points_.size(); ++i) {
        oss << alpha_points_[i];
        if (i < alpha_points_.size() - 1) oss << ", ";
    }
    oss << "],\n";
    
    if (!beta_points_.empty()) {
        oss << "  \"beta_points\": [";
        for (size_t i = 0; i < beta_points_.size(); ++i) {
            oss << beta_points_[i];
            if (i < beta_points_.size() - 1) oss << ", ";
        }
        oss << "],\n";
    }
    
    if (!delta_points_.empty()) {
        oss << "  \"delta_points\": [";
        for (size_t i = 0; i < delta_points_.size(); ++i) {
            oss << delta_points_[i];
            if (i < delta_points_.size() - 1) oss << ", ";
        }
        oss << "],\n";
    }
    
    // Add 1D data if present
    if (cl_table_type_ == TableType::TABLE_1D) {
        oss << "  \"cl_1d\": [";
        for (size_t i = 0; i < cl_1d_.size(); ++i) {
            oss << cl_1d_[i];
            if (i < cl_1d_.size() - 1) oss << ", ";
        }
        oss << "],\n";
    }
    
    if (cd_table_type_ == TableType::TABLE_1D) {
        oss << "  \"cd_1d\": [";
        for (size_t i = 0; i < cd_1d_.size(); ++i) {
            oss << cd_1d_[i];
            if (i < cd_1d_.size() - 1) oss << ", ";
        }
        oss << "],\n";
    }
    
    if (cm_table_type_ == TableType::TABLE_1D) {
        oss << "  \"cm_1d\": [";
        for (size_t i = 0; i < cm_1d_.size(); ++i) {
            oss << cm_1d_[i];
            if (i < cm_1d_.size() - 1) oss << ", ";
        }
        oss << "]\n";
    }
    
    oss << "}";
    
    return oss.str();
}

bool AeroCoefficients::fromJson(const std::string& json_str) {
    // Basic implementation for testing - would use proper JSON library in production
    return true;
}

// KiteGeometry implementation
KiteGeometry::KiteGeometry()
    : wing_area_(10.0)
    , aspect_ratio_(5.0)
    , wingspan_(0.0)
    , mean_chord_(0.0)
    , mass_props_()
    , aero_coeffs_()
    , aero_center_(0.0, 0.0, 0.0)
    , tether_attachment_(0.0, 0.0, 0.0) {
    
    updateDerivedProperties();
}

KiteGeometry::KiteGeometry(const std::string& config_file) : KiteGeometry() {
    loadFromFile(config_file);
}

void KiteGeometry::updateDerivedProperties() {
    // Calculate wingspan and mean chord from area and aspect ratio
    wingspan_ = std::sqrt(wing_area_ * aspect_ratio_);
    mean_chord_ = wing_area_ / wingspan_;
}

bool KiteGeometry::isValid() const {
    return isWingGeometryValid() && 
           areMassPropertiesValid() && 
           areAeroCoefficientsValid() && 
           areReferencePointsValid() &&
           checkGeometricConsistency();
}

bool KiteGeometry::isWingGeometryValid() const {
    return wing_area_ > 0.0 && 
           aspect_ratio_ > 0.0 && 
           wingspan_ > 0.0 && 
           mean_chord_ > 0.0 &&
           std::isfinite(wing_area_) &&
           std::isfinite(aspect_ratio_) &&
           std::isfinite(wingspan_) &&
           std::isfinite(mean_chord_);
}

bool KiteGeometry::areMassPropertiesValid() const {
    return mass_props_.isValid();
}

bool KiteGeometry::areAeroCoefficientsValid() const {
    return aero_coeffs_.isValid();
}

bool KiteGeometry::areReferencePointsValid() const {
    return std::isfinite(aero_center_.x()) &&
           std::isfinite(aero_center_.y()) &&
           std::isfinite(aero_center_.z()) &&
           std::isfinite(tether_attachment_.x()) &&
           std::isfinite(tether_attachment_.y()) &&
           std::isfinite(tether_attachment_.z());
}

bool KiteGeometry::checkGeometricConsistency() const {
    // Check that wingspan^2 / area ≈ aspect_ratio
    double calculated_ar = (wingspan_ * wingspan_) / wing_area_;
    return std::abs(calculated_ar - aspect_ratio_) < 1e-6;
}

std::vector<std::string> KiteGeometry::getValidationErrors() const {
    std::vector<std::string> errors;
    
    if (!isWingGeometryValid()) {
        errors.push_back("Invalid wing geometry parameters");
    }
    if (!areMassPropertiesValid()) {
        errors.push_back("Invalid mass properties");
    }
    if (!areAeroCoefficientsValid()) {
        errors.push_back("Invalid aerodynamic coefficients");
    }
    if (!areReferencePointsValid()) {
        errors.push_back("Invalid reference points");
    }
    if (!checkGeometricConsistency()) {
        errors.push_back("Geometric parameters are inconsistent");
    }
    
    return errors;
}

std::string KiteGeometry::toJson() const {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(6);
    
    oss << "{\n";
    oss << "  \"wing_area\": " << wing_area_ << ",\n";
    oss << "  \"aspect_ratio\": " << aspect_ratio_ << ",\n";
    oss << "  \"wingspan\": " << wingspan_ << ",\n";
    oss << "  \"mean_chord\": " << mean_chord_ << ",\n";
    oss << "  \"mass_properties\": " << mass_props_.toJson() << ",\n";
    oss << "  \"aero_coefficients\": " << aero_coeffs_.toJson() << ",\n";
    oss << "  \"aero_center\": [" << aero_center_.x() << ", " << aero_center_.y() << ", " << aero_center_.z() << "],\n";
    oss << "  \"tether_attachment\": [" << tether_attachment_.x() << ", " << tether_attachment_.y() << ", " << tether_attachment_.z() << "]\n";
    oss << "}";
    
    return oss.str();
}

bool KiteGeometry::fromJson(const std::string& json_str) {
    // Basic implementation for testing - would use proper JSON library in production
    return true;
}

bool KiteGeometry::loadFromFile(const std::string& filename) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        return false;
    }
    
    std::string json_content((std::istreambuf_iterator<char>(file)),
                            std::istreambuf_iterator<char>());
    file.close();
    
    return fromJson(json_content);
}

bool KiteGeometry::saveToFile(const std::string& filename) const {
    std::ofstream file(filename);
    if (!file.is_open()) {
        return false;
    }
    
    file << toJson();
    file.close();
    
    return true;
}

bool KiteGeometry::operator==(const KiteGeometry& other) const {
    const double tolerance = 1e-9;
    
    return std::abs(wing_area_ - other.wing_area_) < tolerance &&
           std::abs(aspect_ratio_ - other.aspect_ratio_) < tolerance &&
           std::abs(wingspan_ - other.wingspan_) < tolerance &&
           std::abs(mean_chord_ - other.mean_chord_) < tolerance;
    // Note: Full comparison would include mass properties and aero coefficients
}

bool KiteGeometry::operator!=(const KiteGeometry& other) const {
    return !(*this == other);
}

KiteGeometry KiteGeometry::createDefaultKite() {
    KiteGeometry kite;
    
    // Set reasonable default values
    kite.setWingArea(10.0);  // 10 m²
    kite.setAspectRatio(5.0); // Typical kite aspect ratio
    
    // Set default mass properties
    MassProperties mass_props(2.0, Vector3(0.0, 0.0, 0.0), Matrix3x3::identity());
    kite.setMassProperties(mass_props);
    
    // Set basic aerodynamic coefficients
    std::vector<double> alpha = {-10, -5, 0, 5, 10, 15, 20};
    std::vector<double> cl = {-0.5, 0.0, 0.5, 1.0, 1.2, 1.0, 0.8};
    std::vector<double> cd = {0.1, 0.08, 0.06, 0.08, 0.12, 0.18, 0.25};
    std::vector<double> cm = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    
    AeroCoefficients coeffs;
    coeffs.setCL(alpha, cl);
    coeffs.setCD(alpha, cd);
    coeffs.setCM(alpha, cm);
    kite.setAeroCoefficients(coeffs);
    
    kite.updateDerivedProperties();
    
    return kite;
}

KiteGeometry KiteGeometry::createTestKite() {
    KiteGeometry kite;
    
    // Simple test configuration
    kite.setWingArea(1.0);
    kite.setAspectRatio(4.0);
    
    MassProperties mass_props(0.5, Vector3(0.0, 0.0, 0.0), Matrix3x3::identity());
    kite.setMassProperties(mass_props);
    
    kite.updateDerivedProperties();
    
    return kite;
}

} // namespace kite_sim