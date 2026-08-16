#ifndef MJETABLE_H
#define MJETABLE_H

#include <custom_nav/matrix.h>
#include <string>

namespace lt {

class ConstraintTable {
private:
    mat::Matrix<double> theta_rad_, kappa_, singular_, arclength_;
    std::vector<mat::Matrix<double>> alphas_;

    std::size_t n_ = 0;
    double dtheta_;

    int getIndex(double theta) const;

public:
    ConstraintTable() = default;

    ConstraintTable(
        std::string& theta_file, std::string& kappa_file, std::string& singular_file,
        std::string& arclength_file, std::vector<std::string>& alphas_files);

    void buildTable(
        std::string& theta_file, std::string& kappa_file, std::string& singular_file,
        std::string& arclength_file, std::vector<std::string>& alphas_files);


    std::pair<double, double> getcurvature(double theta0, double theta5) const;
    double getarclength(double theta0, double theta5) const;
    std::vector<double> getAlphas(double theta0, double theta5) const;
};

bool readNumpyArray(const std::string& filename, mat::Matrix<double>& result);

} // namespace lt

#endif
