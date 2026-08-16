#include <custom_nav/matrix.h>
#include <custom_nav/mjetable.h>
#include <unordered_map>
#include <cmath>

namespace bezier {

static std::unordered_map<std::size_t, mat::Matrix<double>> basis_cache;

double normalizeAngle(double);
const mat::Matrix<double>& getBasis(std::size_t);
mat::Matrix<double> optimalAlphaBezierCP(
    const std::array<double, 3>&, const std::array<double, 3>&, const lt::ConstraintTable&);
mat::Matrix<double> getSymmetricQuinticCP(
        const std::array<double, 3>&, const std::array<double, 3>&);
mat::Matrix<double> discretizeBezierCurve(const mat::Matrix<double>&, int d = 100);
double computeMaxCurvatureNumeric(const mat::Matrix<double>&);
std::pair<bool, double> getKcPosition(const mat::Matrix<double>& curvePoints, double kc = 3.0);
// std::pair<mat::Matrix<double>, std::vector<double>> forwardPass(node::Node, std::vector<node::Node>&, double = 3.0);
// std::pair<std::vector<std::array<double, 3>>, std::vector<bool>>
// extractWaypoints(int, const std::vector<Node>&);
// Structure to cleanly return the 3 output values
struct ProcessResult {
    mat::Matrix<double> smoothenPath;
    std::vector<std::array<double, 2>> cut_points;
    double kmaxPath;
};

ProcessResult processWayPointsF(
    const std::vector<std::array<double, 3>>&, const lt::ConstraintTable&,
    const std::vector<bool>&, // Unused in loop, preserved for signature match
    double);

}
