#include <custom_nav/matrix.h>
#include <custom_nav/bezier.h>
#include <unordered_map>
#include <cmath>

namespace bezier {

/* Normalize angle to [-pi, pi] */
double normalizeAngle(double angle) {
    angle = std::fmod(angle + mat::PI, 2.0 * mat::PI);
    if (angle < 0) {
        angle += 2.0 * mat::PI;
    }
    return angle - mat::PI;
}

const mat::Matrix<double>& getBasis(std::size_t d) {
    auto it = basis_cache.find(d);
    if (it != basis_cache.end()) return it->second;

    mat::Matrix<double> basis(d, 6);
    static constexpr std::array<double, 6> binom_coeff = {1, 5, 10, 10, 5, 1};
    const mat::Matrix<double> t = mat::linspace(0.0, 1.0, d);

    for (std::size_t r = 0; r < d; r++) {
        for (std::size_t c = 0; c < 6; c++) {
            basis(r, c) = binom_coeff[c] * std::pow(t(0, r), c) * std::pow(1.0 - t(0, r), 5-c);
        }

    auto res = basis_cache.emplace(d, std::move(basis));

    return res.first->second;
}

mat::Matrix<double> optimalAlphaBezierCP(
    const std::array<double, 3>& init_pose, 
    const std::array<double, 3>& end_pose, 
    const lt::ConstraintTable& table) 
{
    // Poses: {x, y, theta}
    double dx = end_pose[0] - init_pose[0];
    double dy = end_pose[1] - init_pose[1];
    double chord_angle = std::atan2(dy, dx);

    double theta0 = normalizeAngle(init_pose[2] - chord_angle);
    double theta5 = normalizeAngle(end_pose[2] - chord_angle);

    if (std::abs(theta0) < 1e-2) {
        theta0 = 0.0;
    }

    std::vector<double> alphas = table.getAlphas(theta0, theta5);
    double chordLength = std::hypot(dx, dy);

    double x0 = init_pose[0], y0 = init_pose[1], th0 = init_pose[2];
    double x5 = end_pose[0],  y5 = end_pose[1],  th5 = end_pose[2];

    double delta0 = alphas[0] * chordLength;
    double eps0   = alphas[1] * chordLength;
    double delta5 = alphas[2] * chordLength;
    double eps5   = alphas[3] * chordLength;

    double x1 = x0 + delta0 * std::cos(th0);
    double y1 = y0 + delta0 * std::sin(th0);
    double x2 = x1 + eps0 * std::cos(th0);
    double y2 = y1 + eps0 * std::sin(th0);

    double x4 = x5 - delta5 * std::cos(th5);
    double y4 = y5 - delta5 * std::sin(th5);
    double x3 = x4 - eps5 * std::cos(th5);
    double y3 = y4 - eps5 * std::sin(th5);

    // Populate a 6x2 matrix with the control points
    mat::Matrix<double> CP(6, 2);
    
    CP(0, 0) = x0; CP(0, 1) = y0;
    CP(1, 0) = x1; CP(1, 1) = y1;
    CP(2, 0) = x2; CP(2, 1) = y2;
    CP(3, 0) = x3; CP(3, 1) = y3;
    CP(4, 0) = x4; CP(4, 1) = y4;
    CP(5, 0) = x5; CP(5, 1) = y5;

    return CP;
}

mat::Matrix<double> getSymmetricQuinticCP(const std::array<double, 3>& init_pose, 
                                          const std::array<double, 3>& end_pose) {
    double x0 = init_pose[0], y0 = init_pose[1], th0 = init_pose[2];
    double x5 = end_pose[0],  y5 = end_pose[1],  th5 = end_pose[2];

    double dx = x5 - x0;
    double dy = y5 - y0;
    double L = std::hypot(dx, dy);
    double delta = L / 5.0;

    double x1 = x0 + delta * std::cos(th0);
    double y1 = y0 + delta * std::sin(th0);
    double x2 = x1 + delta * std::cos(th0);
    double y2 = y1 + delta * std::sin(th0);

    double x4 = x5 - delta * std::cos(th5);
    double y4 = y5 - delta * std::sin(th5);
    double x3 = x4 - delta * std::cos(th5);
    double y3 = y4 - delta * std::sin(th5);

    mat::Matrix<double> CP(6, 2);
    CP(0, 0) = x0; CP(0, 1) = y0;
    CP(1, 0) = x1; CP(1, 1) = y1;
    CP(2, 0) = x2; CP(2, 1) = y2;
    CP(3, 0) = x3; CP(3, 1) = y3;
    CP(4, 0) = x4; CP(4, 1) = y4;
    CP(5, 0) = x5; CP(5, 1) = y5;

    return CP;
}

mat::Matrix<double> discretizeBezierCurve(const mat::Matrix<double> &control_points, int d) {
    return getBasis(d) * control_points;
}

double computeMaxCurvatureNumeric(const mat::Matrix<double>& curvePoints) {
    assert(curvePoints.cols() >= 2 && curvePoints().rows() >=3);

    std::size_t n = curvePoints.rows();
    std::vector<double> xdt(n), ydt(n), xddt(n), yddt(n);

    // Lambda function to compute gradients
    auto computeGrad = [&] () {
        xdt[0] = curvePoints(1, 0) - curvePoints(0, 0);
        xdt[n-1] = curvePoints(n-1, 0) - curvePoints(n-2, 0);
        ydt[0] = curvePoints(1, 1) - curvePoints(0, 1);
        ydt[n-1] = curvePoints(n-1, 1) - curvePoints(n-2, 1);
        for (int i = 1; i < n-1; i++) {
            xdt[i] = (curvePoints(i+1, 0) - curvePoints(i-1, 0)) / 2.0;
            ydt[i] = (curvePoints(i+1, 1) - curvePoints(i-1, 1)) / 2.0;
        }

        xddt[0] = xdt[1] - xdt[0];
        xddt[n-1] = xdt[n-1] - xdt[n-2];
        yddt[0] = ydt[1] - ydt[0];
        yddt[n-1] = ydt[n-1] - ydt[n-2];
        for (int i = 1; i < n-1; i++) {
            xddt[i] = (xdt[i+1] - xdt[i-1]) / 2.0;
            yddt[i] = (ydt[i+1] - ydt[i-1]) / 2.0;
        }
    };

    computeGrad();

    // We have computed the gradients
    // Now we will compute the maximum curvature
    double max_cur = 0.0;
    for (std::size_t i = 0; i < n; i++) {
        double num = std::abs(xdt[i] * yddt[i] - ydt[i] * xddt[i]);
        double den = std::pow(xdt[i] * xdt[i] + ydt[i] * ydt[i], 1.5) + 1e-10;
        double cur = num / den;

        if (cur > max_cur) max_cur = cur;
    }

    return max_cur;
}

std::pair<bool, double> getKcPosition(const mat::Matrix<double>& curvePoints, double kc) {
    assert(curvePoints.cols() >= 2 && curvePoints().rows() >=3);

    std::size_t n = curvePoints.rows();
    std::vector<double> xdt(n), ydt(n), xddt(n), yddt(n);

    // Lambda function to compute gradients
    auto computeGrad = [&] () {
        xdt[0] = curvePoints(1, 0) - curvePoints(0, 0);
        xdt[n-1] = curvePoints(n-1, 0) - curvePoints(n-2, 0);
        ydt[0] = curvePoints(1, 1) - curvePoints(0, 1);
        ydt[n-1] = curvePoints(n-1, 1) - curvePoints(n-2, 1);
        for (int i = 1; i < n-1; i++) {
            xdt[i] = (curvePoints(i+1, 0) - curvePoints(i-1, 0)) / 2.0;
            ydt[i] = (curvePoints(i+1, 1) - curvePoints(i-1, 1)) / 2.0;
        }

        xddt[0] = xdt[1] - xdt[0];
        xddt[n-1] = xdt[n-1] - xdt[n-2];
        yddt[0] = ydt[1] - ydt[0];
        yddt[n-1] = ydt[n-1] - ydt[n-2];
        for (int i = 1; i < n-1; i++) {
            xddt[i] = (xdt[i+1] - xdt[i-1]) / 2.0;
            yddt[i] = (ydt[i+1] - ydt[i-1]) / 2.0;
        }
    };

    computeGrad();

    for (std::size_t i = 0; i < n; i++) {
        double num = std::abs(xdt[i] * yddt[i] - ydt[i] * xddt[i]);
        double den = std::pow(xdt[i] * xdt[i] + ydt[i] * ydt[i], 1.5) + 1e-10;
        double cur = num / den;

        if (cur > kc) {
            double t = static_cast<double>(i) / static_cast<double>(n);
            t = (t > 0.05) ? 0.05 : 0.02 - t;
            return std::pair<bool, double>(false, t);
        }
    }

    return std::pair<bool, double>(true, 0.0);
}

// std::pair<mat::Matrix<double>, std::vector<double>> forwardPass(node::Node curr_node, std::vector<node::Node>& vertices, double kmax = 3.0) {
//     std::vector<double> cutPoints, smoothPath;
//     std::size_t curr_idx = curr_node.index;
//     std::size_t par_idx = vectices[curr_node.parent].index;
// 
//     while (i != 0) {
//         std::array<double, 3>& init_pose = vertices[i].pose;
//         std::array<double, 3>& next_pose = vertices[j].pose;
// 
//         mat::Matrix<double> controlPoints = getSymmetricQuinticCP(init_pose, next_pose);
//         mat::Matrix<double> curvePoints = discretizeBezierCurve(contorlPoints);
// 
//         std::par<bool, double> check = getKcPosition(curvePoints);
//         if (feasible) i = j;
//         else {
//             // TODO
//         }
// 
//         smoothPath.addRows(curvePoints);
// 
//         if (j!= 0) j = vertices[j].parent;
//     }
// 
//     // TODO
//     return std::pair<>(smoothPath, cutPoints);
// }

// Returns a pair: {wayPoints, changedList}
// std::pair<std::vector<std::array<double, 3>>, std::vector<bool>>
// extractWaypoints(int currentNodeIndex, const std::vector<Node>& vertices) {
//     std::vector<std::array<double, 3>> wayPoints;
//     std::vector<bool> changedList;
// 
//     int currIdx = currentNodeIndex;
// 
//     // Loop until we hit the root node (parent == -1)
//     while (vertices[currIdx].parent != -1) {
//         const Node& currentNode = vertices[currIdx];
// 
//         changedList.push_back(currentNode.changed);
//         wayPoints.push_back(currentNode.pose);
// 
//         // If this node was reached via a two-stage maneuver, insert intermediate pose m
//         // TODO: Change the via_m type
//         if (currentNode.via_m.has_value()) {
//             const auto& m = currentNode.via_m.value(); // Extract mx, my, mth, _, _
//             wayPoints.push_back({m[0], m[1], m[2]});
//             changedList.push_back(false); // m is not a "changed" waypoint
//         }
// 
//         currIdx = currentNode.parent;
//     }
// 
//     // Append the starting node's pose
//     wayPoints.push_back(vertices[0].pose);
// 
//     return {wayPoints, changedList};
// }

ProcessResult processWayPointsF(
    const std::vector<std::array<double, 3>>& wayPoints,
    const ConstraintTable& curvatureTable,
    const std::vector<bool>& changedList, // Unused in loop, preserved for signature match
    double kc) 
{
    std::vector<std::array<double, 2>> cut_points;
    std::vector<double> kmaxList;
    
    // Optimal path storage using Matrix
    mat::Matrix<double> smoothenPath(0, 2);
    smoothenPath.reserve(wayPoints.size() * 100 * 2); // Pre-allocate to prevent reallocation

    // Helper lambda for angle normalization
    auto normalizeAngle = [](double angle) {
        angle = std::fmod(angle + mat::PI, 2.0 * mat::PI);
        if (angle < 0) {
            angle += 2.0 * mat::PI;
        }
        return angle - mat::PI;
    };

    std::size_t i = 0;
    while (i < wayPoints.size() - 1) {
        std::array<double, 3> init_pose = wayPoints[i];
        std::array<double, 3> end_pose = wayPoints[i + 1];

        double dx = end_pose[0] - init_pose[0];
        double dy = end_pose[1] - init_pose[1];
        
        // chordLength uses [:2] of the pose, which corresponds to dx and dy
        double chordLength = std::hypot(dx, dy); 
        double chord_angle = std::atan2(dy, dx);

        double theta5 = normalizeAngle(end_pose[2] - chord_angle);
        double theta0 = normalizeAngle(init_pose[2] - chord_angle);

        if (std::abs(theta0) < 1e-2) {
            theta0 = 0.0;
        }

        // Extract kUnit from the curvature table
        auto curveData = curvatureTable.getcurvature(theta0, theta5);
        double kUnit = curveData.first; 

        double kmax = kUnit / chordLength;
        kmaxList.push_back(kmax);

        // Check for curvature violations
        if (kmax > kc + 1e-5) {
            std::cout << "poses where violation occured: " 
                      << "[" << init_pose[0] << ", " << init_pose[1] << ", " << init_pose[2] << "] "
                      << "[" << end_pose[0] << ", " << end_pose[1] << ", " << end_pose[2] << "]\n";
            std::cout << "the max curvature hit is: " << kmax << '\n';
            
            cut_points.push_back({init_pose[0], init_pose[1]});
            cut_points.push_back({end_pose[0], end_pose[1]});
            kmaxList.push_back(kmax); // Appends twice as per Python logic
        }

        // Generate and discretize curve points
        mat::Matrix<double> controlPoints = optimalAlphaBezierCP(init_pose, end_pose, curvatureTable);
        mat::Matrix<double> curvePoints = discretizeBezierCurve(controlPoints);

        // Move the curvePoints into the smoothenPath matrix dynamically
        smoothenPath.appendRowWise(std::move(curvePoints));
        
        i = i + 1;
    }

    // Extract the maximum kmax value
    double kmaxPath = 0.0;
    if (!kmaxList.empty()) {
        kmaxPath = *std::max_element(kmaxList.begin(), kmaxList.end());
    }

    return {std::move(smoothenPath), std::move(cut_points), kmaxPath};
}

}
