#include <custom_nav/mjetable.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <cstdint>
#include <numeric>
#include <utility>

namespace lt {

int ConstraintTable::getIndex(double theta) const {
    int idx = static_cast<int>(std::floor((theta - theta_rad_(0, 0)) / dtheta_));
    return std::max(0, std::min(idx, n_ - 1));
}

ConstraintTable::ConstraintTable(
    std::string& theta_file, std::string& kappa_file, std::string& singular_file,
    std::string& arclength_file, std::vector<std::string>& alphas_files)
{
    buildTable(theta_file, kappa_file, singular_file, arclength_file, alphas_files);
}

void ConstraintTable::buildTable(
    std::string& theta_file, std::string& kappa_file, std::string& singular_file,
    std::string& arclength_file, std::vector<std::string>& alphas_files
) {
    bool loaded_files = true;

    loaded_files &= readNumpyArray(theta_file, theta_rad_);
    loaded_files &= readNumpyArray(kappa_file, kappa_);
    loaded_files &= readNumpyArray(arclength_file, arclength_);
    loaded_files &= readNumpyArray(singular_file, singular_);

    alphas_.resize(alphas_files.size());
    for (int i = 0; i < alphas_files.size(); i++) {
        loaded_files &= readNumpyArray(alphas_files[i], alphas_[i]);
    }
    if (!loaded_files) {
        std::cerr << "Unable to initailize";
        return;
    }

    n_ = this->theta_rad_.rows() * this->theta_rad_.cols();
    dtheta_ = theta_rad_.data()[1] - theta_rad_.data()[0];
}

std::pair<double, double> ConstraintTable::getcurvature(double theta0, double theta5) const {
    int i = getIndex(theta0);
    int j = getIndex(theta5);
    return {kappa_(j, i), singular_(j, i)};
}

double ConstraintTable::getarclength(double theta0, double theta5) const {
    int i = getIndex(theta0);
    int j = getIndex(theta5);
    return arclength_(j, i);
}

std::vector<double> ConstraintTable::getAlphas(double theta0, double theta5) const {
    int i = getIndex(theta0);
    int j = getIndex(theta5);
    
    std::vector<double> extracted_alphas;
    extracted_alphas.reserve(alphas_.size());
    
    // return [alpha[j, i] for alpha in self.alphas]
    for (const auto& alpha_mat : alphas_) {
        extracted_alphas.push_back(alpha_mat(j, i));
    }
    
    return extracted_alphas;
}

/**
 * Read a numpy array from a binary file
 */
bool readNumpyArray(const std::string& filename, mat::Matrix<double>& result)
{
    std::ifstream file(filename, std::ios::binary);
    if (!file) {
        std::cerr << "Error: Cannot open '" << filename << "'\n";
        return false;
    }

    // Determine file size
    file.seekg(0, std::ios::end);
    const std::streamoff fileSize = file.tellg();
    file.seekg(0, std::ios::beg);

    if (fileSize < static_cast<std::streamoff>(sizeof(int32_t))) {
        std::cerr << "Error: File is too small.\n";
        return false;
    }

    // Read number of dimensions
    int32_t ndim;
    file.read(reinterpret_cast<char*>(&ndim), sizeof(ndim));

    if (!file || ndim <= 0 || ndim > 2) {
        std::cerr << "Error: Invalid number of dimensions.\n";
        return false;
    }

    // Read shape.
    std::vector<int64_t> shape(ndim);
    file.read(reinterpret_cast<char*>(shape.data()),
              ndim * sizeof(int64_t));

    if (!file) {
        std::cerr << "Error: Failed to read shape.\n";
        return false;
    }

    // Compute total number of elements
    size_t totalElements = 1;

    for (int64_t dim : shape) {

        if (dim <= 0) {
            std::cerr << "Error: Invalid dimension: " << dim << '\n';
            return false;
        }

        const size_t sdim = static_cast<size_t>(dim);

        if (totalElements > std::numeric_limits<size_t>::max() / sdim) {
            std::cerr << "Error: Array size overflow.\n";
            return false;
        }

        totalElements *= sdim;
    }

    // Verify file size
    const std::streamoff expectedSize =
        sizeof(int32_t) +
        static_cast<std::streamoff>(ndim) * sizeof(int64_t) +
        static_cast<std::streamoff>(totalElements) * sizeof(double);

    if (fileSize != expectedSize) {
        std::cerr << "Error: File size mismatch.\n"
                  << "Expected: " << expectedSize
                  << " bytes\nActual:   " << fileSize
                  << " bytes\n";
        return false;
    }

    // Read data into a temporary buffer.
    std::vector<double> data(totalElements);

    file.read(reinterpret_cast<char*>(data.data()),
              totalElements * sizeof(double));

    if (!file) {
        std::cerr << "Error: Failed to read array data.\n";
        return false;
    }

    std::size_t rows, cols;
    // Commit only after success
    if (ndim == 1) {
        rows = 1;
        cols = shape[0];
    } else {
        rows = shape[0];
        cols = shape[1];
    }

    result.assign(rows, cols, std::move(data));

    return true;
}

} // namespace lt
