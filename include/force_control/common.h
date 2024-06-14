#pragma once

#include <Eigen/Eigen>

namespace force_control {

inline const Eigen::Matrix<double, 7, 7> inverse_semi( Eigen::Matrix<double, 7, 7> &mass_matrix ) {
    Eigen::Diagonal<Eigen::Matrix<double, 7, 7>> diag = mass_matrix.diagonal();
    Eigen::Matrix<double, 7, 7> inverse;
    for (int i = 0; i < diag.size(); ++i) {
        if( diag(i) != 0 )
            inverse(i, i) = 1.0 / diag(i);
    }

    return inverse;
}

} // namespace force_control