// cpd - Coherent Point Drift
// Copyright (C) 2017 Pete Gadomski <pete.gadomski@gmail.com>
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License along
// with this program; if not, write to the Free Software Foundation, Inc.,
// 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.

#include <cpd/nonrigid.hpp>

//#include"iostream"

namespace cpdG {

void Nonrigid::init(const Matrix& fixed, const Matrix& moving) {
    m_g = affinity(moving, moving, m_beta);
    m_w = Matrix::Zero(moving.rows(), moving.cols());
}

void Nonrigid::modify_probabilities(Probabilities& probabilities) const {
    probabilities.l += m_lambda / 2.0 * (m_w.transpose() * m_g * m_w).trace();
}

NonrigidResult Nonrigid::compute_one(const Matrix& fixed, const Matrix& moving,
                                     const Probabilities& probabilities,
                                     double sigma2) const {  
    //Is not needed with new sigma calc
    size_t cols = fixed.cols();
    auto dp = probabilities.p1.asDiagonal();
    
    Matrix w = (dp * m_g + m_lambda * sigma2 *
                               Matrix::Identity(moving.rows(), moving.rows()))
                   .colPivHouseholderQr()
                   .solve(probabilities.px - dp * moving);
    NonrigidResult result;
    
    //added to save W and G in the process
    result.G = m_g;
    result.W = w;
    //
    
    result.points = moving + m_g * w;
    //Is not needed with new sigma calc
    double np = probabilities.p1.sum();
    
    //Old calculation of sigma
    result.sigma2 = std::abs(
        ((fixed.array().pow(2) * probabilities.pt1.replicate(1, cols).array())
             .sum() +
         (result.points.array().pow(2) *
          probabilities.p1.replicate(1, cols).array())
             .sum() -
         2 * (probabilities.px.transpose() * result.points).trace()) /
        (np * cols));
    
    
    //sigma calculated like in corbot
    //result.sigma2 = 0.98*sigma2; 
    
    return result;
}

NonrigidResult nonrigid(const Matrix& fixed, const Matrix& moving) {
    Nonrigid nonrigid;
    return nonrigid.run(fixed, moving);
}

    
Matrix NonrigidResult::getW(){
    return W;
}
 

Matrix computeG(Matrix transforming, Matrix base, double sigma){
    return affinity(transforming, base, sigma);
}


    //Return W like in corbot
Matrix running(Matrix X, Matrix Y){
    ///to just calculate without normals
    Matrix Xl;
    Matrix Yl;
    Xl = X.leftCols(3);
    Yl = Y.leftCols(3);
    
    cpdG::Nonrigid nonR;
    nonR.beta(1);
    nonR.lambda(3);
    //nonR.correspondence(true);
    nonR.max_iterations(250);
    nonR.linked(false);
    nonR.normalize(false);
    
    cpdG::NonrigidResult ergG = nonR.run(X, Y);
    return ergG.getW();
}

} // namespace cpd
