// fgt — fast Gauss transforms
// Copyright (C) 2016 Peter J. Gadomski <pete.gadomski@gmail.com>
//
// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 2.1 of the License, or (at your option) any later version.
//
// This library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public
// License along with this library; if not, write to the Free Software
// Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA

#pragma once

#include <cstddef>

#include "fgt.hpp"

namespace fgt {

/// Tuning parameters for the IFGT.
///
/// In general, you shouldn't need to create these yourself.
struct IfgtParameters {
    /// The number of clusters that should be used for the IFGT.
    Matrix::Index nclusters;
    /// The cutoff radius.
    double cutoff_radius;
};

/// Chooses appropriate parameters for an IFGT.
IfgtParameters ifgt_choose_parameters(Matrix::Index cols, double bandwidth,
                                      double epsilon,
                                      Matrix::Index max_num_clusters,
                                      Matrix::Index truncation_number_ul);
/// Chooses the appropriate truncation number for IFGT, given a max clustering
/// radius.
Matrix::Index ifgt_choose_truncation_number(Matrix::Index cols,
                                            double bandwidth, double epsilon,
                                            double max_radius,
                                            Matrix::Index truncation_number_ul);
}
