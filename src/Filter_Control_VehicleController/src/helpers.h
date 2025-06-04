// Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#ifndef HELPERS_H
#define HELPERS_H

#include "Eigen/Core"
#include "Eigen/Householder"
#include "Eigen/QR"
#include "CyC_TYPES.h"

//
// Helper functions to fit and evaluate polynomials.
//

// Evaluate a polynomial.
float polyeval(const Eigen::VectorXf &coeffs, float x);

Eigen::Vector2f rot(const Eigen::Vector2f& pt, float theta);

// Fit a polynomial.
// Adapted from:
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXf polyfit(const Eigen::VectorXf &xvals, const Eigen::VectorXf &yvals, CyC_INT order);

Eigen::VectorXf fit_polynomial(const std::vector<Eigen::Vector2f>& pts);

Eigen::VectorXf get_poly_coefficients_ordered(const CycReferenceSetPoints& ref_points, const CycState& vehicle_state, CyC_INT num_elements);
Eigen::VectorXf get_poly_coefficients(const std::vector<Eigen::Vector2f>& ref_points, const CycState& vehicle_state);

#endif  // HELPERS_H