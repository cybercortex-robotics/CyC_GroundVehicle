// Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#ifndef HELPERS_H
#define HELPERS_H

#include "Eigen/Core"
#include "Eigen/Householder"
#include "Eigen/QR"
#include "CyC_TYPES.h"
#include "helpers.h"

//
// Helper functions to fit and evaluate polynomials.
//

// Evaluate a polynomial.
float polyeval(const Eigen::VectorXf &coeffs, float x)
{
    float result = 0.0;
    for (CyC_INT i = 0; i < coeffs.size(); ++i) {
        result += coeffs[i] * powf(x, (float)i);
    }
    return result;
}

Eigen::Vector2f rot(const Eigen::Vector2f& pt, float theta)
{
    Eigen::Vector2f ret;

    ret.x() = cosf(theta) * pt.x() - sin(theta) * pt.y();
    ret.y() = sinf(theta) * pt.x() + cos(theta) * pt.y();

    return ret;
}

// Fit a polynomial.
// Adapted from:
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXf polyfit(const Eigen::VectorXf &xvals, const Eigen::VectorXf &yvals, CyC_INT order)
{
    Eigen::MatrixXf A(xvals.size(), order + 1);

    for (CyC_INT i = 0; i < xvals.size(); ++i) {
        A(i, 0) = 1.0;
    }

    for (CyC_INT j = 0; j < xvals.size(); ++j) {
        for (size_t i = 0; i < order; ++i) {
            A(j, i + 1) = A(j, i) * xvals(j);
        }
    }

    auto Q = A.householderQr();
    auto result = Q.solve(yvals);
    return result;
}

Eigen::VectorXf fit_polynomial(const std::vector<Eigen::Vector2f>& pts)
{
    Eigen::VectorXf xvals(pts.size());
    Eigen::VectorXf yvals(pts.size());

    CyC_INT i = 0;
    for (const auto& pt : pts)
    {
        xvals[i] = pt.x();
        yvals[i] = pt.y();

        ++i;
    }

    return polyfit(xvals, yvals, 4);
}

Eigen::VectorXf get_poly_coefficients_ordered(const CycReferenceSetPoints& ref_points, const CycState& vehicle_state, CyC_INT num_elements)
{
    std::vector<Eigen::Vector2f> local_pts;
    auto it = ref_points.ref.begin();
    for (size_t i = 0; i < num_elements; ++it, ++i)
    {
        if (it == ref_points.ref.end()) break;
        const Eigen::Vector2f pt{ (*it)(0) - vehicle_state.x_hat[0],
            (*it)(1) - vehicle_state.x_hat[1] };
        const auto pt2 = rot(pt, -vehicle_state.x_hat[2]);
        local_pts.emplace_back(pt2);
    }

    return fit_polynomial(local_pts);
}

Eigen::VectorXf get_poly_coefficients(const std::vector<Eigen::Vector2f>& ref_points, const CycState& vehicle_state)
{
    std::vector<Eigen::Vector2f> local_pts;
    auto min_pt = std::min_element(ref_points.begin(), ref_points.end(),
        [&](const auto& left, const auto& right)
    {
        const float dist_left = sqrt(pow(left.x() - vehicle_state.x_hat[0], 2) + pow(left.y() - vehicle_state.x_hat[1], 2));
        const float dist_right = sqrt(pow(right.x() - vehicle_state.x_hat[0], 2) + pow(right.y() - vehicle_state.x_hat[1], 2));

        return dist_left < dist_right;
    });
    auto it = min_pt;
    for (size_t i = 0; i < 10; ++it, ++i)
    {
        if (it == ref_points.end()) break;

       const Eigen::Vector2f pt{ it->x() - vehicle_state.x_hat[0],
            it->y() - vehicle_state.x_hat[1] };
        const auto pt2 = rot(pt, -vehicle_state.x_hat[2]);
        
        local_pts.emplace_back(pt2);
        //local_pts.emplace_back(*it);
    }
   
    return fit_polynomial(local_pts);
}

#endif  // HELPERS_H