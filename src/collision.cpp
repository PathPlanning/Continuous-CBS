#include "collision.h"
#include <iostream> // For debugging, can be removed

double euclidean_distance(double x1, double y1, double x2, double y2) {
    return std::sqrt(std::pow(x2 - x1, 2) + std::pow(y2 - y1, 2));
}

std::vector<double> solve_quadratic_real_roots(double a, double b, double c, double tol) {
    if (std::abs(a) < tol) { // Linear equation
        if (std::abs(b) < tol) { // Constant equation
            return {};
        } else {
            return {-c / b};
        }
    }

    double delta = b * b - 4 * a * c;
    if (delta < -tol) { // Negative discriminant, ensure it's not due to precision
        return {};
    }
    // Treat very small negative delta as zero due to precision
    if (delta < 0) {
        delta = 0;
    }

    double sqrt_delta = std::sqrt(delta);
    double r1 = (-b - sqrt_delta) / (2 * a);
    double r2 = (-b + sqrt_delta) / (2 * a);

    if (std::abs(r1 - r2) < tol) { // Roots are very close, consider them one unique root
        return {r1};
    }
    std::vector<double> roots = {r1, r2};
    std::sort(roots.begin(), roots.end());
    return roots;
}

Collision::Collision() {}

// Checks if a given start time for agent1 (t_s1_check) leads to an unsafe situation.
bool Collision::is_unsafe_check(double t_s1_check) {
    double current_t_s1 = std::max(m_agent1_initial_time, t_s1_check);

    auto it = memo_is_unsafe.find(current_t_s1);
    if (it != memo_is_unsafe.end()) {
        return it->second;
    }

    double common_start_time = std::max(current_t_s1, m_t_s2);
    double common_end_time = std::min(current_t_s1 + m_T1, m_t_s2 + m_T2);

    if (common_start_time > common_end_time) {
        memo_is_unsafe[current_t_s1] = false;
        return false;
    }

    double x_offset_val = (m_p0x - m_q0x + m_v2x * m_t_s2) - m_v1x * current_t_s1;
    double y_offset_val = (m_p0y - m_q0y + m_v2y * m_t_s2) - m_v1y * current_t_s1;

    double vx_rel = m_v1x - m_v2x;
    double vy_rel = m_v1y - m_v2y;

    double A_coll = vx_rel * vx_rel + vy_rel * vy_rel;
    double B_coll = 2 * (x_offset_val * vx_rel + y_offset_val * vy_rel);
    double C_coll_term = x_offset_val * x_offset_val + y_offset_val * y_offset_val;
    double C_coll_prime = C_coll_term - m_R_sq;

    if (A_coll < m_tol) {
        if (C_coll_prime <= m_tol) {
            memo_is_unsafe[current_t_s1] = true;
            return true;
        } else {
            memo_is_unsafe[current_t_s1] = false;
            return false;
        }
    }

    double discriminant_t = B_coll * B_coll - 4 * A_coll * C_coll_prime;
    double epsilon_discriminant_zero = 1e-12;
    if (discriminant_t < -epsilon_discriminant_zero) {
        memo_is_unsafe[current_t_s1] = false;
        return false;
    }
    
    discriminant_t = std::max(0.0, discriminant_t);
    double sqrt_discriminant_t = std::sqrt(discriminant_t);

    double t_contact1 = (-B_coll - sqrt_discriminant_t) / (2 * A_coll);
    double t_contact2 = (-B_coll + sqrt_discriminant_t) / (2 * A_coll);

    double coll_interval_min_t = std::min(t_contact1, t_contact2);
    double coll_interval_max_t = std::max(t_contact1, t_contact2);

    double overlap_starts = std::max(common_start_time, coll_interval_min_t);
    double overlap_ends = std::min(common_end_time, coll_interval_max_t);

    if (overlap_starts < overlap_ends) {
        memo_is_unsafe[current_t_s1] = true;
        return true;
    }

    memo_is_unsafe[current_t_s1] = false;
    return false;
}

double Collision::get_safe_start_time(
    double p0x, double p0y, double p1x, double p1y,
    double q0x, double q0y, double q1x, double q1y,
    double radius_param, double speed_param, 
    double agent1_initial_time_param, double agent2_initial_time_param, 
    double tol_param
) {
    // Initialize member variables for this call
    m_p0x = p0x; m_p0y = p0y; m_p1x = p1x; m_p1y = p1y;
    m_q0x = q0x; m_q0y = q0y; m_q1x = q1x; m_q1y = q1y;
    m_radius = radius_param;
    m_speed = speed_param;
    m_agent1_initial_time = agent1_initial_time_param;
    m_t_s2 = agent2_initial_time_param;
    m_tol = tol_param;

    memo_is_unsafe.clear(); // Clear memoization cache for the new call

    double R = 2 * m_radius;
    m_R_sq = R * R;

    double d1 = euclidean_distance(m_p0x, m_p0y, m_p1x, m_p1y);
    double d2 = euclidean_distance(m_q0x, m_q0y, m_q1x, m_q1y);

    m_T1 = d1 / m_speed;
    m_T2 = d2 / m_speed;

    m_v1x = (m_p1x - m_p0x) / m_T1;
    m_v1y = (m_p1y - m_p0y) / m_T1;
    m_v2x = (m_q1x - m_q0x) / m_T2;
    m_v2y = (m_q1y - m_q0y) / m_T2;

    std::set<double> critical_t_s1_values;
    critical_t_s1_values.insert(m_agent1_initial_time);

    critical_t_s1_values.insert(m_t_s2 - m_T1);
    critical_t_s1_values.insert(m_t_s2 + m_T2 - m_T1);
    critical_t_s1_values.insert(m_t_s2);
    critical_t_s1_values.insert(m_t_s2 + m_T2);

    double vx_rel_const = m_v1x - m_v2x;
    double vy_rel_const = m_v1y - m_v2y;
    double A_coll_const_val = vx_rel_const * vx_rel_const + vy_rel_const * vy_rel_const;

    if (m_T2 > m_tol) {
        double A_tau_static = m_v2x * m_v2x + m_v2y * m_v2y;
        if (A_tau_static > m_tol) {
            double B_tau_static = 2 * ((m_q0x - m_p0x) * m_v2x + (m_q0y - m_p0y) * m_v2y);
            double C_tau_static = std::pow(m_q0x - m_p0x, 2) + std::pow(m_q0y - m_p0y, 2) - m_R_sq;
            std::vector<double> tau_roots = solve_quadratic_real_roots(A_tau_static, B_tau_static, C_tau_static, m_tol);

            if (tau_roots.size() == 2) {
                double tau_a = tau_roots[0];
                double tau_b = tau_roots[1];
                double valid_tau_start = std::max(0.0, tau_a);
                double valid_tau_end = std::min(m_T2, tau_b);

                if (valid_tau_start <= valid_tau_end + m_tol) {
                    critical_t_s1_values.insert(m_t_s2 + valid_tau_start);
                    critical_t_s1_values.insert(m_t_s2 + valid_tau_end);
                }
            } else if (tau_roots.size() == 1) {
                double tau_contact = tau_roots[0];
                if (0 <= tau_contact && tau_contact <= m_T2 + m_tol) {
                    critical_t_s1_values.insert(m_t_s2 + tau_contact);
                }
            }
        } else if (std::abs(A_tau_static) < m_tol) { // v2 is zero
            double dist_sq_p0_q0 = std::pow(m_p0x - m_q0x, 2) + std::pow(m_p0y - m_q0y, 2);
            if (dist_sq_p0_q0 <= m_R_sq + m_tol) {
                critical_t_s1_values.insert(m_t_s2);
                critical_t_s1_values.insert(m_t_s2 + m_T2);
            }
        }
    }

    // Critical points from discriminant of collision equation being zero or interval endpoints
    if (A_coll_const_val > m_tol) {
        double X_s_const = m_p0x - m_q0x + m_v2x * m_t_s2;
        double Y_s_const = m_p0y - m_q0y + m_v2y * m_t_s2;

        double b0_c = 2 * (X_s_const * vx_rel_const + Y_s_const * vy_rel_const);
        double b1_c = -2 * (m_v1x * vx_rel_const + m_v1y * vy_rel_const);

        double c2_c = m_v1x * m_v1x + m_v1y * m_v1y;
        double c1_c = -2 * (X_s_const * m_v1x + Y_s_const * m_v1y);
        double c0_prime_c = X_s_const * X_s_const + Y_s_const * Y_s_const - m_R_sq;

        double K2_dsc = b1_c * b1_c - 4 * A_coll_const_val * c2_c;
        double K1_dsc = 2 * b0_c * b1_c - 4 * A_coll_const_val * c1_c;
        double K0_dsc = b0_c * b0_c - 4 * A_coll_const_val * c0_prime_c;
        for (double r : solve_quadratic_real_roots(K2_dsc, K1_dsc, K0_dsc, m_tol)) {
            critical_t_s1_values.insert(r);
        }
        
        // t_contact = common_start_time (alpha)
        double qa = A_coll_const_val + b1_c + c2_c;
        double qb = b0_c + c1_c;
        double qc = c0_prime_c;
        for (double r_ts1 : solve_quadratic_real_roots(qa, qb, qc, m_tol)) {
            if (r_ts1 >= m_t_s2 - m_tol) critical_t_s1_values.insert(r_ts1);
        }

        // t_contact = t_s2 (alpha prime when t_s1 < t_s2)
        qa = c2_c;
        qb = b1_c * m_t_s2 + c1_c;
        qc = A_coll_const_val * m_t_s2 * m_t_s2 + b0_c * m_t_s2 + c0_prime_c;
        for (double r_ts1 : solve_quadratic_real_roots(qa, qb, qc, m_tol)) {
            if (r_ts1 < m_t_s2 + m_tol) critical_t_s1_values.insert(r_ts1);
        }

        // t_contact = common_end_time (beta)
        qa = A_coll_const_val + b1_c + c2_c;
        qb = 2 * A_coll_const_val * m_T1 + b0_c + b1_c * m_T1 + c1_c;
        qc = A_coll_const_val * m_T1 * m_T1 + b0_c * m_T1 + c0_prime_c;
        for (double r_ts1 : solve_quadratic_real_roots(qa, qb, qc, m_tol)) {
            if (r_ts1 + m_T1 <= m_t_s2 + m_T2 + m_tol) critical_t_s1_values.insert(r_ts1);
        }

        // t_contact = t_s2 + T2 (beta prime when t_s1 + T1 > t_s2 + T2)
        double t_val_beta_prime = m_t_s2 + m_T2;
        qa = c2_c;
        qb = b1_c * t_val_beta_prime + c1_c;
        qc = A_coll_const_val * t_val_beta_prime * t_val_beta_prime + b0_c * t_val_beta_prime + c0_prime_c;
        for (double r_ts1 : solve_quadratic_real_roots(qa, qb, qc, m_tol)) {
            if (r_ts1 + m_T1 > m_t_s2 + m_T2 - m_tol) critical_t_s1_values.insert(r_ts1);
        }

    } else { // Relative speed is zero (A_coll_const_val approx 0)
        double X_s_const = m_p0x - m_q0x + m_v2x * m_t_s2;
        double Y_s_const = m_p0y - m_q0y + m_v2y * m_t_s2;
        
        // Equation becomes: (v1x*t_s1 - X_s_const)^2 + (v1y*t_s1 - Y_s_const)^2 = R^2
        // (v1x^2+v1y^2)t_s1^2 - 2(X_s_const*v1x + Y_s_const*v1y)t_s1 + (X_s_const^2 + Y_s_const^2 - R^2) = 0
        double Pa_par = m_v1x * m_v1x + m_v1y * m_v1y; // This is v1_sq
        double Pb_par = -2 * (X_s_const * m_v1x + Y_s_const * m_v1y);
        double Pc_par = X_s_const * X_s_const + Y_s_const * Y_s_const - m_R_sq;
        
        for (double r : solve_quadratic_real_roots(Pa_par, Pb_par, Pc_par, m_tol)) {
            critical_t_s1_values.insert(r);
        }
    }

    std::set<double> test_point_candidates;
    test_point_candidates.insert(m_agent1_initial_time);

    double max_relevant_duration = 1.0;  // Start with 1.0 as minimum
    if (m_T1 != std::numeric_limits<double>::infinity()) max_relevant_duration = std::max(max_relevant_duration, m_T1);
    if (m_T2 != std::numeric_limits<double>::infinity()) max_relevant_duration = std::max(max_relevant_duration, m_T2);

    for (double t_crit_val : critical_t_s1_values) {
        // Heuristic from Python to filter out very distant critical points
        if (t_crit_val < m_agent1_initial_time - max_relevant_duration * 100 &&
            t_crit_val < m_t_s2 - max_relevant_duration * 100) {
            continue;
        }

        if (t_crit_val >= m_agent1_initial_time - m_tol) {
            //test_point_candidates.insert(t_crit_val);
            test_point_candidates.insert(t_crit_val + m_tol);
        }
        test_point_candidates.insert(std::max(m_agent1_initial_time, t_crit_val - m_tol));
    }

    double final_check_point = std::max(m_agent1_initial_time, m_t_s2 + m_T2);
    test_point_candidates.insert(final_check_point);

    std::vector<double> sorted_unique_test_points;
    for (double t : test_point_candidates) {
        if (t >= m_agent1_initial_time - m_tol) {
            sorted_unique_test_points.push_back(t);
        }
    }
    std::sort(sorted_unique_test_points.begin(), sorted_unique_test_points.end());

    sorted_unique_test_points.erase(std::unique(sorted_unique_test_points.begin(), sorted_unique_test_points.end()), 
                                   sorted_unique_test_points.end());


    for (double t_candidate : sorted_unique_test_points) {
        double current_test_t_s1 = std::max(m_agent1_initial_time, t_candidate);

        bool is_unsafe_result = is_unsafe_check(current_test_t_s1);

        if (!is_unsafe_result) {
            // This logic attempts to return the earliest point of a safe interval.
            // If t_candidate was formed by t_crit + tol, check t_crit as well.
            double original_t_crit = current_test_t_s1 - m_tol;
            bool is_offset_candidate = std::abs(t_candidate - (original_t_crit + m_tol)) < m_tol / 2.0;

            if (is_offset_candidate && original_t_crit >= m_agent1_initial_time - m_tol) {
                bool is_original_unsafe = is_unsafe_check(original_t_crit);
                if (!is_original_unsafe) {
                    return original_t_crit;
                }
            }
            return current_test_t_s1;
        }
    }
    
    // Fix: Match Python's fallback_time calculation exactly
    double fallback_time = std::max(m_agent1_initial_time, m_t_s2 + m_T2);
    // If T2 is infinite, agent 2 effectively doesn't move from q0 or its path is infinitely long.
    // If agent 1 also has infinite path, they might be unsafe forever if they start unsafe.
    // The original Python code defaults to max(agent1_initial_time, t_s2 + T2).
    // If T2 is inf, this becomes inf. This might not be desirable.
    // A more robust fallback might be just agent1_initial_time if agent 2 is static indefinitely and they don't collide at t_s1.
    // For now, sticking to the direct translation.
    if (is_unsafe_check(fallback_time)){
         // This situation implies that even the fallback time is unsafe.
         // This can happen if, for example, both agents are static and overlapping.
         // Or if agent 2's path is infinite and they are on a collision course with agent 1's initial segment.
         // The Python code doesn't explicitly handle returning an "impossible" or "infinite" time
         // beyond what max(t_s2+T2) would yield.
         // If T2 is infinity and unsafe at fallback_time (which is agent1_initial_time), 
         // then any finite start time for agent 1 might be unsafe if they are on a collision path.
         // For now, we return fallback_time as per python logic, but this could be a point of refinement
         // depending on how "no solution" should be represented.
         // Consider returning std::numeric_limits<double>::infinity() if no safe time is found.
    }

    return fallback_time;
}