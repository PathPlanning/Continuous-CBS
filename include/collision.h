#ifndef COLLISION_H
#define COLLISION_H

#include <vector>
#include <cmath> // For std::sqrt, std::abs, etc.
#include <algorithm> // For std::min, std::max, std::sort
#include <map>
#include <set>
#include <limits> // Required for std::numeric_limits

// Calculates the Euclidean distance between two 2D points.
double euclidean_distance(double x1, double y1, double x2, double y2);

// Solves ax^2 + bx + c = 0 for real roots, considering a tolerance.
std::vector<double> solve_quadratic_real_roots(double a, double b, double c, double tol = 1e-9);

class Collision {
public:
    Collision();

    // Calculates the safe start time for agent1 to avoid collision with agent2.
    // Parameters mirror the Python version.
    double get_safe_start_time(
        double p0x, double p0y, double p1x, double p1y, // Agent 1 path
        double q0x, double q0y, double q1x, double q1y, // Agent 2 path
        double radius = 0.5,                            // Agent radius (half of separation distance)
        double speed = 1.0,                             // Agent speed
        double agent1_initial_time = 0.0,               // Agent 1 current start time
        double agent2_initial_time = 0.0,               // Agent 2 current start time
        double tol = 1e-7                               // General tolerance for comparisons
    );

private:
    // Member variables to store parameters and derived values for internal use by methods.
    // These are set at the beginning of each get_safe_start_time call.
    double m_p0x, m_p0y, m_p1x, m_p1y;
    double m_q0x, m_q0y, m_q1x, m_q1y;
    double m_radius, m_speed, m_agent1_initial_time, m_tol;
    
    double m_v1x, m_v1y, m_T1; // Agent 1 velocity and duration
    double m_v2x, m_v2y, m_T2; // Agent 2 velocity and duration
    
    double m_R_sq;       // Squared separation distance ( (2*radius)^2 )
    double m_t_s2;       // Agent 2 initial time (parameter agent2_initial_time)

    // Memoization cache for is_unsafe_check, cleared per get_safe_start_time call.
    std::map<double, bool> memo_is_unsafe;

    // Private helper method, equivalent to Python's _is_unsafe.
    // Checks if a given start time for agent1 (t_s1_check) leads to an unsafe situation.
    bool is_unsafe_check(double t_s1_check);
};


#endif // COLLISION_H 