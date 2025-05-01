// Discrete‑time LQG simulation in C++ for a planar quadrotor hover
// This runs a closed‑loop sim on the desktop and prints state vs. estimate

#include <iostream>
#include <array>
#include <cmath>
#include <random>
#include <iomanip>
#include <fstream>

// Sample time (seconds)
constexpr double Ts = 0.02;
// Simulation length
constexpr int STEPS = 500;

// Dimensions
enum {n=6, m=2, p=3};

// ————— Discrete‑time system matrices Filled in from MATLAB
static const double Ad[6][6] = {
  {1, 0 , 0.0020, 0.0200, 0 , 0},
  {0, 1, 0, 0, 0.0200, 0},
  {0, 0, 1, 0, 0, 0.0200},
  {0, 0, 0.1962, 1, 0 , 0.0020},
  {0, 0, 0, 0, 1, 0},
  {0, 0, 0 , 0, 0, 1}
};
static const double Bd[6][2] = {
  {0, 0},
  {0.0004, 0},
  {0, 0.0100},
  {0, 0.0007},
  {0.0400, 0},
  {0, 1}
};
static const double Cd[3][6] = {
  {1, 0, 0, 0, 0, 0},
  {0, 1, 0, 0, 0, 0},
  {0, 0, 1, 0, 0, 0}
};
static const double Kd[2][6] = {
  {0.0000, 2.9411, 0.0000, 0.0000, 3.4046, 0.0000},
  {0.8677, 0.0000, 4.9837, 1.2785, 0.0000, 0.9737}
};
static const double Ld[6][3] = {
  {0.1809, 0.000, 0.0074},
  {0.000, 0.0776, 0.000},
  {0.0001,  0.000, 0.1479},
  {0.8098, 0.000, 0.1364},
  {0.000, 0.607, 0.000},
  {0.0001, 0.000, 0.1305}
};

// Wind disturbance parameters
double wind_amp = 0.5;
double wind_freq = 1.0;

int main() {
    // Open file for CSV output
    std::ofstream out("data.csv");
    if (!out) {
        std::cerr << "Failed to open data.csv for writing\n";
        return 1;
    }
    out << std::fixed << std::setprecision(4);

    //  CSV header
    out << "step,time,";
    for (int i = 0; i < n; ++i) out << "x" << i << ",";
    for (int i = 0; i < n; ++i) out << "xhat" << i << (i < n-1 ? "," : "\n");

    // RNG for measurement noise
    std::default_random_engine rng;
    double meas_sigma[p] = { std::sqrt(0.5), std::sqrt(0.5), std::sqrt(0.01) };
    std::array<std::normal_distribution<double>, p> dist_meas;
    for (int i = 0; i < p; ++i) {
        dist_meas[i] = std::normal_distribution<double>(0.0, meas_sigma[i]);
    }

    // State arrays
    std::array<double, n> x{}, xhat{}, innov{}, y{}, u{};
    // Initial condition
    x.fill(0.0);
    xhat.fill(0.0);
    x[2] = 0.03; // initial theta offset

    double t = 0.0;
    // Simulation loop
    for (int step = 0; step < STEPS; ++step) {
        // 1) Measurement: y = Cd*x + noise
        for (int i = 0; i < p; ++i) {
            double sum = 0.0;
            for (int j = 0; j < n; ++j) sum += Cd[i][j] * x[j];
            y[i] = sum + dist_meas[i](rng);
        }
        // 2) Control: u = -Kd * xhat
        for (int i = 0; i < m; ++i) {
            double sum = 0.0;
            for (int j = 0; j < n; ++j) sum += Kd[i][j] * xhat[j];
            u[i] = -sum;
        }
        // 3) True plant update: x = Ad*x + Bd*u + disturbance
        std::array<double, n> xnext{};
        double aw = wind_amp * std::sin(wind_freq * t);
        for (int i = 0; i < n; ++i) {
            double a = 0.0, b = 0.0;
            for (int j = 0; j < n; ++j) a += Ad[i][j] * x[j];
            for (int j = 0; j < m; ++j) b += Bd[i][j] * u[j];
            xnext[i] = a + b;
        }
        // Add wind-induced velocity change to x_dot (index 3)
        xnext[3] += aw * Ts;
        x = xnext;

        // 4) Estimator update
        for (int i = 0; i < p; ++i) {
            double pred = 0.0;
            for (int j = 0; j < n; ++j) pred += Cd[i][j] * xhat[j];
            innov[i] = y[i] - pred;
        }
        std::array<double, n> xhat_next{};
        for (int i = 0; i < n; ++i) {
            double a = 0.0, b = 0.0, c = 0.0;
            for (int j = 0; j < n; ++j) a += Ad[i][j] * xhat[j];
            for (int j = 0; j < m; ++j) b += Bd[i][j] * u[j];
            for (int j = 0; j < p; ++j) c += Ld[i][j] * innov[j];
            xhat_next[i] = a + b + c;
        }
        xhat = xhat_next;

        // 5) Write CSV line
        out << step << "," << t << ",";
        for (int i = 0; i < n; ++i) out << x[i] << ",";
        for (int i = 0; i < n; ++i) out << xhat[i] << (i < n-1 ? "," : "\n");

        t += Ts;
    }
    out.close();
    return 0;
}