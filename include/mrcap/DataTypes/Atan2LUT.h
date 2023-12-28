#pragma once

class Atan2LUT {
private:
    static constexpr int LUT_SIZE = 162409; // Number of precomputed values
    static constexpr double LUT_MIN = -100.0;
    static constexpr double LUT_MAX = 100.0;
    static constexpr double LUT_STEP = (LUT_MAX - LUT_MIN) / (LUT_SIZE - 1);
    static std::vector<double> lut;

public:
    Atan2LUT() {
        if (lut.empty()) {
            for (int i = 0; i < LUT_SIZE; ++i) {
                double x = LUT_MIN + i * LUT_STEP;
                lut.push_back(std::atan(x));
            }
        }
        std::cout << "Atan2LUT initialized with " << LUT_SIZE << " values" << std::endl;
    }

    double lookup(double y, double x) const {
        if (x == 0.0 && y == 0.0) return 0.0; // Handle origin

        const double EPSILON = 1e-10;  // A small threshold value to check if x is close to zero
        double slope = y / x;
        if (std::isinf(slope)) {
            return std::copysign(M_PI_2, slope);
        }
        if (slope < LUT_MIN || slope > LUT_MAX) {
            // std::cout << "Warning: atan2 lookup out of bounds: " << slope << std::endl;
            return std::atan2(y, x); // Fallback for out-of-bounds
        }
        int index = static_cast<int>((slope - LUT_MIN) / LUT_STEP);
        if (index < 0) index = 0;
        if (index >= LUT_SIZE) index = LUT_SIZE - 1;

        // Simple linear interpolation
        double t = (slope - (LUT_MIN + index * LUT_STEP)) / LUT_STEP;
        return (1.0 - t) * lut[index] + t * lut[index + 1];
    }
};

std::vector<double> Atan2LUT::lut;

Atan2LUT lut;