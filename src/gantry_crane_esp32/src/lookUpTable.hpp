#ifndef LOOKUPTABLE_HPP
#define LOOKUPTABLE_HPP

#include <Arduino.h>

float lookUpTable[41][3] = {
    // PWM, ACS712, Multimeter
    {500.0, 1.81, 1.3},
    {475.0, 1.84, 1.2},
    {450.0, 1.875, 0.99},
    {425.0, 1.91, 0.72},
    {400.0, 1.96, 0.61},
    {375.0, 2.02, 0.52},
    {350.0, 2.05, 0.46},
    {325.0, 2.1, 0.42},
    {300.0, 2.13, 0.38},
    {275.0, 2.18, 0.34},
    {250.0, 2.21, 0.28},
    {225.0, 2.248, 0.229},
    {200.0, 2.295, 0.194},
    {175.0, 2.317, 0.161},
    {150.0, 2.345, 0.133},
    {125.0, 2.375, 0.1},
    {100.0, 2.396, 0.062200000000000005},
    {75.0, 2.415, 0.048600000000000004},
    {50.0, 2.458, 0.0221},
    {25.0, 2.441, 0.0065},
    {0.0, 2.445, 0.0},
    {-25.0, 2.453, -0.0052},
    {-50.0, 2.463, -0.0193},
    {-75.0, 2.475, -0.0417},
    {-100.0, 2.496, -0.069},
    {-125.0, 2.517, -0.107},
    {-150.0, 2.539, -0.13},
    {-175.0, 2.567, -0.18},
    {-200.0, 2.595, -0.203},
    {-225.0, 2.618, -0.239},
    {-250.0, 2.67, -0.28},
    {-275.0, 2.695, -0.33},
    {-300.0, 2.735, -0.382},
    {-325.0, 2.765, -0.45},
    {-350.0, 2.81, -0.51},
    {-375.0, 2.85, -0.6},
    {-400.0, 2.895, -0.74},
    {-425.0, 2.91, -1.0},
    {-450.0, 2.958, -1.1},
    {-475.0, 2.98, -1.34},
    {-500.0, 3.01, -1.5}};

// Function to find multimeter value for a given ACS712 value
float findMultimeterValue(float acs712Value)
{
    // Extrapolate for values below the minimum in the table
    if (acs712Value < lookUpTable[0][1])
    {
        return lookUpTable[0][2] + (acs712Value - lookUpTable[0][1]) * (lookUpTable[1][2] - lookUpTable[0][2]) / (lookUpTable[1][1] - lookUpTable[0][1]);
    }
    // Extrapolate for values above the maximum in the table
    else if (acs712Value > lookUpTable[40][1])
    {
        return lookUpTable[40][2] + (acs712Value - lookUpTable[40][1]) * (lookUpTable[40][2] - lookUpTable[39][2]) / (lookUpTable[40][1] - lookUpTable[39][1]);
    }
    // Find the multimeter value for a given ACS712 value
    else
    {
        for (int i = 0; i < 41; i++)
        {
            if (acs712Value == lookUpTable[i][1])
            {
                return lookUpTable[i][2];
            }
            else if (acs712Value > lookUpTable[i][1] && acs712Value < lookUpTable[i + 1][1])
            {
                return lookUpTable[i][2] + (acs712Value - lookUpTable[i][1]) * (lookUpTable[i + 1][2] - lookUpTable[i][2]) / (lookUpTable[i + 1][1] - lookUpTable[i][1]);
            }
        }
    }

    return 0.0;
}

#endif // LOOKUPTABLE_HPP