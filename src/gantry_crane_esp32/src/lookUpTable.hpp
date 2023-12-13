#ifndef LOOKUPTABLE_HPP
#define LOOKUPTABLE_HPP

#include <Arduino.h>

float lookUpTable[41][3] = {
    // PWM, ACS712, Multimeter
    {500.0, 1.815, 1.3},
    {475.0, 1.835, 1.2},
    {450.0, 1.86, 0.99},
    {425.0, 1.88, 0.72},
    {400.0, 1.895, 0.61},
    {375.0, 1.917, 0.52},
    {350.0, 1.94, 0.46},
    {325.0, 1.965, 0.42},
    {300.0, 1.975, 0.38},
    {275.0, 1.995, 0.34},
    {250.0, 2.016, 0.28},
    {225.0, 2.0395, 0.229},
    {200.0, 2.05, 0.194},
    {175.0, 2.069, 0.161},
    {150.0, 2.079, 0.133},
    {125.0, 2.092, 0.1},
    {100.0, 2.1, 0.0622},
    {75.0, 2.11, 0.0486},
    {50.0, 2.1155, 0.0221},
    {25.0, 2.1195, 0.0065},
    {0.0, 2.106, 0.0},
    {-25.0, 2.123, -0.0052},
    {-50.0, 2.134, -0.0193},
    {-75.0, 2.14, -0.0417},
    {-100.0, 2.14, -0.069},
    {-125.0, 2.158, -0.107},
    {-150.0, 2.167, -0.13},
    {-175.0, 2.184, -0.18},
    {-200.0, 2.19, -0.203},
    {-225.0, 2.216, -0.239},
    {-250.0, 2.23, -0.28},
    {-275.0, 2.254, -0.33},
    {-300.0, 2.258, -0.382},
    {-325.0, 2.285, -0.45},
    {-350.0, 2.3, -0.51},
    {-375.0, 2.325, -0.6},
    {-400.0, 2.335, -0.74},
    {-425.0, 2.36, -1.0},
    {-450.0, 2.375, -1.1},
    {-475.0, 2.386, -1.34},
    {-500.0, 2.395, -1.5}

};

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