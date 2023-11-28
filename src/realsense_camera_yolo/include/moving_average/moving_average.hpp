#ifndef MOVING_AVERAGE_HPP
#define MOVING_AVERAGE_HPP

#include <iostream>
#include <vector>

class MovingAverage {
private:
    std::vector<double> data;  // Vector to store the data stream
    size_t windowSize;         // Size of the moving average window
    double sum;                // Running sum of the values in the window

public:
    // Constructor to initialize the moving average object with a specified window size
    MovingAverage(size_t size) : windowSize(size), sum(0.0) {}

    // Function to add a new value to the data stream and update the moving average
    void addValue(double value) {
        data.push_back(value);  // Add the new value to the data stream

        sum += value;  // Add the new value to the running sum

        // If the size of the data stream exceeds the window size, remove the oldest value
        if (data.size() > windowSize) {
            sum -= data.front();  // Subtract the oldest value from the running sum
            data.erase(data.begin());  // Remove the oldest value from the data stream
        }
    }

    // Function to calculate and return the current moving average
    double getMovingAverage() const {
        // If the data stream is empty, return 0 (or handle it according to your application)
        if (data.empty()) {
            return 0.0;
        }

        // Calculate and return the moving average
        return sum / data.size();
    }
};

#endif  // MOVING_AVERAGE_HPP