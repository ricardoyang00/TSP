/**
 * @file Utilities.h
 * @brief Contains utility functions for string manipulation and distance calculation.
 *
 * This header file provides various utility functions used for string manipulation and distance calculation.
 * Functions include string trimming and formatting text in bold for terminal output and Harversine Distance formula.
 */

#ifndef PROJ_DA_02_UTILITIES_H
#define PROJ_DA_02_UTILITIES_H

#include <iostream>
#include <algorithm>
#include <sstream>
#include <cmath>

/**
 * @brief Trim whitespace from the beginning and end of a string.
 * @param toTrim The string to be trimmed.
 * @return The trimmed string.
 */
std::string TrimString(const std::string& toTrim);

/**
 * @brief Formats a value in bold for terminal output.
 * @param value The value to be formatted.
 * @return The formatted value in bold.
 */
template <typename T>
std::string makeBold(const T& value) {
    std::ostringstream oss;
    oss << "\033[1m" << value << "\033[0m";
    return oss.str();
};

/**
 * @brief Converts degrees to radians.
 * @param degrees The angle in degrees to be converted.
 * @return The angle in radians.
 */
double ToRadians(double degrees);

/**
 * @brief Calculates the Haversine distance between two coordinates.
 * @param lat1 Latitude of the first coordinate.
 * @param lon1 Longitude of the first coordinate.
 * @param lat2 Latitude of the second coordinate.
 * @param lon2 Longitude of the second coordinate.
 * @return The distance between the coordinates in meters.
 */
double HarversineDistance(double lat1, double lon1, double lat2, double lon2);


#endif
