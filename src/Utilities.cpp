#include "Utilities.h"
using namespace std;

string TrimString(const string& toTrim) {
    string trimmed = toTrim;
    trimmed.erase(trimmed.begin(), find_if(trimmed.begin(), trimmed.end(),
                                           [](char c) { return !isspace(c); }));
    trimmed.erase(find_if(trimmed.rbegin(), trimmed.rend(),
                          [](char c) { return !isspace(c); }).base(), trimmed.end());
    return trimmed;
}

double ToRadians(double degrees) {
    return degrees * M_PI / 180.0;
}

double HarversineDistance(double lat1, double lon1, double lat2, double lon2) {
    constexpr double kEarthRadiusM = 6371000.0;

    lat1 = ToRadians(lat1);
    lon1 = ToRadians(lon1);
    lat2 = ToRadians(lat2);
    lon2 = ToRadians(lon2);

    double dLat = lat2 - lat1;
    double dLon = lon2 - lon1;

    double a = sin(dLat / 2.0) * sin(dLat / 2.0) +
               cos(lat1) * cos(lat2) * sin(dLon / 2.0) * sin(dLon / 2.0);
    double c = 2.0 * atan2(sqrt(a), sqrt(1.0 - a));

    return kEarthRadiusM * c;
}
