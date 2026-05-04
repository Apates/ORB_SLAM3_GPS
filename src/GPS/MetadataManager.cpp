//
// Created by luk24515 on 14.12.25.
//
#include "GPS/MetadataManager.h"

#include "GPS/GPSUtils.h"
#include <fstream>
#include <iostream>
#include <algorithm>
#include <regex>
#include <cmath>

// Timestamp parsing
// "2024-07-25 17:31:48.867"
// → seconds (relative to first GPS entry)
double MetadataManager::ParseTimestampToSeconds(const std::string& line)
{
    int Y, M, D, h, m;
    double s;
    sscanf(line.c_str(),
           "%d-%d-%d %d:%d:%lf",
           &Y, &M, &D, &h, &m, &s);

    // Convert to seconds-of-day (relative is enough)
    return h * 3600.0 + m * 60.0 + s;
}

// Extract value like:
// [latitude: 51.493723]
bool MetadataManager::ExtractDouble(const std::string& line,
                               const std::string& key,
                               double& value)
{
    std::regex re(key + R"(\s*:\s*([-0-9\.]+))");
    std::smatch match;
    if (std::regex_search(line, match, re)) {
        value = std::stod(match[1]);
        return true;
    }
    return false;
}


bool MetadataManager::LoadFromSRT(const std::string& path)
{
    std::ifstream file(path);
    if (!file.is_open()) {
        std::cerr << "[GPSManager] Cannot open SRT: " << path << std::endl;
        return false;
    }

    std::vector<double> timestamps;
    std::vector<double> lats, lons, alts;

    std::string line;
    while (std::getline(file, line)) {

        // Detect timestamp line: "YYYY-MM-DD HH:MM:SS.mmm"
        if (line.size() > 20 && line[4] == '-' && line[7] == '-') {

            double t = ParseTimestampToSeconds(line);

            // Accumulate GPS block until </font>
            std::string gps_block;
            while (std::getline(file, line)) {
                gps_block += " " + line;
                if (line.find("</font>") != std::string::npos)
                    break;
            }

            double lat, lon, alt;

            if (!ExtractDouble(gps_block, "latitude", lat)) continue;
            if (!ExtractDouble(gps_block, "longitude", lon)) continue;

            // Prefer rel_alt, fallback to abs_alt due to better precision of rel_alt
            if (!ExtractDouble(gps_block, "rel_alt", alt)) {
                if (!ExtractDouble(gps_block, "abs_alt", alt))
                    continue;
            }

            timestamps.push_back(t);
            lats.push_back(lat);
            lons.push_back(lon);
            alts.push_back(alt);
        }
    }

    if (timestamps.empty()) {
        std::cerr << "[GPSManager] No GPS entries found" << std::endl;
        return false;
    }

    // Normalize time
    double t0 = timestamps.front();
    for (double& t : timestamps)
        t -= t0;

    // Set ENU origin
    GPSUtils::SetENUOrigin(lats[0], lons[0], alts[0]);

    // Convert to ENU
    mMeasurements.clear();
    for (size_t i = 0; i < timestamps.size(); ++i) {
        GPSMeasurement g;
        g.timestamp = timestamps[i] - timestamps[0];
        g.enu = GPSUtils::LatLonAltToENU(lats[i], lons[i], alts[i]);
        mMeasurements.push_back(g);
    }

    std::cout << "[GPSManager] Loaded "
              << mMeasurements.size()
              << " GPS measurements" << std::endl;
    cout << "[GPSManager] Time range: "
         << mMeasurements.front().timestamp << "s to "
         << mMeasurements.back().timestamp << "s" << endl;

    return true;
}

// Nearest-neighbor lookup
bool MetadataManager::GetMeasurementAtTime(double t,
                              Eigen::Vector3d& enu,
                              double max_dt) const
{
    if (mMeasurements.empty())
        return false;

    // binary search
    auto it = std::lower_bound(
        mMeasurements.begin(),
        mMeasurements.end(),
        t,
        [](const GPSMeasurement& g, double tval) {
            return g.timestamp < tval;
        });

    // check nearest neighbor
    const GPSMeasurement* best = nullptr;

    if (it != mMeasurements.end())
        best = &(*it);

    if (it != mMeasurements.begin()) {
        const GPSMeasurement* prev = &(*(it - 1));
        if (!best ||
            std::fabs(prev->timestamp - t) <
            std::fabs(best->timestamp - t)) {
            best = prev;
            }
    }

    if (!best) {
        cout << "[GPSManager] No GPSMeasurement found" << endl;
        return false;
    }

    if (std::fabs(best->timestamp - t) > max_dt) {
        cout << "[GPSManager] Time difference is " << best->timestamp - t << endl;
        return false;
    }
    enu = best->enu;
    return true;
}
