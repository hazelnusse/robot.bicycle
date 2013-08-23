/*
 * ============================================================================
 *
 *         Author:  Dale Lukas Peterson (dlp), hazelnusse@gmail.com
 *
 *    Description:  POD type for storing meta data about each recorded signal
 *
 * ============================================================================
 */
#ifndef RUN_META_DATA_H
#define RUN_META_DATA_H

#include <algorithm>
#include <cmath>
#include <limits>

namespace gui {

class MetaData {
public:
    MetaData() :  min_{0.0}, max_{0.0}, mean_{0.0}, std_{0.0} {}
    MetaData(const QVector<double> & x) :
        min_{std::numeric_limits<double>::max()},
        max_{std::numeric_limits<double>::min()},
        mean_{0.0}, std_{0.0}
    {
        if (x.size()) {
            for (const double & xi : x) {
                mean_ += xi;
                min_ = std::min(min_, xi);
                max_ = std::max(max_, xi);
            }
            mean_ /= x.size();
            for (const double & xi : x)
                std_ += std::pow(xi - mean_, 2);
            std_ /= x.size();
            std_ = std::sqrt(std_);
        }
    }
    double min_;
    double max_;
    double mean_;
    double std_;
};

} // namespace gui

#endif

