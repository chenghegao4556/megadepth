//
// Created by chenghe on 11/22/19.
//

#ifndef MEGADEPTH_UTIL_H
#define MEGADEPTH_UTIL_H

#include <chrono>
namespace MegaDepth
{
    ///! timer
    class Timer
    {
    public:

        typedef std::chrono::steady_clock::time_point TimePoint;

        void Tic()
        {
            t1_ = std::chrono::steady_clock::now();
        }

        void Toc()
        {
            t2_ = std::chrono::steady_clock::now();
        }

        double Duration()
        {
            std::chrono::duration<double> time_used = std::chrono::duration_cast<std::chrono::duration<double>> ( t2_-t1_ );

            return time_used.count();
        }

    private:
        TimePoint t1_, t2_;
    };//end of Timer
}//end of MegaDepth

#endif //MEGADEPTH_UTIL_H
