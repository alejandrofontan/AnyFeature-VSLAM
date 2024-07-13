//
// Created by fontan on 15/02/24.
//

#ifndef ANYFEATURE_VSLAM_MATHFUNCTIONS_H
#define ANYFEATURE_VSLAM_MATHFUNCTIONS_H

#include "iostream"
#include "algorithm"
#include "vector"
#include "limits"
#include "cmath"

namespace ANYFEATURE_VSLAM{

    template <typename T>
    void vectorMedian(T& median, const std::vector<T>& v){
        if(v.empty())
        {
            median = 0.0;
            return;
        }

        std::vector<T> vOrdered = v;
        std::sort(vOrdered.begin(),vOrdered.end());
        median = vOrdered[vOrdered.size()/2];
    }

    template <typename T>
    void vectorPercentil(T& median, const float& p, const std::vector<T>& v){
        if(v.empty())
        {
            median = 0.0;
            return;
        }

        std::vector<T> vOrdered = v;
        std::sort(vOrdered.begin(),vOrdered.end());
        median = vOrdered[int(float(vOrdered.size()) * p)];
    }

    template <typename T>
    void vectorMax(T& max, const std::vector<T>& v){
        if(v.empty())
        {
            max = 0.0;
            return;
        }

        max = std::numeric_limits<T>::min();
        for(auto& value: v)
            if(value > max)
                max = value;
    }

    template <typename T>
    void vectorMean(T& mean, const std::vector<T>& v){
        if(v.empty())
        {
            mean = 0.0;
            return;
        }

        mean = (T) 0.0;
        for(auto& value: v)
            mean += value;
        mean = (T) float(mean)/float(v.size());
    }

    template <typename T>
    void vectorStd(T& std, T& mean, const std::vector<T>& v){
        if(v.empty())
        {
            std = 0.0;
            return;
        }

        vectorMean(mean,v);

        std = (T) 0.0;
        for(auto& value: v)
            std += (value - mean) * (value - mean);

        std = (T) sqrtf(float(std)/float(v.size()));
    }

}

#endif //ANYFEATURE_VSLAM_MATHFUNCTIONS_H
