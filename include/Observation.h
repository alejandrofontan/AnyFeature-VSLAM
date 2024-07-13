//
// Created by fontan on 18/02/24.
//

#ifndef ANYFEATURE_VSLAM_OBSERVATION_H
#define ANYFEATURE_VSLAM_OBSERVATION_H

#include "KeyFrame.h"
#include "Types.h"

namespace ANYFEATURE_VSLAM {

    class KeyFrame;

    class Observation {
    public:
        Observation() = default;

        Observation(Keyframe projKeyframe, const KeypointIndex &projIndex,
                    Keyframe refKeyframe , const KeypointIndex &refIndex);

        // Observations are matched respect to a reference keyframe and projected to a projection keyframe
        // ref : reference keyframe
        // proj : projection keyframe

        Keyframe projKeyframe{};
        KeypointIndex projIndex{};

        Keyframe refKeyframe{};
        KeypointIndex refIndex{};
    };

    typedef shared_ptr<Observation> Obs;

}
#endif //ANYFEATURE_VSLAM_OBSERVATION_H
