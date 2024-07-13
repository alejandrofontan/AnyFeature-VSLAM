//
// Created by fontan on 18/02/24.
//

#include "Observation.h"

ANYFEATURE_VSLAM::Observation::Observation(Keyframe projKeyframe, const KeypointIndex& projIndex,
                                    Keyframe refKeyframe,  const KeypointIndex& refIndex):
        projKeyframe(projKeyframe), projIndex(projIndex),
        refKeyframe(refKeyframe), refIndex(refIndex){
}