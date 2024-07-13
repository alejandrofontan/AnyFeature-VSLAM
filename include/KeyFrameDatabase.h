/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef KEYFRAMEDATABASE_H
#define KEYFRAMEDATABASE_H

#include <vector>
#include <list>
#include <set>

#include "KeyFrame.h"
#include "Frame.h"
#include "FeatureVocabulary.h"
#include "Vocabulary.h"

#include<mutex>


namespace ANYFEATURE_VSLAM
{

class Frame;
class KeyFrame;
typedef shared_ptr<ANYFEATURE_VSLAM::KeyFrame> Keyframe;

class KeyFrameDatabase
{
public:

    KeyFrameDatabase(const shared_ptr<Vocabulary> vocabulary);

   void add(Keyframe  pKF);

   void erase(Keyframe  pKF);

   void clear();

   // Loop Detection
   std::vector<Keyframe> DetectLoopCandidates(Keyframe  pKF, float minScore);

   // Relocalization
   std::vector<Keyframe> DetectRelocalizationCandidates(Frame* F);

protected:

  // Associated vocabulary
  const shared_ptr<Vocabulary> vocabulary;

  // Inverted file
  std::vector<list<Keyframe>> mvInvertedFile;

  // Mutex
  std::mutex mMutex;
};

} //namespace ORB_SLAM

#endif
