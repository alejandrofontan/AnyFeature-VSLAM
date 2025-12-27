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



#include "System.h"
#include "Converter.h"
#include "MathFunctions.h"
#include "FeatureMatcher.h"

#include "DBoW2/Random.h"

#include <thread>
#include <pangolin/pangolin.h>
#include <iomanip>
#include <yaml-cpp/yaml.h>

namespace ANYFEATURE_VSLAM
{

System::System(const string &vocabularyFolder,
               const string &strCalibrationFile, const string &strSettingsFile,  
               const string &feature_settings_yaml_file,
               const eSensor sensor,
               const bool activateVisualization,
               const vector<FeatureType>& featureTypes,
               const bool& fixImageSize):
               mSensor(sensor), viewer(static_cast<shared_ptr<Viewer>>(nullptr)), mbReset(false),mbActivateLocalizationMode(false),
        mbDeactivateLocalizationMode(false), featureTypes(featureTypes)
{
    // Output welcome message
    cout << "Any-Feature V-SLAM 2024, Alejandro Fontan Villacampa, Queensland University of Technology\n"
    "    Acknowledgments to: Javier Civera and Michael Milford (Any-Feature V-SLAM)\n"
    "    Raul Mur-Artal, Juan D. Tardos, J. M. M. Montiel (ORB-SLAM2), Dorian Galvez-Lopez (DBoW2),\n    Carlos Campos, Richard Elvira and Juan J. Gómez Rodríguez (ORB-SLAM3)."
    "\n\nThis program comes with ABSOLUTELY NO WARRANTY;" << endl  <<
    "This is free software, and you are welcome to redistribute it under certain conditions. See LICENSE.txt." << endl << endl;

    cout << "Input sensor was set to: ";

    if(mSensor==MONOCULAR)
        cout << "mono" << endl;
    else if(mSensor==STEREO)
        cout << "Stereo" << endl;
    else if(mSensor==RGBD)
        cout << "RGB-D" << endl;

    //Check settings file
    cv::FileStorage fsCalibration(strCalibrationFile.c_str(), cv::FileStorage::READ);
    if(!fsCalibration.isOpened())
    {
       cerr << "Failed to open calibration file at: " << strCalibrationFile << endl;
       exit(-1);
    }

    cv::FileStorage fsSettings(strSettingsFile.c_str(), cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        cerr << "Failed to open settings file at: " << strSettingsFile << endl;
        exit(-1);
    }

    DUtils::Random::SeedRandOnce(0);

    //Load ORB Vocabulary
    vocabulary = make_shared<Vocabulary>(vocabularyFolder, GetDescriptorType(featureTypes[0]));
    vocabulary->createVocabulary();
    bool vocabularyLoaded = vocabulary->loadFromTextFile();
    if(!vocabularyLoaded){
        std::cout <<"[System] Vocabulary loading failed" << std::endl;
        terminate();
    }

    //Create KeyFrame Database
    mpKeyFrameDatabase = make_shared<KeyFrameDatabase>(vocabulary);

    //Create the Map
    mpMap = make_shared<Map>();

    //Create Drawers. These are used by the Viewer
    frameDrawer = make_shared<FrameDrawer>(mpMap,featureTypes);
    mapDrawer = make_shared<MapDrawer>(mpMap, strSettingsFile,featureTypes);

    // Initialize matching thresholds
    FeatureMatcher::setDescriptorDistanceThresholds(feature_settings_yaml_file);

    //Initialize the Tracking thread
    //(it will live in the main thread of execution, the one that called this constructor)
    tracker = make_shared<Tracking>(this, vocabulary, frameDrawer, mapDrawer,
                             mpMap, mpKeyFrameDatabase,
                             strCalibrationFile, strSettingsFile, 
                             feature_settings_yaml_file,
                             mSensor, featureTypes, fixImageSize);

    //Initialize the Local Mapping thread and launch
    localMapper = make_shared<LocalMapping>(mpMap, mSensor==MONOCULAR, featureTypes);
    mptLocalMapping = make_shared<thread>(&ANYFEATURE_VSLAM::LocalMapping::Run, localMapper);

    //Initialize the Loop Closing thread and launch
    loopCloser =  make_shared<LoopClosing>(mpMap, mpKeyFrameDatabase, vocabulary, mSensor!=MONOCULAR);
    mptLoopClosing = make_shared<thread>(&ANYFEATURE_VSLAM::LoopClosing::Run, loopCloser);

    //Initialize the Viewer thread and launch
    if(activateVisualization)
    {
        viewer = make_shared<Viewer>(static_cast<shared_ptr<System>>(this), frameDrawer,mapDrawer,tracker,
                                    strCalibrationFile, strSettingsFile,
                                    featureTypes);
        mptViewer = make_shared<thread>(&Viewer::Run, viewer);
        tracker->SetViewer(viewer);
    }

    //Set pointers between threads
    tracker->SetLocalMapper(localMapper);
    tracker->SetLoopClosing(loopCloser);

    localMapper->SetTracker(tracker);
    localMapper->SetLoopCloser(loopCloser);

    loopCloser->SetTracker(tracker);
    loopCloser->SetLocalMapper(localMapper);
    loopCloser->SetMapDrawer(mapDrawer);
}

mat4f System::TrackStereo(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timestamp)
{
    if(mSensor!=STEREO)
    {
        cerr << "ERROR: you called TrackStereo but input sensor was not set to STEREO." << endl;
        exit(-1);
    }   

    // Check mode change
    {
        unique_lock<mutex> lock(mMutexMode);
        if(mbActivateLocalizationMode)
        {
           localMapper->RequestStop();

            // Wait until Local Mapping has effectively stopped
            while(!localMapper->isStopped())
            {
                usleep(1000);
            }

            tracker->InformOnlyTracking(true);
            mbActivateLocalizationMode = false;
        }
        if(mbDeactivateLocalizationMode)
        {
            tracker->InformOnlyTracking(false);
            localMapper->Release();
            mbDeactivateLocalizationMode = false;
        }
    }

    // Check reset
    {
    unique_lock<mutex> lock(mMutexReset);
    if(mbReset)
    {
        tracker->Reset();
        mbReset = false;
    }
    }

    mat4f Tcw = tracker->GrabImageStereo(imLeft,imRight,timestamp);

    unique_lock<mutex> lock2(mMutexState);
    mTrackingState = tracker->mState;
    mTrackedMapPoints = tracker->currentFrame.pts;
    mTrackedKeyPointsUn = tracker->currentFrame.mvKeysUn;
    return Tcw;
}

    mat4f System::TrackRGBD(const cv::Mat &im, const cv::Mat &depthmap, const double &timestamp)
{
    if(mSensor!=RGBD)
    {
        cerr << "ERROR: you called TrackRGBD but input sensor was not set to RGBD." << endl;
        exit(-1);
    }    

    // Check mode change
    {
        unique_lock<mutex> lock(mMutexMode);
        if(mbActivateLocalizationMode)
        {
            localMapper->RequestStop();

            // Wait until Local Mapping has effectively stopped
            while(!localMapper->isStopped())
            {
                usleep(1000);
            }

            tracker->InformOnlyTracking(true);
            mbActivateLocalizationMode = false;
        }
        if(mbDeactivateLocalizationMode)
        {
            tracker->InformOnlyTracking(false);
            localMapper->Release();
            mbDeactivateLocalizationMode = false;
        }
    }

    // Check reset
    {
    unique_lock<mutex> lock(mMutexReset);
    if(mbReset)
    {
        tracker->Reset();
        mbReset = false;
    }
    }

    mat4f Tcw = tracker->GrabImageRGBD(im,depthmap,timestamp);

    unique_lock<mutex> lock2(mMutexState);
    mTrackingState = tracker->mState;
    mTrackedMapPoints = tracker->currentFrame.pts;
    mTrackedKeyPointsUn = tracker->currentFrame.mvKeysUn;
    return Tcw;
}

mat4f System::TrackMonocular(Image &im, const double &timestamp)
{
    std::chrono::steady_clock::time_point t_start = std::chrono::steady_clock::now();

    if(mSensor!=MONOCULAR)
    {
        cerr << "ERROR: you called TrackMonocular but input sensor was not set to mono." << endl;
        exit(-1);
    }

    // Check mode change
    {
        unique_lock<mutex> lock(mMutexMode);
        if(mbActivateLocalizationMode)
        {
            localMapper->RequestStop();

            // Wait until Local Mapping has effectively stopped
            while(!localMapper->isStopped())
            {
                usleep(1000);
            }

            tracker->InformOnlyTracking(true);
            mbActivateLocalizationMode = false;
        }
        if(mbDeactivateLocalizationMode)
        {
            tracker->InformOnlyTracking(false);
            localMapper->Release();
            mbDeactivateLocalizationMode = false;
        }
    }

    // Check reset
    {
    unique_lock<mutex> lock(mMutexReset);
    if(mbReset)
    {
        tracker->Reset();
        mbReset = false;
    }
    }

    mat4f Tcw = tracker->GrabImageMonocular(im,timestamp);

    unique_lock<mutex> lock2(mMutexState);
    mTrackingState = tracker->mState;
    mTrackedMapPoints = tracker->currentFrame.pts;
    mTrackedKeyPointsUn = tracker->currentFrame.mvKeysUn;

    std::chrono::steady_clock::time_point t_end = std::chrono::steady_clock::now();
    double t_duration = std::chrono::duration_cast<std::chrono::duration<double> >(t_end - t_start).count();
    trackingTime.push_back(t_duration);

    return Tcw;
}

void System::ActivateLocalizationMode()
{
    unique_lock<mutex> lock(mMutexMode);
    mbActivateLocalizationMode = true;
}

void System::DeactivateLocalizationMode()
{
    unique_lock<mutex> lock(mMutexMode);
    mbDeactivateLocalizationMode = true;
}

bool System::MapChanged()
{
    static int n=0;
    int curn = mpMap->GetLastBigChangeIdx();
    if(n<curn)
    {
        n=curn;
        return true;
    }
    else
        return false;
}

void System::Reset()
{
    unique_lock<mutex> lock(mMutexReset);
    mbReset = true;
}

void System::Shutdown()
{
    localMapper->RequestFinish();
    loopCloser->RequestFinish();
    if(viewer)
    {
        viewer->RequestFinish();
        while(!viewer->isFinished())
            usleep(5000);
    }

    // Wait until all thread have effectively stopped
    while(!localMapper->isFinished() || !loopCloser->isFinished() || loopCloser->isRunningGBA())
    {
        usleep(5000);
    }

    if(viewer)
        pangolin::BindToContext("ORB-SLAM2: Map Viewer");
}

void System::SaveTrajectoryTUM(const string &filename)
{
    cout << endl << "Saving camera trajectory to " << filename << " ..." << endl;
    if(mSensor==MONOCULAR)
    {
        cerr << "ERROR: SaveTrajectoryTUM cannot be used for monocular." << endl;
        return;
    }

    vector<Keyframe> vpKFs = mpMap->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    mat4f Two = vpKFs[0]->GetPoseInverse();

    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
    // We need to get first the keyframe pose and then concatenate the relative transformation.
    // Frames not localized (tracking failure) are not saved.

    // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
    // which is true when tracking failed (lbL).
    list<ANYFEATURE_VSLAM::Keyframe>::iterator lRit = tracker->mlpReferences.begin();
    list<double>::iterator lT = tracker->mlFrameTimes.begin();
    list<bool>::iterator lbL = tracker->mlbLost.begin();
    for(list<mat4f>::iterator lit=tracker->mlRelativeFramePoses.begin(),
        lend=tracker->mlRelativeFramePoses.end();lit!=lend;lit++, lRit++, lT++, lbL++)
    {
        if(*lbL)
            continue;

        Keyframe pKF = *lRit;

        mat4f Trw{mat4f::Identity()};

        // If the reference keyframe was culled, traverse the spanning tree to get a suitable keyframe.
        while(pKF->isBad())
        {
            Trw = Trw * pKF->Tcp;
            pKF = pKF->GetParent();
        }

        Trw = Trw*pKF->GetPose()*Two;

        mat4f Tcw = (*lit) * Trw;
        mat3f Rwc = Tcw.block<3,3>(0,0).transpose();
        vec3f twc = -Rwc * Tcw.block<3,1>(0,3);
        Eigen::Quaternionf q(Rwc);

        f << setprecision(6) << *lT << " " <<  setprecision(9) <<
            twc(0) << " " << twc(1) << " " << twc(2) << " " <<
            q.x() << " " << q.y() << " " << q.z()<< " " << q.w() << endl;
    }
    f.close();
    cout << endl << "trajectory saved!" << endl;
}


void System::SaveKeyFrameTrajectoryVSLAMLAB(const string &filename)
{
    cout << endl << "Saving keyframe trajectory to " << filename << " ..." << endl;

    vector<Keyframe> vpKFs = mpMap->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    //cv::Mat Two = vpKFs[0]->GetPoseInverse();

    std::ofstream f(filename.c_str());
    f.imbue(std::locale::classic());

    // CSV header
    f << "ts (ns),tx (m),ty (m),tz (m),qx,qy,qz,qw\n";

    for(size_t i=0; i<vpKFs.size(); i++)
    {
        Keyframe pKF = vpKFs[i];

       // pKF->SetPose(pKF->GetPose()*Two);

        if(pKF->isBad())
            continue;

        mat3f R = pKF->GetRotation().transpose();
        Eigen::Quaternionf q(R);
        vec3f t = pKF->GetCameraCenter();

        long long ts_ns = static_cast<long long>(std::round(pKF->mTimeStamp * 1e9));
        f << std::fixed << std::setprecision(9) << ts_ns << ','
          << std::scientific << std::setprecision(7)
          << static_cast<double>(t(0)) << ','
          << static_cast<double>(t(1)) << ','
          << static_cast<double>(t(2)) << ','
          << static_cast<double>(q.x()) << ','
          << static_cast<double>(q.y()) << ','
          << static_cast<double>(q.z()) << ','
          << static_cast<double>(q.w()) << '\n';
    }

    f.close();
    cout << endl << "trajectory saved!" << endl;
}

void System::SaveTrajectoryKITTI(const string &filename)
{
    cout << endl << "Saving camera trajectory to " << filename << " ..." << endl;
    if(mSensor==MONOCULAR)
    {
        cerr << "ERROR: SaveTrajectoryKITTI cannot be used for monocular." << endl;
        return;
    }

    vector<Keyframe> vpKFs = mpMap->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    mat4f Two = vpKFs[0]->GetPoseInverse();

    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
    // We need to get first the keyframe pose and then concatenate the relative transformation.
    // Frames not localized (tracking failure) are not saved.

    // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
    // which is true when tracking failed (lbL).
    list<ANYFEATURE_VSLAM::Keyframe>::iterator lRit = tracker->mlpReferences.begin();
    list<double>::iterator lT = tracker->mlFrameTimes.begin();
    for(list<mat4f>::iterator lit=tracker->mlRelativeFramePoses.begin(), lend=tracker->mlRelativeFramePoses.end();lit!=lend;lit++, lRit++, lT++)
    {
        ANYFEATURE_VSLAM::Keyframe pKF = *lRit;

        mat4f Trw{mat4f::Identity()};

        while(pKF->isBad())
        {
          //  cout << "bad parent" << endl;
            Trw = Trw * pKF->Tcp;
            pKF = pKF->GetParent();
        }

        Trw = Trw *pKF->GetPose()*Two;

        mat4f Tcw = (*lit) * Trw;
        mat3f Rwc = Tcw.block<3,3>(0,0).transpose();
        vec3f twc = -Rwc * Tcw.block<3,1>(0,3);

        f << setprecision(9) <<
             Rwc(0,0) << " " << Rwc(0,1)  << " " << Rwc(0,2) << " "  << twc(0) << " " <<
             Rwc(1,0) << " " << Rwc(1,1)  << " " << Rwc(1,2) << " "  << twc(1) << " " <<
             Rwc(2,0) << " " << Rwc(2,1)  << " " << Rwc(2,2) << " "  << twc(2) << endl;
    }
    f.close();
    cout << endl << "trajectory saved!" << endl;
}

int System::GetTrackingState()
{
    unique_lock<mutex> lock(mMutexState);
    return mTrackingState;
}

vector<Pt> System::GetTrackedMapPoints()
{
    unique_lock<mutex> lock(mMutexState);
    return mTrackedMapPoints;
}

vector<cv::KeyPoint> System::GetTrackedKeyPointsUn()
{
    unique_lock<mutex> lock(mMutexState);
    return mTrackedKeyPointsUn;
}

void System::SaveStatistics(const std::string &filename){
    auto keyframes = mpMap->GetAllKeyFrames();
    auto pts = mpMap->GetAllMapPoints();

    size_t numKeyframes{0};
    for (auto& keyframe: keyframes) {
        if(keyframe->isBad())
            continue;
        numKeyframes++;
    }

    size_t numPts{0};
    size_t numObservations{0};
    for (auto& pt: pts) {
        if(pt->isBad())
            continue;
        numPts++;
        numObservations += pt->NumberOfObservations();
    }
    float numObservationsPerPt = float(numObservations)/float(numPts);

    double medianTrackingTime{};
    ANYFEATURE_VSLAM::vectorMedian(medianTrackingTime,trackingTime);

    double medianLocalMapppingTime{};
    ANYFEATURE_VSLAM::vectorMedian(medianLocalMapppingTime,localMapper->localMappingTime);

    double medianLoopClosingTime{};
    ANYFEATURE_VSLAM::vectorMedian(medianLoopClosingTime,loopCloser->loopClosingTime);

    long long finalVirtualMemUsed{virtualMemUsed.back()};
    long long firstVirtualMemUsed{virtualMemUsed.front()};
    long long maxVirtualMemUsed{0};
    ANYFEATURE_VSLAM::vectorMax(maxVirtualMemUsed,virtualMemUsed);

    ofstream f;
    string statisticsFile = filename + ".txt";
    f.open(statisticsFile.c_str());
    f << fixed;
    f << setprecision(0) << numKeyframes << " " << numPts << " " << numObservations << " " << setprecision(3) << numObservationsPerPt  <<
    " " << setprecision(9) << medianTrackingTime << " " << medianLocalMapppingTime << " " << medianLoopClosingTime <<
    " " << tracker->numTrackedFrames <<" " << loopCloser->numOfLoopClosures << setprecision(0) <<
    " " << firstVirtualMemUsed <<" " << maxVirtualMemUsed << " " << finalVirtualMemUsed <<
    endl;

    // Create statistics yaml file
    string statisticsFile_yaml = filename + ".yaml";
    YAML::Node node;

    node["graph"]["numKeyframes"] = numKeyframes;
    node["graph"]["numPts"] = numPts;
    node["graph"]["numObservations"] = numObservations;
    node["graph"]["numObservationsPerPt"] = numObservationsPerPt;

    node["profiling"]["medianTrackingTime"] = medianTrackingTime;
    node["profiling"]["medianLocalMapppingTime"] = medianLocalMapppingTime;
    node["profiling"]["medianLoopClosingTime"] = medianLoopClosingTime;

    node["recall"]["numTrackedFrames"] = tracker->numTrackedFrames;
    node["recall"]["loopCloser->numOfLoopClosures"] = loopCloser->numOfLoopClosures;

    node["memory"]["firstVirtualMemUsed"] = firstVirtualMemUsed;
    node["memory"]["maxVirtualMemUsed"] = maxVirtualMemUsed;
    node["memory"]["finalVirtualMemUsed"] = finalVirtualMemUsed;

    std::ofstream fout(statisticsFile_yaml);

    fout << node;
    fout.close();
    std::cout << statisticsFile_yaml + " file written successfully!" << std::endl;

}

} //namespace ORB_SLAM
