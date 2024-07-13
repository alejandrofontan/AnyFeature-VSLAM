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


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include<opencv2/core/core.hpp>
#include "sys/sysinfo.h"

#include<System.h>
#include<Types.h>

using namespace std;
namespace ANYFEATURE_VSLAM{
    using Seconds = double;
}

void LoadImages(const string &pathToSequence, vector<string> &imageFilenames, vector<ANYFEATURE_VSLAM::Seconds> &timestamps);
std::string paddingZeros(const std::string& number, const size_t numberOfZeros = 5);

void removeSubstring(std::string& str, const std::string& substring) {
    size_t pos;
    while ((pos = str.find(substring)) != std::string::npos) {
        str.erase(pos, substring.length());
    }
}
int main(int argc, char **argv)
{

    /////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Memory reading (temporal)
    struct sysinfo memInfo;
    sysinfo (&memInfo);
    long long virtualMemUsed0 = memInfo.totalram - memInfo.freeram;
    virtualMemUsed0 += memInfo.totalswap - memInfo.freeswap;
    virtualMemUsed0 *= memInfo.mem_unit;
    /////////////////////////////////////////////////////////////////////////////////////////////////////////
    bool activateVisualization{true};
    string anyFeature_path;
    string path_to_vocabulary_folder;
    string feature_settings_yaml_file;

    string path_to_sequence;
    string path_to_output;
    string experimentIndex{"0"};
    string feature{"orb32"};

    bool fixImageSize = false;
    for (int i = 0; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg.find("anyfeat:") != std::string::npos) {
            removeSubstring(arg, "anyfeat:");
            anyFeature_path = arg;
            std::cout << "AnyFeature path = " << anyFeature_path << std::endl;
            continue;
        }
        if (arg.find("Voc:") != std::string::npos) {
            removeSubstring(arg, "Voc:");
            path_to_vocabulary_folder = arg;
            std::cout << "Path to vocabulary folder = " << path_to_vocabulary_folder << std::endl;
            continue;
        }
        if (arg.find("FeatSet:") != std::string::npos) {
            removeSubstring(arg, "FeatSet:");
            feature_settings_yaml_file = arg;
            std::cout << "Feature settings yaml file = " << feature_settings_yaml_file << std::endl;
            continue;
        }
        if (arg.find("Vis:") != std::string::npos) {
            removeSubstring(arg, "Vis:");
            activateVisualization = bool(std::stoi(arg));
            std::cout << "Activate Visualization = " << activateVisualization << std::endl;
            continue;
        }
        if (arg.find("sequence_path:") != std::string::npos) {
            removeSubstring(arg, "sequence_path:");
            path_to_sequence =  arg;
            std::cout << "Path to sequence = " << path_to_sequence << std::endl;
            continue;
        }
        if (arg.find("exp_folder:") != std::string::npos) {
            removeSubstring(arg, "exp_folder:");
            path_to_output =  arg;
            std::cout << "Path to output = " << path_to_output << std::endl;
            continue;
        }
        if (arg.find("exp_id:") != std::string::npos) {
            removeSubstring(arg, "exp_id:");
            experimentIndex =  arg;
            std::cout << "Exp id = " << experimentIndex << std::endl;
            continue;
        }
        if (arg.find("Feat:") != std::string::npos) {
            removeSubstring(arg, "Feat:");
            feature =  arg;
            std::cout << "Feature = " << feature << std::endl;
            continue;
        }
        if (arg.find("FixRes:") != std::string::npos) {
            removeSubstring(arg, "FixRes:");
            fixImageSize =  bool(std::stoi(arg));
            std::cout << "Fix image size = " << fixImageSize << std::endl;
            continue;
        }
    }

    // AnyFeature-VSLAM inputs
    string path_to_settings = path_to_sequence + "/calibration.yaml";
    int feature_id = get_feature_id(feature);
    auto featureType = FeatureType(feature_id);

    if(feature_settings_yaml_file.empty())
        feature_settings_yaml_file = anyFeature_path + "settings/" + feature + "_settings.yaml";
    if(path_to_vocabulary_folder.empty())
        path_to_vocabulary_folder = anyFeature_path + "anyfeature_vocabulary";

    // Retrieve paths to images
    vector<string> imageFilenames{};
    vector<ANYFEATURE_VSLAM::Seconds> timestamps{};

    LoadImages(path_to_sequence, imageFilenames, timestamps);

    size_t nImages = imageFilenames.size();

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    const vector<FeatureType> featureTypes{featureType};
    ANYFEATURE_VSLAM::System SLAM(path_to_vocabulary_folder, path_to_settings, feature_settings_yaml_file,
                                  ANYFEATURE_VSLAM::System::MONOCULAR,activateVisualization,
                                  featureTypes, fixImageSize);

    // Vector for tracking time statistics
    vector<ANYFEATURE_VSLAM::Seconds> vTimesTrack;
    vTimesTrack.resize(nImages);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;

    /////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Memory reading (temporal)
    sysinfo (&memInfo);
    long long virtualMemUsed = memInfo.totalram - memInfo.freeram;
    virtualMemUsed += memInfo.totalswap - memInfo.freeswap;
    virtualMemUsed *= memInfo.mem_unit;
    SLAM.virtualMemUsed.push_back(virtualMemUsed - virtualMemUsed0);
    /////////////////////////////////////////////////////////////////////////////////////////////////////////

    // Main loop
    for(size_t ni = 0; ni < nImages; ni++)
    {

        // Read image from file
        ANYFEATURE_VSLAM::Image im(imageFilenames[ni]);

        //im.LoadMask(imageFilenames[ni]);
        ANYFEATURE_VSLAM::Seconds tframe = timestamps[ni];

        // Pass the image to the SLAM system
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
        SLAM.TrackMonocular(im,tframe);
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();

        ANYFEATURE_VSLAM::Seconds ttrack = std::chrono::duration_cast<std::chrono::duration<ANYFEATURE_VSLAM::Seconds> >(t2 - t1).count();
        vTimesTrack[ni] = ttrack;

        // Wait to load the next frame
        ANYFEATURE_VSLAM::Seconds T = 0.0;
        if(ni < nImages-1)
            T = timestamps[ni+1] - tframe;
        else if(ni > 0)
            T = tframe - timestamps[ni-1];

        if(ttrack < T)
            usleep((T-ttrack)  * 1e6);

        /////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Memory reading (temporal)
        sysinfo (&memInfo);
        long long virtualMemUsed = memInfo.totalram - memInfo.freeram;
        virtualMemUsed += memInfo.totalswap - memInfo.freeswap;
        virtualMemUsed *= memInfo.mem_unit;
        SLAM.virtualMemUsed.push_back(virtualMemUsed - virtualMemUsed0);
        /////////////////////////////////////////////////////////////////////////////////////////////////////////
        //std::cin.get();
    }

    // Stop all threads
    SLAM.Shutdown();

    // Tracking time statistics
    sort(vTimesTrack.begin(),vTimesTrack.end());
    ANYFEATURE_VSLAM::Seconds totaltime = 0.0;
    for(int ni = 0; ni < nImages; ni++)
    {
        totaltime+=vTimesTrack[ni];
    }
    cout << "-------" << endl << endl;
    cout << "median tracking time: " << vTimesTrack[nImages/2] << endl;
    cout << "mean tracking time: " << totaltime/nImages << endl;

    // Save camera trajectory
    string resultsPath_expId = path_to_output + "/" + paddingZeros(experimentIndex);
    SLAM.SaveKeyFrameTrajectoryTUM(resultsPath_expId + "_" + "KeyFrameTrajectory.txt");
    SLAM.SaveStatistics(resultsPath_expId + "_" + "statistics");

    return 0;
}

void LoadImages(const string &pathToSequence, vector<string> &imageFilenames, vector<ANYFEATURE_VSLAM::Seconds> &timestamps)
{

    ifstream times;
    string pathToTimeFile = pathToSequence + "/rgb.txt";
    times.open(pathToTimeFile.c_str());

    string s0;
    while(!times.eof())
    {
        string s;
        getline(times,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;

            ANYFEATURE_VSLAM::Seconds t;
            string sRGB;
            ss >> t;
            timestamps.push_back(t);
            ss >> sRGB;
            imageFilenames.push_back(pathToSequence + "/" +  sRGB);
        }
    }
}

std::string paddingZeros(const std::string& number, const size_t numberOfZeros){
    std::string zeros{};
    for(size_t iZero{}; iZero < numberOfZeros - number.size(); ++iZero)
        zeros += "0";
    return (zeros + number);
}
