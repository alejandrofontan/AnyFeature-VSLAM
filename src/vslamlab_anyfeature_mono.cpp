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


void LoadImages(const string &pathToSequence, const string &rgb_csv,
                vector<string> &imageFilenames, vector<ANYFEATURE_VSLAM::Seconds> &timestamps,
                const string cam_name = "rgb0");
std::string paddingZeros(const std::string& number, const size_t numberOfZeros = 5);

void removeSubstring(std::string& str, const std::string& substring) {
    size_t pos;
    while ((pos = str.find(substring)) != std::string::npos) {
        str.erase(pos, substring.length());
    }
}


int main(int argc, char **argv)
{
    // ANYFEATURE_VSLAM inputs
    string sequence_path;
    string calibration_yaml;
    string rgb_csv;
    string exp_folder;
    string exp_id{"0"};
    string settings_yaml{"vslamlab_anyfeature-dev_settings.yaml"};
    bool verbose{true};

    string feature{"orb32"};
    string feature_settings_yaml_file;
    string path_to_vocabulary_folder("anyfeature_vocabulary");
    bool fixImageSize = false;

    cout << endl;
    for (int i = 0; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg.find("sequence_path:") != std::string::npos) {
            removeSubstring(arg, "sequence_path:");
            sequence_path =  arg;
            std::cout << "[vslamlab_anyfeature_mono.cpp] Path to sequence = " << sequence_path << std::endl;
            continue;
        }
        if (arg.find("calibration_yaml:") != std::string::npos) {
            removeSubstring(arg, "calibration_yaml:");
            calibration_yaml =  arg;
            std::cout << "[vslamlab_anyfeature_mono.cpp] Path to calibration.yaml = " << calibration_yaml << std::endl;
            continue;
        }
        if (arg.find("rgb_csv:") != std::string::npos) {
            removeSubstring(arg, "rgb_csv:");
            rgb_csv =  arg;
            std::cout << "[vslamlab_anyfeature_mono.cpp] Path to rgb_csv = " << rgb_csv << std::endl;
            continue;
        }
        if (arg.find("exp_folder:") != std::string::npos) {
            removeSubstring(arg, "exp_folder:");
            exp_folder =  arg;
            std::cout << "[vslamlab_anyfeature_mono.cpp] Path to exp_folder = " << exp_folder << std::endl;
            continue;
        }
        if (arg.find("exp_id:") != std::string::npos) {
            removeSubstring(arg, "exp_id:");
            exp_id =  arg;
            std::cout << "[vslamlab_anyfeature_mono.cpp] Exp id = " << exp_id << std::endl;
            continue;
        }
        if (arg.find("settings_yaml:") != std::string::npos) {
            removeSubstring(arg, "settings_yaml:");
            settings_yaml =  arg;
            std::cout << "[vslamlab_anyfeature_mono.cpp] Path to settings_yaml = " << settings_yaml << std::endl;
            continue;
        }
        if (arg.find("verbose:") != std::string::npos) {
            removeSubstring(arg, "verbose:");
            verbose = bool(std::stoi(arg));
            std::cout << "[vslamlab_anyfeature_mono.cpp] Activate Visualization = " << verbose << std::endl;
            continue;
        }
        if (arg.find("vocabulary_folder:") != std::string::npos) {
            removeSubstring(arg, "vocabulary_folder:");
            path_to_vocabulary_folder = arg;
            std::cout << "[vslamlab_anyfeature_mono.cpp] Path to vocabulary folder = " << path_to_vocabulary_folder << std::endl;
            continue;
        }
        if (arg.find("feature:") != std::string::npos) {
            removeSubstring(arg, "feature:");
            feature =  arg;
            std::cout << "[vslamlab_anyfeature_mono.cpp] Feature = " << feature << std::endl;
            continue;
        }
        if (arg.find("feature_yaml:") != std::string::npos) {
            removeSubstring(arg, "feature_yaml:");
            feature_settings_yaml_file =  arg;
            std::cout << "[vslamlab_anyfeature_mono.cpp] Path to feature_yaml = " << feature_settings_yaml_file << std::endl;
            continue;
        }
    }
    
    // AnyFeature-VSLAM inputs
    int feature_id = get_feature_id(feature);
    auto featureType = FeatureType(feature_id);

    if(feature_settings_yaml_file.empty())
        feature_settings_yaml_file = "settings/" + feature + "_settings.yaml";

    // Retrieve paths to images
    vector<string> imageFilenames{};
    vector<ANYFEATURE_VSLAM::Seconds> timestamps{};
    LoadImages(sequence_path, rgb_csv, imageFilenames, timestamps);

    // Retrieve paths to images
    size_t nImages = imageFilenames.size();

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    const vector<FeatureType> featureTypes{featureType};
    ANYFEATURE_VSLAM::System SLAM(path_to_vocabulary_folder, 
                                  calibration_yaml, settings_yaml, feature_settings_yaml_file,
                                  ANYFEATURE_VSLAM::System::MONOCULAR,
                                  verbose,
                                  featureTypes, 
                                  fixImageSize);

    // Vector for tracking time statistics
    vector<ANYFEATURE_VSLAM::Seconds> vTimesTrack;
    vTimesTrack.resize(nImages);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;

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
        string resultsPath_expId = exp_folder + "/" + paddingZeros(exp_id);
    SLAM.SaveKeyFrameTrajectoryTUM(resultsPath_expId + "_" + "KeyFrameTrajectory.csv");

    return 0;
}

std::vector<std::string> split(const std::string& s, char delimiter) {
    std::vector<std::string> tokens;
    std::string token;
    std::istringstream tokenStream(s);
    while (std::getline(tokenStream, token, delimiter)) {
        // Simple trim for leading/trailing whitespace, often needed in real-world CSVs
        token.erase(0, token.find_first_not_of(" \t\n\r"));
        token.erase(token.find_last_not_of(" \t\n\r") + 1);
        tokens.push_back(token);
    }
    return tokens;
}

void LoadImages(const string &pathToSequence, const string &rgb_csv,
                vector<string> &imageFilenames, vector<ANYFEATURE_VSLAM::Seconds> &timestamps,
                const string cam_name)
{

    imageFilenames.clear();
    timestamps.clear();
    
    std::ifstream in(rgb_csv);
    std::string line;

    // Read and map the header row to find indices
    if (!std::getline(in, line)) return; 
    if (!line.empty() && line.back() == '\r') line.pop_back();

    std::vector<std::string> headers = split(line, ',');
    std::map<std::string, int> col_map;
    for (size_t i = 0; i < headers.size(); ++i) {
        col_map[headers[i]] = i;
    }

    // Required headers
    const std::string header_ts = "ts_" + cam_name;
    const std::string header_rgb0 = "path_" + cam_name;

    // Safely get indices
    auto get_index = [&](const std::string& key) -> int {
        return col_map[key];
    };

    int ts_idx = get_index(header_ts);
    int rgb0_idx = get_index(header_rgb0);   

    // Read and process data lines using fixed indices
    while (std::getline(in, line)) {
        if (line.empty()) continue;
        if (!line.empty() && line.back() == '\r') line.pop_back();

        std::vector<std::string> tokens = split(line, ',');
        
        // Assign variables using indices, regardless of column order
        std::string t_str = tokens[ts_idx];
        std::string rel_rgb0_path = tokens[rgb0_idx];

        ANYFEATURE_VSLAM::Seconds t = std::stod(t_str);

        timestamps.push_back(t);
        imageFilenames.push_back(pathToSequence + "/" + rel_rgb0_path);
    }
}

std::string paddingZeros(const std::string& number, const size_t numberOfZeros){
    std::string zeros{};
    for(size_t iZero{}; iZero < numberOfZeros - number.size(); ++iZero)
        zeros += "0";
    return (zeros + number);
}