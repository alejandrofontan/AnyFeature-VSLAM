/**
 * File: CreateVocabulary.cpp
 * Date: March 2024
 * Original Author: Dorian Galvez-Lopez
 * Modified by Alejandro Fontan Villacampa for AnyFeature-VSLAM
 * Description: create binary vocabulary application of DBoW2
 * License: see the LICENSE.txt file
 */

#include <iostream>
#include <vector>

// DBoW2
#include "Thirdparty/DBoW2/include/DBoW2/DBoW2.h"
#include "Feature_anyFeatNonBin.h"
#include "Feature_anyFeatBin.h"
#include "Feature_sift128.h"
#include "Feature_akaze61.h"
#include "Feature_brisk48.h"
#include "Feature_orb32.h"
#include "Feature_surf64.h"

// OpenCV
#include <opencv2/core.hpp>


#include <fstream>

using namespace DBoW2;
using namespace std;

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 
std::vector<std::vector<std::string>> read_txt(const std::string &filePath, const size_t &numCols, char delimiter ,int headerRows);
void displayProgressBar(int width, double progressPercentage);

void loadBinaryFeatures(vector<vector<cv::Mat>> &features, const std::vector<std::string>& imagePaths);
void changeStructure(const cv::Mat &plain, vector<cv::Mat> &out);
void testBinaryVocCreation(const vector<vector<cv::Mat>> &features);

void loadNonBinaryFeatures(vector<vector<vector<float>>> &features, const std::vector<std::string>& imagePaths);
void changeStructure(const cv::Mat &plain, vector<vector<float>> &out);
void testNonBinaryVocCreation(const vector<vector<vector<float>>> &features);

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 
int numberOfImages{};
string savePath{""};
string vocName{""};
int descriptorId{};
string descriptorName{""};
int k = 9; // branching factor
int L = 3; // depth levels
const WeightingType weight = TF_IDF;
const ScoringType scoring = L1_NORM;
const string script_label = "[createVocabulary.cpp] ";

string featureSettingsYamlFile{};
string anyFeature_path{};

// ----------------------------------------------------------------------------
int main(int argc,char **argv){
    if(argc != 10){
        cerr << endl << "Usage: ./createVocabulary descriptorId descriptorName isBinary rgb_folder rgbTxt savePath k L featureSettingsYamlFile" << endl;
        return 1;
    }

    descriptorId = stoi(argv[1]);
    descriptorName = argv[2];
    bool isBinary = bool(stoi(argv[3]));
    string rgb_folder = argv[4];
    string rgbTxt = argv[5];
    savePath = argv[6];
    k =  stoi(argv[7]); // branching factor
    L = stoi(argv[8]); // depth levels
    featureSettingsYamlFile = argv[9];

    vocName = descriptorName + "_DBoW2.txt";
    cout << script_label + "Create : " << savePath + "/" + vocName << endl;
    std::string feature = descriptorName;
    std::transform(feature.begin(), feature.end(), feature.begin(), ::tolower);

    // Feature specifications
    cout << "    Feature specifications" << endl;
    cout << "      - descriptorName = " << descriptorName << endl;
    cout << "      - isBinary = " << isBinary << endl;
    cout << "      - descriptorId = " << descriptorId << endl;

    // Dataset specifications
    cout << "    Dataset specifications" << endl;
    cout << "      - rgb_folder factor = " << rgb_folder << endl;
    cout << "      - rgbTxt = " << rgbTxt << endl;
    cout << "      - savePath = " << savePath << endl;

    // Vocabulary specifications
    cout << "    Vocabulary specifications" << endl;
    cout << "      - branching factor = " << k << endl;
    cout << "      - depth levels = " << L << endl;

    // Load images
    std::vector<std::string> imagePaths;
    std::vector<std::vector<std::string>> imagesTxt = read_txt(rgbTxt,1,' ',0);
    for(size_t imageId{0}; imageId < imagesTxt.size(); ++imageId)
        imagePaths.push_back( rgb_folder + imagesTxt[imageId][0]);

    numberOfImages = imagePaths.size();
    cout << script_label + "Number Of Images = " << numberOfImages << endl;

    if(isBinary){
        vector<vector<cv::Mat>> features;
        loadBinaryFeatures(features,imagePaths);
        testBinaryVocCreation(features);
    }
    else{
        vector<vector<vector<float>>> features;
        loadNonBinaryFeatures(features,imagePaths);
        testNonBinaryVocCreation(features);
    }

    return 0;
}

// ----------------------------------------------------------------------------
void loadBinaryFeatures(vector<vector<cv::Mat>> &features, const std::vector<std::string>& imagePaths)
{
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    features.clear();
    features.reserve(numberOfImages);

    const int progressBarWidth = 50; // Width of the progress bar in characters
    const int totalIterations = numberOfImages; // Total iterations for the process

    shared_ptr<ANYFEATURE_VSLAM::FeatureExtractorSettings> extractorSettings;
    shared_ptr<ANYFEATURE_VSLAM::FeatureExtractor> extractor;

    switch(descriptorId) { // loadBinaryFeatures
        case DESC_ANYFEATBIN:{
            cout << script_label + "Extracting " + "anyFeatBin" + " features..." << endl;
            extractorSettings = make_shared<ANYFEATURE_VSLAM::FeatureExtractorSettings>(KEYP_ANYFEATBIN, DESC_ANYFEATBIN, featureSettingsYamlFile);
            extractor = make_shared<ANYFEATURE_VSLAM::FeatureExtractor_anyFeatBin>(1000,extractorSettings);
            break;
        }
        case DESC_BRISK:{
            cout << script_label + "Extracting " + "Brisk48" + " features..." << endl;
            extractorSettings = make_shared<ANYFEATURE_VSLAM::FeatureExtractorSettings>(KEYP_BRISK, DESC_BRISK, featureSettingsYamlFile);
            extractor = make_shared<ANYFEATURE_VSLAM::FeatureExtractor_brisk48>(1000,extractorSettings);
            break;          
        }      
        case DESC_AKAZE61: {
            cout << script_label + "Extracting " + "Akaze61" + " features..." << endl;
            extractorSettings = make_shared<ANYFEATURE_VSLAM::FeatureExtractorSettings>(KEYP_AKAZE, DESC_AKAZE61, featureSettingsYamlFile);
            extractor = make_shared<ANYFEATURE_VSLAM::FeatureExtractor_akaze61>(1000,extractorSettings);
            break;
        }
        case DESC_ORB: {
            cout << script_label + "Extracting " + "Orb" + " features..." << endl;
            extractorSettings = make_shared<ANYFEATURE_VSLAM::FeatureExtractorSettings>(KEYP_ORB, DESC_ORB, featureSettingsYamlFile);
            extractor = make_shared<ANYFEATURE_VSLAM::FeatureExtractor_orb32>(1000,extractorSettings);
            break;
        }
    }

    for(int i = 0; i < numberOfImages; ++i){
        vector<cv::KeyPoint> keypoints;
        cv::Mat descriptors;

        ANYFEATURE_VSLAM::Image im(imagePaths[i]);
        im.GetGrayImage(true);

        (*extractor)(im,keypoints,descriptors);
        features.push_back(vector<cv::Mat>());
        changeStructure(descriptors, features.back());

        double progress = (double)i / totalIterations;
        displayProgressBar(progressBarWidth, progress);
    }
    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();

    double tduration = std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
    cout << "        finished (" + std::to_string(tduration) + " s). " << endl;
}

void loadNonBinaryFeatures(vector<vector<vector<float>>> &features, const std::vector<std::string>& imagePaths)
{
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    features.clear();
    features.reserve(numberOfImages);

    const int progressBarWidth = 50; // Width of the progress bar in characters
    const int totalIterations = numberOfImages; // Total iterations for the process

    shared_ptr<ANYFEATURE_VSLAM::FeatureExtractorSettings> extractorSettings;
    shared_ptr<ANYFEATURE_VSLAM::FeatureExtractor> extractor;

    switch(descriptorId) { // loadNonBinaryFeatures
        case DESC_ANYFEATNONBIN:{
            cout << script_label + "Extracting " + "AnyFeat" + " features..." << endl;
            extractorSettings = make_shared<ANYFEATURE_VSLAM::FeatureExtractorSettings>(KEYP_ANYFEATNONBIN, DESC_ANYFEATNONBIN, featureSettingsYamlFile);
            extractor = make_shared<ANYFEATURE_VSLAM::FeatureExtractor_anyFeatNonBin>(1000,extractorSettings);
            break;
        }
        case DESC_SIFT128:{
            cout << script_label + "Extracting " + "Sift128" + " features..." << endl;
            extractorSettings = make_shared<ANYFEATURE_VSLAM::FeatureExtractorSettings>(KEYP_SIFT, DESC_SIFT128, featureSettingsYamlFile);
            extractor = make_shared<ANYFEATURE_VSLAM::FeatureExtractor_sift128>(1000,extractorSettings);
            break;
        }
        case DESC_SURF64:{
            cout << script_label + "Extracting " + "Surf64" + " features..." << endl;
            extractorSettings = make_shared<ANYFEATURE_VSLAM::FeatureExtractorSettings>(KEYP_SURF, DESC_SURF64, featureSettingsYamlFile);
            extractor = make_shared<ANYFEATURE_VSLAM::FeatureExtractor_surf64>(1000,extractorSettings);
            break;
        }
    }

    for(int i = 0; i < numberOfImages; ++i){
        vector<cv::KeyPoint> keypoints;
        cv::Mat descriptors;

        ANYFEATURE_VSLAM::Image im(imagePaths[i]);
        im.GetGrayImage(true);

        (*extractor)(im,keypoints,descriptors);

        features.push_back(vector<vector<float>>());
        changeStructure(descriptors, features.back());

        double progress = (double)i / totalIterations;
        displayProgressBar(progressBarWidth, progress);
    }
    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();

    double tduration = std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
    cout << "        finished (" + std::to_string(tduration) + " s). " << endl;
}

// ----------------------------------------------------------------------------
void changeStructure(const cv::Mat &plain, vector<cv::Mat> &out)
{
  out.resize(plain.rows);

  for(int i = 0; i < plain.rows; ++i)
  {
    out[i] = plain.row(i);
  }
}

void changeStructure(const cv::Mat &plain, vector<vector<float>> &out)
{
    out.resize(plain.rows);

    for(int i = 0; i < plain.rows; ++i)
    {
        out[i] = plain.row(i);
    }
}

// ----------------------------------------------------------------------------
void testBinaryVocCreation(const vector<vector<cv::Mat>> &features){
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

    cout << script_label + "Creating a " << k << "^" << L << " vocabulary..." << endl;

    switch (descriptorId) { // create binary vocabulary
        case DESC_ANYFEATBIN:{
            AnyFeatBinVocabulary voc(k, L, weight, scoring);
            voc.create(features);
            cout << script_label + "Vocabulary information: " << endl
                 << voc  << endl;
            cout << script_label +  "Saving vocabulary..." << endl;
            voc.saveToTextFile(savePath + "/AnyFeatBin_DBoW2_voc.txt");
            break;
        }
        case DESC_BRISK:{
            BriskVocabulary voc(k, L, weight, scoring);
            voc.create(features);
            cout << script_label + "Vocabulary information: " << endl
                 << voc  << endl;
            cout << script_label +  "Saving vocabulary..." << endl;
            voc.saveToTextFile(savePath + "/Brisk_DBoW2_voc.txt");
            break;
        }
        case DESC_AKAZE61:
        {
            Akaze61Vocabulary voc(k, L, weight, scoring);
            voc.create(features);
            cout << script_label + "Vocabulary information: " << endl
                 << voc << endl;
            cout << script_label +  "Saving vocabulary..." << endl;
            //voc.save(savePath + "/" + descriptorName + "_DBoW2_voc.yml.gz");
            voc.saveToTextFile(savePath + "/" + descriptorName + "_DBoW2_voc.txt");
            break;
        }
        case DESC_ORB:
        {
            OrbVocabulary voc(k, L, weight, scoring);
            voc.create(features);
            cout << script_label + "Vocabulary information: " << endl
                 << voc << endl;
            cout << script_label +  "Saving vocabulary..." << endl;
            //voc.save(savePath + "/" + descriptorName + "_DBoW2_voc.yml.gz");
            voc.saveToTextFile(savePath + "/" + descriptorName + "_DBoW2_voc.txt");
            break;
        }
    }

    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    double tduration = std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
    cout << "        finished (" + std::to_string(tduration) + " s). " << endl;
}

void testNonBinaryVocCreation(const vector<vector<vector<float>>> &features){
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

    cout << script_label + "Creating a " << k << "^" << L << " vocabulary..." << endl;
    switch (descriptorId) { // create non binary vocabulary
        case DESC_ANYFEATNONBIN:{
            AnyFeatNonBinVocabulary voc(k, L, weight, scoring);
            voc.create(features);
            cout << script_label + "Vocabulary information: " << endl
                 << voc << endl;
            cout << script_label +  "Saving vocabulary..." << endl;
            voc.saveToTextFile(savePath + "/AnyFeatNonBin_DBoW2_voc.txt");
            break;
        }
        case DESC_SIFT128:{
            Sift128Vocabulary voc(k, L, weight, scoring);
            voc.create(features);
            cout << script_label + "Vocabulary information: " << endl
                 << voc << endl;
            cout << script_label +  "Saving vocabulary..." << endl;
            voc.saveToTextFile(savePath + "/Sift128_DBoW2_voc.txt");
            break;
        }
        case DESC_SURF64:{
            Surf64Vocabulary voc(k, L, weight, scoring);
            voc.create(features);
            cout << script_label + "Vocabulary information: " << endl
                 << voc << endl;
            cout << script_label +  "Saving vocabulary..." << endl;
            voc.saveToTextFile(savePath + "/Surf64_DBoW2_voc.txt");
            break;
        }
    }

    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    double tduration = std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
    cout << "        finished (" + std::to_string(tduration) + " s). " << endl;
}

// ----------------------------------------------------------------------------

std::vector<std::vector<std::string>> read_txt(const std::string &filePath, const size_t &numCols, char delimiter ,int headerRows) {
    std::vector<std::vector<std::string>> txtFile{};
    std::ifstream file(filePath);
    std::string line, word;

    if (file.good()) {
        file.clear();
        file.seekg(0, std::ios::beg);

        // Skip the rows of the header
        for (int jRow{0}; jRow < headerRows; jRow++) {
            getline(file, line);
        }

        // Stpre rows and works in txtFile vectors
        while (getline(file, line)) {
            std::stringstream line_stream(line);
            std::vector <std::string> row{};
            for (int jRow{0}; jRow < numCols; jRow++) {
                getline(line_stream, word, delimiter);
                row.push_back(word);
            }
            txtFile.push_back(row);
        }
    } else {
        // throw std::invalid_argument("File not found : " + filePath);
        cout << "File not found : " + filePath<< endl;
    }
    return txtFile;
}

void displayProgressBar(int width, double progressPercentage) {
    int pos = width * progressPercentage;
    std::cout << "[";
    for (int i = 0; i < width; ++i) {
        if (i < pos) std::cout << "=";
        else if (i == pos) std::cout << ">";
        else std::cout << " ";
    }
    std::cout << "] " << int(progressPercentage * 100.0) << " %\r";
    std::cout.flush();
}