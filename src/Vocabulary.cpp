//
// Created by fontan on 18/03/24.
//

#include "Vocabulary.h"
#include "Converter.h"

ANYFEATURE_VSLAM::Vocabulary::Vocabulary(const string vocabularyFolder, const DescriptorType descriptorType):
        vocabularyFolder(vocabularyFolder), descriptorType(descriptorType)
{}

void ANYFEATURE_VSLAM::Vocabulary::createVocabulary(){
    switch(descriptorType) {
        // Create Vocabulary
        case DESC_ANYFEATNONBIN: {
            anyFeatNonBinVocabulary = make_shared<AnyFeatNonBinVocabulary>();
            return;
        }
        case DESC_ANYFEATBIN: {
            anyFeatBinVocabulary = make_shared<AnyFeatBinVocabulary>();
            return;
        }
        case DESC_R2D2: {
            r2d2Vocabulary = make_shared<R2d2Vocabulary>();
            return;
        }
        case DESC_SIFT128: {
            sift128Vocabulary = make_shared<Sift128Vocabulary>();
            return;
        }
        case DESC_KAZE64: {
            kaze64Vocabulary = make_shared<Kaze64Vocabulary>();
            return;
        }
        case DESC_SURF64: {
            surf64Vocabulary = make_shared<Surf64Vocabulary>();
            return;
        }
        case DESC_BRISK: {
            briskVocabulary = make_shared<BriskVocabulary>();
            return;
        }
        case DESC_AKAZE61: {
            akaze61Vocabulary = make_shared<Akaze61Vocabulary>();
            return;
        }
        case DESC_ORB: {
            orbVocabulary = make_shared<OrbVocabulary>();
            return;
        }
    }
}

bool ANYFEATURE_VSLAM::Vocabulary::loadFromTextFile(){
    switch(descriptorType) {
        // load from text file
        case DESC_ANYFEATNONBIN:{
            cout << "Loading AnyFeatNonBin Vocabulary. This could take a while..." << endl;
            string vocabulary_path = vocabularyFolder + "/AnyFeatNonBin_DBoW2_voc.txt";
            return anyFeatNonBinVocabulary->loadFromTextFile(vocabulary_path);
        }
        case DESC_ANYFEATBIN:{
            cout << "Loading AnyFeatBin Vocabulary. This could take a while..." << endl;
            string vocabulary_path = vocabularyFolder + "/AnyFeatBin_DBoW2_voc.txt";
            return anyFeatBinVocabulary->loadFromTextFile(vocabulary_path);
        }
        case DESC_R2D2:{
            cout << "Loading R2d2 Vocabulary. This could take a while..." << endl;
            string vocabulary_path = vocabularyFolder + "/R2d2_DBoW2_voc.txt";
            return r2d2Vocabulary->loadFromTextFile(vocabulary_path);
        }
        case DESC_SIFT128:{
            cout << "Loading Sift128 Vocabulary. This could take a while..." << endl;
            string vocabulary_path = vocabularyFolder + "/Sift128_DBoW2_voc.txt";
            return sift128Vocabulary->loadFromTextFile(vocabulary_path);
        }
        case DESC_KAZE64:{
            cout << "Loading Kaze64 Vocabulary. This could take a while..." << endl;
            string vocabulary_path = vocabularyFolder + "/Kaze64_DBoW2_voc.txt";
            return kaze64Vocabulary->loadFromTextFile(vocabulary_path);
        }
        case DESC_SURF64:{
            cout << "Loading Surf64 Vocabulary. This could take a while..." << endl;
            string vocabulary_path = vocabularyFolder + "/Surf64_DBoW2_voc.txt";
            return surf64Vocabulary->loadFromTextFile(vocabulary_path);
        }
        case DESC_BRISK:{
            string vocabulary_path = vocabularyFolder + "/Brisk_DBoW2_voc.txt";
            cout << "Loading Brisk Vocabulary from: " + vocabulary_path << endl;
            cout << "This could take a while ..." << endl;
            return briskVocabulary->loadFromTextFile(vocabulary_path);
        }
        case DESC_AKAZE61:{
            string vocabulary_path = vocabularyFolder + "/Akaze61_DBoW2_voc.txt";
            cout << "Loading Akaze61 Vocabulary from: " + vocabulary_path << endl;
            cout << "This could take a while ..." << endl;
            return akaze61Vocabulary->loadFromTextFile(vocabulary_path);
        }
        case DESC_ORB:{
            string vocabulary_path = vocabularyFolder + "/ORBvoc.txt";
            cout << "Loading Orb Vocabulary from: " + vocabulary_path << endl;
            cout << "This could take a while ..." << endl;
            return orbVocabulary->loadFromTextFile(vocabulary_path);
        }
    }
}

unsigned int ANYFEATURE_VSLAM::Vocabulary::size(){
    switch(descriptorType) {
        // size
        case DESC_ANYFEATNONBIN:
            return anyFeatNonBinVocabulary->size();
        case DESC_ANYFEATBIN:
            return anyFeatBinVocabulary->size();
        case DESC_R2D2:
            return r2d2Vocabulary->size();
        case DESC_SIFT128:
            return sift128Vocabulary->size();
        case DESC_KAZE64:
            return kaze64Vocabulary->size();
        case DESC_SURF64:
            return surf64Vocabulary->size();
        case DESC_BRISK:
            return briskVocabulary->size();
        case DESC_AKAZE61:
            return akaze61Vocabulary->size();
        case DESC_ORB:
            return orbVocabulary->size();
    }
}

double ANYFEATURE_VSLAM::Vocabulary::score(const DBoW2::BowVector &BowVec_1, const DBoW2::BowVector &BowVec_2){
    switch(descriptorType) {
        // score
        case DESC_ANYFEATNONBIN:
            return anyFeatNonBinVocabulary->score(BowVec_1,BowVec_2);
        case DESC_ANYFEATBIN:
            return anyFeatBinVocabulary->score(BowVec_1,BowVec_2);
        case DESC_R2D2:
            return r2d2Vocabulary->score(BowVec_1,BowVec_2);
        case DESC_SIFT128:
            return sift128Vocabulary->score(BowVec_1,BowVec_2);
        case DESC_KAZE64:
            return kaze64Vocabulary->score(BowVec_1,BowVec_2);
        case DESC_SURF64:
            return surf64Vocabulary->score(BowVec_1,BowVec_2);
        case DESC_BRISK:
            return briskVocabulary->score(BowVec_1,BowVec_2);
        case DESC_AKAZE61:
            return akaze61Vocabulary->score(BowVec_1,BowVec_2);
        case DESC_ORB:
            return orbVocabulary->score(BowVec_1, BowVec_2);
    }
}

void ANYFEATURE_VSLAM::Vocabulary::transform(
        const cv::Mat& mDescriptors,  DBoW2::BowVector& mBowVec,DBoW2::FeatureVector& mFeatVec){
    switch(descriptorType) {
        // transform
        case DESC_ANYFEATNONBIN:{
            vector<vector<float>> vCurrentDesc = Converter::toDescriptorVector_float(mDescriptors);
            anyFeatNonBinVocabulary->transform(vCurrentDesc,mBowVec,mFeatVec,4);
            return;
        }
        case DESC_ANYFEATBIN:{
            vector<cv::Mat> vCurrentDesc = Converter::toDescriptorVector_mat(mDescriptors);
            anyFeatBinVocabulary->transform(vCurrentDesc,mBowVec,mFeatVec,4);
            return;
        }
        case DESC_R2D2:{
            vector<vector<float>> vCurrentDesc = Converter::toDescriptorVector_float(mDescriptors);
            r2d2Vocabulary->transform(vCurrentDesc,mBowVec,mFeatVec,4);
            return;
        }
        case DESC_SIFT128:{
            vector<vector<float>> vCurrentDesc = Converter::toDescriptorVector_float(mDescriptors);
            sift128Vocabulary->transform(vCurrentDesc,mBowVec,mFeatVec,4);
            return;
        }
        case DESC_KAZE64:{
            vector<vector<float>> vCurrentDesc = Converter::toDescriptorVector_float(mDescriptors);
            kaze64Vocabulary->transform(vCurrentDesc,mBowVec,mFeatVec,4);
            return;
        }
        case DESC_SURF64:{
            vector<vector<float>> vCurrentDesc = Converter::toDescriptorVector_float(mDescriptors);
            surf64Vocabulary->transform(vCurrentDesc,mBowVec,mFeatVec,4);
            return;
        }
        case DESC_BRISK:{
            vector<cv::Mat> vCurrentDesc = Converter::toDescriptorVector_mat(mDescriptors);
            briskVocabulary->transform(vCurrentDesc,mBowVec,mFeatVec,4);
            return;
        }
        case DESC_AKAZE61:{
            vector<cv::Mat> vCurrentDesc = Converter::toDescriptorVector_mat(mDescriptors);
            akaze61Vocabulary->transform(vCurrentDesc,mBowVec,mFeatVec,4);
            return;
        }
        case DESC_ORB:{
            vector<cv::Mat> vCurrentDesc = Converter::toDescriptorVector_mat(mDescriptors);
            orbVocabulary->transform(vCurrentDesc,mBowVec,mFeatVec,4);
            return;
        }
    }
}

