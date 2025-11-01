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

#include "Viewer.h"
#include "Utils.h"
#include <pangolin/pangolin.h>

#include <mutex>

namespace ANYFEATURE_VSLAM
{

Viewer::Viewer(std::shared_ptr<System> system, std::shared_ptr<FrameDrawer> frameDrawer,
               std::shared_ptr<MapDrawer> mapDrawer, std::shared_ptr<Tracking> tracker,
               const string &strCalibrationPath, const string &strSettingPath,
               const vector<FeatureType>& featureTypes):
        system(system), frameDrawer(frameDrawer),mapDrawer(mapDrawer), tracker(tracker),
    mbFinishRequested(false), mbFinished(true), mbStopped(true), mbStopRequested(false),
    featureTypes(featureTypes)
{

    cv::FileStorage fCalibration(strCalibrationPath, cv::FileStorage::READ);

    float fps = fCalibration["Camera0.fps"];
    if(fps<1)
        fps=30;
    mT = 1e3/fps;

    imageWidth = fCalibration["Camera0.w"];
    imageHeight = fCalibration["Camera0.h"];
    if(imageWidth<1 || imageHeight<1)
    {
        imageWidth = 640;
        imageHeight = 480;
    }

    cout << endl << "[Viewer.cc] Camera Parameters: " << strCalibrationPath << endl;
    cout << "- w: " << imageWidth << endl;
    cout << "- h: " << imageHeight << endl;
    cout << "- fps: " << fps << endl;
    
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
    mViewpointX = fSettings["Viewer.ViewpointX"];
    mViewpointY = fSettings["Viewer.ViewpointY"];
    mViewpointZ = fSettings["Viewer.ViewpointZ"];
    mViewpointF = fSettings["Viewer.ViewpointF"];

    cout << endl << "[Viewer.cc] Viewer Parameters: " << strSettingPath << endl;
    cout << "- mViewpointX: " << mViewpointX << endl;
    cout << "- mViewpointY: " << mViewpointY << endl;
    cout << "- mViewpointZ: " << mViewpointZ << endl;
    cout << "- mViewpointF: " << mViewpointF << endl; 
}

void Viewer::Run()
{
    mbFinished = false;
    mbStopped = false;

    const float scaleFactor{1.3f};
    const float w = scaleFactor * 1280.0f;
    const float h = scaleFactor * 720.0f;
    const float wS{0.4f};
    const float wS_inv{1.0f - wS};

    pangolin::CreateWindowAndBind("AnyFeature-VSLAM (" + featureName(featureTypes[0]) + ") : Map Viewer",w,h);

    // 3D Mouse handler requires depth testing to be enabled
    glEnable(GL_DEPTH_TEST);

    // Issue specific OpenGl we might need
    glEnable (GL_BLEND);
    glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);


    // Define Camera Render Object (for view / scene browsing)
    pangolin::OpenGlRenderState s_cam(
                pangolin::ProjectionMatrix(wS_inv * w, h ,mViewpointF,mViewpointF, wS_inv * w/2.0f, h/2.0f,0.1,1000),
                pangolin::ModelViewLookAt(mViewpointX,mViewpointY,mViewpointZ, 0,0,0,0.0,-1.0, 0.0)
                );

    pangolin::OpenGlRenderState s_cam_aerial(
            pangolin::ProjectionMatrix(wS * w, h /2.0f ,mViewpointF,mViewpointF, wS * w / 2.0f, h /4.0f ,0.1,1000),
            pangolin::ModelViewLookAt(0,-1.2,-0.001, 0,0,0,0.0,-1.0, 0.0)
    );

    pangolin::View& d_img = pangolin::Display("img")
            .SetBounds(0.5,  0.99, 0.01, wS);

    pangolin::View& d_cam_aerial = pangolin::CreateDisplay()
            .SetBounds(0.0,  0.5, 0.01 , wS)
            .SetHandler(new pangolin::Handler3D(s_cam_aerial)
            );

    pangolin::View& d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, wS ,1.0)
            .SetHandler(new pangolin::Handler3D(s_cam)
            );

    pangolin::OpenGlMatrix Twc;
    Twc.SetIdentity();

    pangolin::OpenGlMatrix Twc_aerial;
    Twc_aerial.SetIdentity();

    //cv::namedWindow("ORB-SLAM2: Current Frame");
    vec3f trajectoryCenter0{vec3f::Zero()};
    float cameraHeight0{1.0f};

    cv::Mat im = frameDrawer->DrawFrame();
    const int width =  640;
    int height =  (int) (float(imageHeight) * (float(width) / float(imageWidth)));
    pangolin::GlTexture imageTexture  = pangolin::GlTexture(width,height,GL_RGB,false,0,GL_RGB,GL_UNSIGNED_BYTE);
    float ratio = float(height) * 0.5f / 480.0;
    d_img.SetBounds(1.0f - ratio ,  0.99, 0.01, wS);
    //float margin = (0.5f - ratio) * 0.5f;
    //d_cam_aerial.SetBounds(margin,  0.5f + margin, 0.01 , wS);


    while(1)
    {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glClearColor(1.0f,1.0f,1.0f,1.0f);

        mapDrawer->GetCurrentOpenGLCameraMatrix(Twc);

        d_cam.Activate(s_cam);
        mapDrawer->DrawCurrentCamera(Twc);
        mapDrawer->DrawKeyFrames(true,true);
        mapDrawer->DrawMapPoints();
        s_cam.Follow(Twc);

        im = frameDrawer->DrawFrame();
        cv::Mat imResized;
        cv::resize(im,imResized,cv::Size(width,height));
        ///////////////////////////////////////////////////////// ------------
        /*static int img_idx = 0;
        std::stringstream filename;
        filename << std::setw(5) << std::setfill('0') << img_idx;
        cv::imwrite("/home/fontan/imgs/" + filename.str() + "_rgb.png", imResized);
        img_idx++;*/

        /////////////////////////////////////////////////////////
        cv::flip(imResized.clone(),imResized,0);

        d_img.Activate();
        glColor3f(1.0,1.0,1.0);
        imageTexture.Upload(imResized.data,GL_RGB,GL_UNSIGNED_BYTE);
        imageTexture.RenderToViewport();

        ///////////////////////////////////////////////////////// ------------
        //d_cam.SaveOnRender("/home/fontan/imgs/" + filename.str() + "_map.png");
        //pangolin::SaveWindowOnRender("/home/fontan/imgs/a",d_cam.v);
        pangolin::FinishFrame();

        /*pangolin::Image<unsigned char> buffer;
        pangolin::VideoPixelFormat fmt = pangolin::VideoFormatFromString("RGBA32");
        buffer.Alloc(d_cam.v.w, d_cam.v.h, d_cam.v.w * fmt.bpp/8 );
        glReadBuffer(GL_BACK);
        glPixelStorei(GL_PACK_ALIGNMENT, 1);
        glReadPixels(d_cam.v.l, d_cam.v.b, d_cam.v.w, d_cam.v.h, GL_RGBA, GL_UNSIGNED_BYTE, buffer.ptr );
        cv::Mat img, imgBuffer = cv::Mat(d_cam.v.h, d_cam.v.w, CV_8UC4, buffer.ptr);
        cv::cvtColor(imgBuffer, img,  cv::COLOR_RGBA2BGR);
        cv::flip(imgBuffer.clone(), imgBuffer, 0);
        cv::imwrite("/home/fontan/imgs/" + filename.str() + "_map.png", imgBuffer);*/

        //cv::flip(imagen, img, 0);
        //cv::imshow("some window", img);

        //////////////////////////////////////////////////////



        if(Stop())
        {
            while(isStopped())
            {
                usleep(3000);
            }
        }

        if(CheckFinish())
            break;
    }

    SetFinish();
}

void Viewer::RequestFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinishRequested = true;
}

bool Viewer::CheckFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinishRequested;
}

void Viewer::SetFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinished = true;
}

bool Viewer::isFinished()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinished;
}

void Viewer::RequestStop()
{
    unique_lock<mutex> lock(mMutexStop);
    if(!mbStopped)
        mbStopRequested = true;
}

bool Viewer::isStopped()
{
    unique_lock<mutex> lock(mMutexStop);
    return mbStopped;
}

bool Viewer::Stop()
{
    unique_lock<mutex> lock(mMutexStop);
    unique_lock<mutex> lock2(mMutexFinish);

    if(mbFinishRequested)
        return false;
    else if(mbStopRequested)
    {
        mbStopped = true;
        mbStopRequested = false;
        return true;
    }

    return false;

}

void Viewer::Release()
{
    unique_lock<mutex> lock(mMutexStop);
    mbStopped = false;
}

}
