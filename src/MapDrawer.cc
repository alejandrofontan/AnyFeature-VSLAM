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

#include "MapDrawer.h"
#include "MapPoint.h"
#include "KeyFrame.h"
#include "Converter.h"
#include "Utils.h"

#include <pangolin/pangolin.h>
#include <mutex>

namespace ANYFEATURE_VSLAM
{


MapDrawer::MapDrawer(shared_ptr<Map> pMap, const string &strSettingPath, const vector<FeatureType>& featureTypes):
    mpMap(pMap),featureTypes(featureTypes)
{
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);

    if (!fSettings["Viewer.KeyFrameSize"].empty())
        mKeyFrameSize = fSettings["Viewer.KeyFrameSize"];
    if (!fSettings["Viewer.KeyFrameLineWidth"].empty())
        mKeyFrameLineWidth = fSettings["Viewer.KeyFrameLineWidth"];
    if (!fSettings["Viewer.GraphLineWidth"].empty())
        mGraphLineWidth = fSettings["Viewer.GraphLineWidth"];
    if (!fSettings["Viewer.PointSize"].empty())
        mPointSize = fSettings["Viewer.PointSize"];
    if (!fSettings["Viewer.CameraSize"].empty())
        mCameraSize = fSettings["Viewer.CameraSize"];
    if (!fSettings["Viewer.CameraLineWidth"].empty())
        mCameraLineWidth = fSettings["Viewer.CameraLineWidth"];
}

void MapDrawer::DrawMapPoints()
{
    const vector<Pt> &vpMPs = mpMap->GetAllMapPoints();
    const vector<Pt> &vpRefMPs = mpMap->GetReferenceMapPoints();

    set<Pt> spRefMPs(vpRefMPs.begin(), vpRefMPs.end());

    if(vpMPs.empty())
        return;

    glPointSize(mPointSize);
    glBegin(GL_POINTS);
    glColor3f(0.0,0.0,0.0);

    for(size_t i=0, iend=vpMPs.size(); i<iend;i++)
    {
        if(vpMPs[i]->isBad() || spRefMPs.count(vpMPs[i]))
            continue;
        vec3f pos = vpMPs[i]->GetWorldPos();
        glVertex3f(pos(0),pos(1),pos(2));
    }
    glEnd();

    glPointSize(mPointSize);
    glBegin(GL_POINTS);
    cv::Scalar color = getFeatureColor(featureTypes[0],0);
    glColor3f(GLfloat(color[0]),GLfloat(color[1]),GLfloat(color[2]));

    for(set<Pt>::iterator sit=spRefMPs.begin(), send=spRefMPs.end(); sit!=send; sit++)
    {
        if((*sit)->isBad())
            continue;
        vec3f pos = (*sit)->GetWorldPos();
        glVertex3f(pos(0),pos(1),pos(2));

    }

    glEnd();
}

void MapDrawer::DrawKeyFrames(const bool bDrawKF, const bool bDrawGraph)
{
    const float &w = mKeyFrameSize;
    const float h = w*0.75;
    const float z = w*0.6;

    const vector<Keyframe> vpKFs = mpMap->GetAllKeyFrames();

    if(bDrawKF)
    {
        for(size_t i=0; i<vpKFs.size(); i++)
        {
            Keyframe pKF = vpKFs[i];
            mat4f Twc_tmp = pKF->GetPoseInverse().transpose();
            cv::Mat Twc = Converter::toCvMat(Twc_tmp);

            glPushMatrix();

            glMultMatrixf(Twc.ptr<GLfloat>(0));

            glLineWidth(mKeyFrameLineWidth);
            glColor3f(0.0f,0.0f,1.0f);
            glBegin(GL_LINES);
            glVertex3f(0,0,0);
            glVertex3f(w,h,z);
            glVertex3f(0,0,0);
            glVertex3f(w,-h,z);
            glVertex3f(0,0,0);
            glVertex3f(-w,-h,z);
            glVertex3f(0,0,0);
            glVertex3f(-w,h,z);

            glVertex3f(w,h,z);
            glVertex3f(w,-h,z);

            glVertex3f(-w,h,z);
            glVertex3f(-w,-h,z);

            glVertex3f(-w,h,z);
            glVertex3f(w,h,z);

            glVertex3f(-w,-h,z);
            glVertex3f(w,-h,z);
            glEnd();

            glPopMatrix();
        }
    }

    if(bDrawGraph)
    {
        glLineWidth(mGraphLineWidth);
        glColor4f(0.0f,1.0f,0.0f,0.6f);
        glBegin(GL_LINES);

        for(size_t i=0; i<vpKFs.size(); i++)
        {
            // Covisibility Graph
            const vector<Keyframe > vCovKFs = vpKFs[i]->GetCovisiblesByWeight(100);
            vec3f Ow = vpKFs[i]->GetCameraCenter();
            if(!vCovKFs.empty())
            {
                for(vector<Keyframe >::const_iterator vit=vCovKFs.begin(), vend=vCovKFs.end(); vit!=vend; vit++)
                {
                    if((*vit)->keyId < vpKFs[i]->keyId)
                        continue;
                    vec3f Ow2 = (*vit)->GetCameraCenter();
                    glVertex3f(Ow(0),Ow(1),Ow(2));
                    glVertex3f(Ow2(0),Ow2(1),Ow2(2));
                }
            }

            // Spanning tree
            Keyframe pParent = vpKFs[i]->GetParent();
            if(pParent)
            {
                vec3f Owp = pParent->GetCameraCenter();
                glVertex3f(Ow(0),Ow(1),Ow(2));
                glVertex3f(Owp(0),Owp(1),Owp(2));
            }

            // Loops
            set<Keyframe > sLoopKFs = vpKFs[i]->GetLoopEdges();
            for(set<Keyframe>::iterator sit=sLoopKFs.begin(), send=sLoopKFs.end(); sit!=send; sit++)
            {
                if((*sit)->keyId < vpKFs[i]->keyId)
                    continue;
                vec3f Owl = (*sit)->GetCameraCenter();
                glVertex3f(Ow(0),Ow(1),Ow(2));
                glVertex3f(Owl(0),Owl(1),Owl(2));
            }
        }

        glEnd();
    }
}

void MapDrawer::DrawTrajectory(vec3f& trajectoryCenter, float& cameraHeight) {
    {
        const vector<Keyframe> keyframes = mpMap->GetAllKeyFrames();
        glPointSize(10.0f);
        glColor3f(0.0, 0.0, 1.0);
        glBegin(GL_POINTS);

        // Draw Keyframes
        trajectoryCenter = vec3f::Zero();
        for (auto& keyframe: keyframes) {
            vec3f twc = keyframe->GetCameraCenter();
            glVertex3f(twc(0), twc(1), twc(2));
            trajectoryCenter += twc;
        }
        if(!keyframes.empty())
            trajectoryCenter /= keyframes.size();

        glEnd();
    }
    {
        float maxDistance{1.0};
        for (auto& twc: trajectory) {
            float distance = (twc - trajectoryCenter).norm() + 1.0f;
            if(distance > maxDistance)
                maxDistance = distance;
        }
        cameraHeight = 1.0f * maxDistance;
    }
    {
        // Draw Loop Closures
        glPointSize(20.0f);
        glColor3f(1.0, 0.0, 0.0);
        glBegin(GL_POINTS);

        for (auto& twc: loopClosures)
            glVertex3f(twc(0), twc(1), twc(2));

        glEnd();
    }
    {
        // Draw Camera
        glPointSize(20.0f);
        glColor3f(0.0, 1.0, 0.0);
        glBegin(GL_POINTS);

        vec3f twc;
        {
            unique_lock<mutex> lock(mMutexCamera);
            mat3f Rwc = mCameraPose.block<3,3>(0,0).transpose();
            twc = -Rwc * mCameraPose.block<3,1>(0,3);
        }
        glVertex3f(twc(0), twc(1), twc(2));

        glEnd();
    }
}

void MapDrawer::DrawCurrentCamera(pangolin::OpenGlMatrix &Twc)
{
    const float &w = mCameraSize;
    const float h = w*0.75;
    const float z = w*0.6;

    glPushMatrix();

#ifdef HAVE_GLES
        glMultMatrixf(Twc.m);
#else
        glMultMatrixd(Twc.m);
#endif

    glLineWidth(mCameraLineWidth);
    glColor3f(0.0f,1.0f,0.0f);
    glBegin(GL_LINES);
    glVertex3f(0,0,0);
    glVertex3f(w,h,z);
    glVertex3f(0,0,0);
    glVertex3f(w,-h,z);
    glVertex3f(0,0,0);
    glVertex3f(-w,-h,z);
    glVertex3f(0,0,0);
    glVertex3f(-w,h,z);

    glVertex3f(w,h,z);
    glVertex3f(w,-h,z);

    glVertex3f(-w,h,z);
    glVertex3f(-w,-h,z);

    glVertex3f(-w,h,z);
    glVertex3f(w,h,z);

    glVertex3f(-w,-h,z);
    glVertex3f(w,-h,z);
    glEnd();

    glPopMatrix();
}


void MapDrawer::SetCurrentCameraPose(const mat4f &Tcw_)
{
    unique_lock<mutex> lock(mMutexCamera);
    mCameraPose = Tcw_;
    trajectory.push_back(Tcw_.block<3,1>(0,3));
}

void MapDrawer::AddLoopClosureKeyframe(const mat4f &Tcw_)
{
    unique_lock<mutex> lock(mutexLoopClosures);
    loopClosures.emplace_back(Tcw_.block<3,1>(0,3));
}

void MapDrawer::GetCurrentOpenGLCameraMatrix(pangolin::OpenGlMatrix &M)
{
    if(mCameraPose(3,3) == 1.0f)
    {
        mat3f Rwc{};
        vec3f twc{};
        {
            unique_lock<mutex> lock(mMutexCamera);
            Rwc = mCameraPose.block<3,3>(0,0).transpose();
            twc = -Rwc * mCameraPose.block<3,1>(0,3);
        }

        SetCurrentOpenGLCameraMatrix(Rwc,twc, M);
    }
    else
        M.SetIdentity();
}
void MapDrawer::SetCurrentOpenGLCameraMatrix(const mat3f& Rwc,const vec3f& twc, pangolin::OpenGlMatrix &M){
    M.m[0] = Rwc(0,0);
    M.m[1] = Rwc(1,0);
    M.m[2] = Rwc(2,0);
    M.m[3]  = 0.0;

    M.m[4] = Rwc(0,1);
    M.m[5] = Rwc(1,1);
    M.m[6] = Rwc(2,1);
    M.m[7]  = 0.0;

    M.m[8] = Rwc(0,2);
    M.m[9] = Rwc(1,2);
    M.m[10] = Rwc(2,2);
    M.m[11]  = 0.0;

    M.m[12] = twc(0);
    M.m[13] = twc(1);
    M.m[14] = twc(2);
    M.m[15]  = 1.0;
}

} //namespace ORB_SLAM
