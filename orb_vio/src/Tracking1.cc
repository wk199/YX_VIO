/**
* This file is part of ORB-SLAM.
*
* Copyright (C) 2014 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <http://webdiis.unizar.es/~raulmur/orbslam/>
*
* ORB-SLAM is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM. If not, see <http://www.gnu.org/licenses/>.
*/

#include "Tracking.h"
//#include<ros/ros.h>
//#include <cv_bridge/cv_bridge.h>

#include<opencv2/opencv.hpp>
#include "opencv2/highgui/highgui.hpp"

#include"ORBmatcher.h"
//#include"FramePublisher.h"
#include"Converter.h"
#include"Map.h"
#include"Initializer.h"

#include"Optimizer.h"
#include"PnPsolver.h"

#include<iostream>
#include<fstream>


using namespace std;
using namespace cv;

unsigned char           * g_pRgbBuffer;     //处理后数据缓存区
cv::Mat Iimag;
namespace ORB_SLAM
{


Tracking::Tracking(ORBVocabulary* pVoc, FrameDrawer *pFrameDrawer, MapDrawer *pMapDrawer,
		Map *pMap, string strSettingPath):mState(NO_IMAGES_YET), mpORBVocabulary(pVoc),
	mpFrameDrawer(pFrameDrawer), mpMapDrawer(pMapDrawer),mpMap(pMap),keyframeNum(0),
    mnLastRelocFrameId(0), mbPublisherStopped(false), mbReseting(false), mbForceRelocalisation(false), mbMotionModel(false),needInitialization(false)
{
    // Load camera parameters from settings file
	tkCount = 0;
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
    float fx = fSettings["Camera.fx"];
    float fy = fSettings["Camera.fy"];
    float cx = fSettings["Camera.cx"];
    float cy = fSettings["Camera.cy"];

    cv::Mat K = cv::Mat::eye(3,3,CV_32F);
    K.at<float>(0,0) = fx;
    K.at<float>(1,1) = fy;
    K.at<float>(0,2) = cx;
    K.at<float>(1,2) = cy;
    K.copyTo(mK);

    cv::Mat DistCoef(4,1,CV_32F);
    DistCoef.at<float>(0) = fSettings["Camera.k1"];
    DistCoef.at<float>(1) = fSettings["Camera.k2"];
    DistCoef.at<float>(2) = fSettings["Camera.p1"];
    DistCoef.at<float>(3) = fSettings["Camera.p2"];
    DistCoef.copyTo(mDistCoef);

    float fps = fSettings["Camera.fps"];
    if(fps==0)
        fps=30;

    // Max/Min Frames to insert keyframes and to check relocalisation
    mMinFrames = 0;
    mMaxFrames = 18*fps/30;


    cout << "Camera Parameters: " << endl;
    cout << "- fx: " << fx << endl;
    cout << "- fy: " << fy << endl;
    cout << "- cx: " << cx << endl;
    cout << "- cy: " << cy << endl;
    cout << "- k1: " << DistCoef.at<float>(0) << endl;
    cout << "- k2: " << DistCoef.at<float>(1) << endl;
    cout << "- p1: " << DistCoef.at<float>(2) << endl;
    cout << "- p2: " << DistCoef.at<float>(3) << endl;
    cout << "- fps: " << fps << endl;


    int nRGB = fSettings["Camera.RGB"];
    mbRGB = nRGB;

    if(mbRGB)
        cout << "- color order: RGB (ignored if grayscale)" << endl;
    else
        cout << "- color order: BGR (ignored if grayscale)" << endl;

    // Load ORB parameters

    int nFeatures = fSettings["ORBextractor.nFeatures"];
    float fScaleFactor = fSettings["ORBextractor.scaleFactor"];
    int nLevels = fSettings["ORBextractor.nLevels"];
    int fastTh = fSettings["ORBextractor.fastTh"];    
    int Score = fSettings["ORBextractor.nScoreType"];

    assert(Score==1 || Score==0);

    mpORBextractor = new ORBextractor(nFeatures,fScaleFactor,nLevels,Score,fastTh);

    cout << endl  << "ORB Extractor Parameters: " << endl;
    cout << "- Number of Features: " << nFeatures << endl;
    cout << "- Scale Levels: " << nLevels << endl;
    cout << "- Scale Factor: " << fScaleFactor << endl;
    cout << "- Fast Threshold: " << fastTh << endl;
    if(Score==0)
        cout << "- Score: HARRIS" << endl;
    else
        cout << "- Score: FAST" << endl;


    // ORB extractor for initialization
    // Initialization uses only points from the finest scale level
    mpIniORBextractor = new ORBextractor(nFeatures*2,1.2,8,Score,fastTh);  

    int nMotion = fSettings["UseMotionModel"];
    mbMotionModel = nMotion;

    if(mbMotionModel)
    {
        mVelocity = cv::Mat::eye(4,4,CV_32F);
        cout << endl << "Motion Model: Enabled" << endl << endl;
    }
    else
        cout << endl << "Motion Model: Disabled (not recommended, change settings UseMotionModel: 1)" << endl << endl;

    iCameraCounts = 1;
    iStatus=-1;
    iDisplayFrames = 10000;
    iplImage = NULL;
    channel=3;
     CameraSdkInit(1);

    //枚举设备，并建立设备列表
    CameraEnumerateDevice(&tCameraEnumList,&iCameraCounts);

    //没有连接设备
    if(iCameraCounts==0){
        return;
    }

    //相机初始化。初始化成功后，才能调用任何其他相机相关的操作接口
    iStatus = CameraInit(&tCameraEnumList,-1,-1,&hCamera);

    //初始化失败
    if(iStatus!=CAMERA_STATUS_SUCCESS){
        return;
    }

    //获得相机的特性描述结构体。该结构体中包含了相机可设置的各种参数的范围信息。决定了相关函数的参数
    CameraGetCapability(hCamera,&tCapability);

    //
    g_pRgbBuffer = (unsigned char*)malloc(tCapability.sResolutionRange.iHeightMax*tCapability.sResolutionRange.iWidthMax*3);
    //g_readBuf = (unsigned char*)malloc(tCapability.sResolutionRange.iHeightMax*tCapability.sResolutionRange.iWidthMax*3);

    /*让SDK进入工作模式，开始接收来自相机发送的图像
    数据。如果当前相机是触发模式，则需要接收到
    触发帧以后才会更新图像。    */
    CameraPlay(hCamera);

    /*其他的相机参数设置
    例如 CameraSetExposureTime   CameraGetExposureTime  设置/读取曝光时间
         CameraSetImageResolution  CameraGetImageResolution 设置/读取分辨率
         CameraSetGamma、CameraSetConrast、CameraSetGain等设置图像伽马、对比度、RGB数字增益等等。
         更多的参数的设置方法，，清参考MindVision_Demo。本例程只是为了演示如何将SDK中获取的图像，转成OpenCV的图像格式,以便调用OpenCV的图像处理函数进行后续开发
    */

    if(tCapability.sIspCapacity.bMonoSensor){
        channel=1;
        CameraSetIspOutFormat(hCamera,CAMERA_MEDIA_TYPE_MONO8);
    }else{
        channel=3;
        CameraSetIspOutFormat(hCamera,CAMERA_MEDIA_TYPE_BGR8);
    }
    
    initialNum = 10;
 //   tf::Transform tfT;
  //  tfT.setIdentity();
  //  mTfBr.sendTransform(tf::StampedTransform(tfT,ros::Time::now(), "/ORB_SLAM/World", "/ORB_SLAM/Camera"));
}

Tracking::Tracking(ORBVocabulary* pVoc, FrameDrawer *pFrameDrawer, MapDrawer *pMapDrawer,
		Map *pMap, string strSettingPath, cv::VideoCapture& cap):mState(NO_IMAGES_YET), mpORBVocabulary(pVoc),
	mpFrameDrawer(pFrameDrawer), mpMapDrawer(pMapDrawer),mpMap(pMap),
    mnLastRelocFrameId(0), mbPublisherStopped(false), mbReseting(false), mbForceRelocalisation(false), mbMotionModel(false), mCamera(cap),needInitialization(false)
{
    // Load camera parameters from settings file
	tkCount = 0;
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
    float fx = fSettings["Camera.fx"];
    float fy = fSettings["Camera.fy"];
    float cx = fSettings["Camera.cx"];
    float cy = fSettings["Camera.cy"];

    cv::Mat K = cv::Mat::eye(3,3,CV_32F);
    K.at<float>(0,0) = fx;
    K.at<float>(1,1) = fy;
    K.at<float>(0,2) = cx;
    K.at<float>(1,2) = cy;
    K.copyTo(mK);

    cv::Mat DistCoef(4,1,CV_32F);
    DistCoef.at<float>(0) = fSettings["Camera.k1"];
    DistCoef.at<float>(1) = fSettings["Camera.k2"];
    DistCoef.at<float>(2) = fSettings["Camera.p1"];
    DistCoef.at<float>(3) = fSettings["Camera.p2"];
    DistCoef.copyTo(mDistCoef);

    float fps = fSettings["Camera.fps"];
    if(fps==0)
        fps=30;

    // Max/Min Frames to insert keyframes and to check relocalisation
    mMinFrames = 0;
    mMaxFrames = 18*fps/30;


    cout << "Camera Parameters: " << endl;
    cout << "- fx: " << fx << endl;
    cout << "- fy: " << fy << endl;
    cout << "- cx: " << cx << endl;
    cout << "- cy: " << cy << endl;
    cout << "- k1: " << DistCoef.at<float>(0) << endl;
    cout << "- k2: " << DistCoef.at<float>(1) << endl;
    cout << "- p1: " << DistCoef.at<float>(2) << endl;
    cout << "- p2: " << DistCoef.at<float>(3) << endl;
    cout << "- fps: " << fps << endl;


    int nRGB = fSettings["Camera.RGB"];
    mbRGB = nRGB;

    if(mbRGB)
        cout << "- color order: RGB (ignored if grayscale)" << endl;
    else
        cout << "- color order: BGR (ignored if grayscale)" << endl;

    // Load ORB parameters

    int nFeatures = fSettings["ORBextractor.nFeatures"];
    float fScaleFactor = fSettings["ORBextractor.scaleFactor"];
    int nLevels = fSettings["ORBextractor.nLevels"];
    int fastTh = fSettings["ORBextractor.fastTh"];    
    int Score = fSettings["ORBextractor.nScoreType"];

    assert(Score==1 || Score==0);

    mpORBextractor = new ORBextractor(nFeatures,fScaleFactor,nLevels,Score,fastTh);

    cout << endl  << "ORB Extractor Parameters: " << endl;
    cout << "- Number of Features: " << nFeatures << endl;
    cout << "- Scale Levels: " << nLevels << endl;
    cout << "- Scale Factor: " << fScaleFactor << endl;
    cout << "- Fast Threshold: " << fastTh << endl;
    if(Score==0)
        cout << "- Score: HARRIS" << endl;
    else
        cout << "- Score: FAST" << endl;


    // ORB extractor for initialization
    // Initialization uses only points from the finest scale level
    mpIniORBextractor = new ORBextractor(nFeatures*2,1.2,8,Score,fastTh);  

    int nMotion = fSettings["UseMotionModel"];
    mbMotionModel = nMotion;

    if(mbMotionModel)
    {
        mVelocity = cv::Mat::eye(4,4,CV_32F);
        cout << endl << "Motion Model: Enabled" << endl << endl;
    }
    else
        cout << endl << "Motion Model: Disabled (not recommended, change settings UseMotionModel: 1)" << endl << endl;

    iCameraCounts = 1;
    iStatus=-1;
    iDisplayFrames = 10000;
    iplImage = NULL;
    channel=1;
    initialNum = 10;
}


void Tracking::SetLocalMapper(LocalMapping *pLocalMapper)
{
    mpLocalMapper=pLocalMapper;
}

void Tracking::SetLoopClosing(LoopClosing *pLoopClosing)
{
    mpLoopClosing=pLoopClosing;
}
void Tracking::SetIMU(IMUMeasurementList* _imufactor)
{
	mpIMU = _imufactor;
}
void Tracking::SetViewer(Viewer *pViewer)
{
    mpViewer=pViewer;
}
void Tracking::SetKeyFrameDatabase(KeyFrameDatabase *pKFDB)
{
    mpKeyFrameDB = pKFDB;
}
void Tracking::Camera()
{
   
  while(1)
   {
    if(CameraGetImageBuffer(hCamera,&sFrameInfo,&pbyBuffer,1000) == CAMERA_STATUS_SUCCESS)
    {
	   CameraImageProcess(hCamera, pbyBuffer, g_pRgbBuffer,&sFrameInfo);
           if (iplImage)
            {
                cvReleaseImageHeader(&iplImage);
            }
            iplImage = cvCreateImageHeader(cvSize(sFrameInfo.iWidth,sFrameInfo.iHeight),IPL_DEPTH_8U,channel);
            cvSetData(iplImage,g_pRgbBuffer,sFrameInfo.iWidth*channel);//此处只是设置指针，无图像块数据拷贝，不需担心转换效率
            //以下两种方式都可以显示图像或者处理图像
            //  #if 0
            //cvShowImage("OpenCV Demo",iplImage);
           // #else
            Iimag=cv::Mat(iplImage);//这里只是进行指针转换，将IplImage转换成Mat类型
           // #endif;
            //在成功调用CameraGetImageBuffer后，必须调用CameraReleaseImageBuffer来释放获得的buffer。
	    //否则再次调用CameraGetImageBuffer时，程序将被挂起一直阻塞，直到其他线程中调用CameraReleaseImageBuffer来释放了buffer
	    CameraReleaseImageBuffer(hCamera,pbyBuffer);
     }
     GrabImage(Iimag,0.0); 
   }  

}
void Tracking::Run()
{
    Mat frame;
    int fCount = 0;
    int _numCount = 3670;
    while(mCamera.isOpened()){
        mCamera >> frame;
        GrabImage(frame , 0.033333);
        //cout << "grabImage here" << std::endl;
        if(fCount ++ >= _numCount)
            break;    
    }
    
    vector<ORB_SLAM::KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
 	sort(vpKFs.begin(),vpKFs.end(),ORB_SLAM::KeyFrame::lId);
    ofstream fout;
    fout.open("orb_out.txt", ios::app);
    fout <<setiosflags(ios::fixed);
    fout << vpKFs.size()<<endl;
    
    char buf[255];
    for(int i = 0 ; i < vpKFs.size(); i ++){
        ORB_SLAM::KeyFrame* pKF = vpKFs[i];
        if(pKF -> isBad()){
            
        }
        else{
            cv::Mat t_pose = pKF->GetPose();
            t_pose.convertTo(t_pose , CV_32F);
            for(int k1= 0; k1<4;k1++){
                for(int k2 = 0 ; k2 < 4; k2 ++)
                fout << t_pose.at<float>(k1 , k2) << " ";
                fout << endl;
            }
            //fout << endl;
            int index = pKF->mnFrameId;
            cv::Mat img = pKF->GetImage();
            sprintf(buf , "data/img_%d.png" , i);
            imwrite(buf, img);
        }
    }
    fout.close();
}

void Tracking::GrabImage(const cv::Mat& img, double timesmp)
{

    //cv::Mat im;
    mImGray = img;
    if (mImGray.channels() == 3) 
    {
    		if (mbRGB)
    				cvtColor(mImGray, mImGray, CV_RGB2GRAY);
    		else
    				cvtColor(mImGray, mImGray, CV_BGR2GRAY);
    } 
   else if (mImGray.channels() == 1) {
    		mImGray.copyTo(mImGray);
    }


    if(_debug_info == 0){
        //cout << "_debug_info = "<<_debug_info<<endl;
	    time_t sys_now;
	    struct tm* sys_tm;
	    struct timeval tv;
	    time(&sys_now);	
	    gettimeofday(&tv,NULL);
	    sys_tm = localtime(&sys_now);
    	timesmp = sys_tm->tm_mday * 24.0 * 3600.0 + sys_tm->tm_hour * 3600.0 +
	        sys_tm->tm_min * 60.0 + double(sys_tm->tm_sec) + double(tv.tv_usec)/1000000.0;        
    }
    else{
        //cout <<setiosflags(ios::fixed);
        //cout << "_debug_info = "<<_debug_info << endl;
        timesmp = _timestample[tkCount++];
        //cout << "Frame Number is : "<<tkCount<<", timesmp = "<<setprecision(6)<<timesmp<<endl;
    }

    if(mState==WORKING || mState==LOST)
        mCurrentFrame = Frame(mImGray,timesmp,mpORBextractor,mpORBVocabulary,mK,mDistCoef);
    else
        mCurrentFrame = Frame(mImGray,timesmp,mpIniORBextractor,mpORBVocabulary,mK,mDistCoef);

    // Depending on the state of the Tracker we perform different tasks

    if(mState==NO_IMAGES_YET)
    {
        mState = NOT_INITIALIZED;
    }

    mLastProcessedState=mState;

    if(mState==NOT_INITIALIZED)
    {
        FirstInitialization();
    }
    else if(mState==INITIALIZING)
    {
        Initialize();  
        needInitialization = true;         
    }
    else
    {
        // System is initialized. Track Frame.
        //cout << "System is initialized. Track Frame"<<endl;
        if(needInitialization){     
            if(keyframeNum >= initialNum){
                cout << "begin InitialScalityAndVelocity" << std::endl;
                mpLocalMapper->InterruptBA();
                bool isInitialized = Optimizer::InitialScalityAndVelocity(mpMap , mpIMU , &mLastFrame, e_gw);
                if(isInitialized){
                    initialNum = 10;
                    mpLocalMapper->SetOptUseIMU(true);
                    needInitialization = false;
                    mpLastKeyFrame->updateAfterBA = true;
                    cout << "finish InitialScalityAndVelocity" << std::endl;                   
                }
                else{
                    initialNum += 10;
                }

            } 
        }
        
        bool bOK;
        // Initial Camera Pose Estimation from Previous Frame (Motion Model or Coarse) or Relocalisation
        //if(mState==WORKING && !RelocalisationRequested() && needInitialization == true)
        
        /*else if(mState==WORKING && !RelocalisationRequested() && needInitialization == false)
        {
            bOK = TrackUseImu();
        }*/
        if(mState==WORKING && !RelocalisationRequested() && needInitialization == true)
        {
        	
            if(!mbMotionModel || mpMap->KeyFramesInMap()<4 || mVelocity.empty() || mCurrentFrame.mnId<mnLastRelocFrameId+2)
                bOK = TrackPreviousFrame();
            else
            {
                bOK = TrackWithMotionModel();
                if(!bOK)
                    bOK = TrackPreviousFrame();
            }
        	//bOK = TrackPreviousFrame();
        }
        else if(mState==WORKING && !RelocalisationRequested() && needInitialization == false)
        {
            bOK = TrackUseImuPredict();
        }
        else
        {
            bOK = Relocalisation();
        }

        // If we have an initial estimation of the camera pose and matching. Track the local map.
        if(bOK)
            bOK = TrackLocalMap();

        // If tracking were good, check if we insert a keyframe
        if(bOK)
        {
            mpMapDrawer->SetCurrentCameraPose(mCurrentFrame.mTcw);
            if(NeedNewKeyFrame())
                CreateNewKeyFrame();

            // We allow points with high innovation (considererd outliers by the Huber Function)
            // pass to the new keyframe, so that bundle adjustment will finally decide
            // if they are outliers or not. We don't want next frame to estimate its position
            // with those points so we discard them in the frame.
            for(size_t i=0; i<mCurrentFrame.mvbOutlier.size();i++)
            {
                if(mCurrentFrame.mvpMapPoints[i] && mCurrentFrame.mvbOutlier[i])
                    mCurrentFrame.mvpMapPoints[i]=NULL;
            }
        }
        
        if(bOK){
            //cout << "Frame " << mCurrentFrame.mnId << " works well." << endl;
        }
        else{
            cout << "Frame " << mCurrentFrame.mnId << " is lost." << endl;
        }

        if(bOK)
            mState = WORKING;
        else
            mState=LOST;

        // Reset if the camera get lost soon after initialization
        if(mState==LOST)
        {
            if(mpMap->KeyFramesInMap()<=5)
            {
                cout << "KeyFrame Number is "<<mpMap->KeyFramesInMap()<<". Tracking Lost,  Reset the Tracking"<<endl;
                while(mpLocalMapper->AcceptKeyFrames()){
                    mpLocalMapper->InterruptBA();
                    Reset();
                    break;
                }
                //Reset();
                return;
            }
        }

        // Update motion model
        if(mbMotionModel)
        {
            if(bOK && !mLastFrame.mTcw.empty())
            {
                cv::Mat LastRwc = mLastFrame.mTcw.rowRange(0,3).colRange(0,3).t();
                cv::Mat Lasttwc = -LastRwc*mLastFrame.mTcw.rowRange(0,3).col(3);
                cv::Mat LastTwc = cv::Mat::eye(4,4,CV_32F);
                LastRwc.copyTo(LastTwc.rowRange(0,3).colRange(0,3));
                Lasttwc.copyTo(LastTwc.rowRange(0,3).col(3));
                mVelocity = mCurrentFrame.mTcw*LastTwc;
            }
            else
                mVelocity = cv::Mat();
        }

        mLastFrame = Frame(mCurrentFrame);
     }       

    // Update drawer
    mpFrameDrawer->Update(this);
}
void Tracking::DrawTrj(cv::Mat twc)
{

}
void Tracking::FirstInitialization()
{
    //We ensure a minimum ORB features to continue, otherwise discard frame
    cout << "First Initialization, mCurrentFrame.mnId = " << mCurrentFrame.mnId<< ", with mnFrameId = " << mCurrentFrame.mnId<<endl;
    if(mCurrentFrame.mvKeys.size()>100)
    {
        cout << "Get the First Frame"<<endl;
        cv::Mat framePose(4,4,CV_32F);
        if(_debug_info == 0){
            mpIMU->setFirstTimeStampleandmnId(mCurrentFrame.mTimeStamp , 0 , framePose);
        }
        else{
            mpIMU->setFirstTimeStampleandmnIdDebug(mCurrentFrame.mTimeStamp , 0, framePose);
        }
        mCurrentFrame.mTcw = framePose;

        mInitialFrame = Frame(mCurrentFrame);
        mLastFrame = Frame(mCurrentFrame);
        mvbPrevMatched.resize(mCurrentFrame.mvKeysUn.size());
        for(size_t i=0; i<mCurrentFrame.mvKeysUn.size(); i++)
            mvbPrevMatched[i]=mCurrentFrame.mvKeysUn[i].pt;

        if(mpInitializer)
            delete mpInitializer;

        mpInitializer =  new Initializer(mCurrentFrame,1.0,200);
        //mpIMU->setFirstTimeStample(mCurrentFrame.mTimeStamp);        
        mState = INITIALIZING;
        cout<< "First Initialization Complete"<<endl;
    }
    else{
        cout << "First Initialization NOT Complete"<<endl;
    }
}

void Tracking::Initialize()
{
    // Check if current frame has enough keypoints, otherwise reset initialization process
    cout << "Initialization begin"<<endl;
    if(mCurrentFrame.mvKeys.size()<=100)
    {
        fill(mvIniMatches.begin(),mvIniMatches.end(),-1);
        mState = NOT_INITIALIZED;
        cout << "Initialization NOT Complete, return NOT_INITIALIZED Status"<<endl;
        return;
    }    

    // Find correspondences
    ORBmatcher matcher(0.9,true);
    int nmatches = matcher.SearchForInitialization(mInitialFrame,mCurrentFrame,mvbPrevMatched,mvIniMatches,100);

    // Check if there are enough correspondences
    if(nmatches<100)
    {
        mState = NOT_INITIALIZED;
        cout << "Initialization NOT Complete, nMatches NOT enough, return NOT_INITIALIZED Status"<<endl;
        return;
    }  

    cv::Mat Rcw; // Current Camera Rotation
    cv::Mat tcw; // Current Camera Translation
    vector<bool> vbTriangulated; // Triangulated Correspondences (mvIniMatches)

    if(mpInitializer->Initialize(mCurrentFrame, mvIniMatches, Rcw, tcw, mvIniP3D, vbTriangulated))
    {
        for(size_t i=0, iend=mvIniMatches.size(); i<iend;i++)
        {
            if(mvIniMatches[i]>=0 && !vbTriangulated[i])
            {
                mvIniMatches[i]=-1;
                nmatches--;
            }           
        }
        cout << "Good Initialization, add an IMUFactor"<<endl;
        //cout << "_debug_info = "<<_debug_info<<endl;
        if(_debug_info == 0){
            mpIMU->Addimufactor(mCurrentFrame.mTimeStamp ,int( mCurrentFrame.mnId));  
            //cout << "Enter mpIMU->Addimufactor"  <<endl;         
        }
        else{
            //mpIMU->AddimufactorDebug(mCurrentFrame.mTimeStamp ,int( mCurrentFrame.mnId));
            mpIMU->AddimufactorDebug(mCurrentFrame.mTimeStamp ,int(1));
        }

        cout << "Finish Add IMUFactor"<<endl;
        CreateInitialMap(Rcw,tcw);
        //cout << "Rcw = " << Rcw << endl;
        //cout << "tcw = " << tcw << endl;
        cout << "Initialization Complete, mCurrentFrame.mnId = " << mCurrentFrame.mnId<<", with mnFrameId = " << mCurrentFrame.mnId<<endl;
    }
    else{
        cout << "Initializer Failed"<<endl;
    }
}

void Tracking::CreateInitialMap(cv::Mat &Rcw, cv::Mat &tcw)
{
    cout << "CreateInitialMap:"<<endl;
    // Set Frame Poses
    //mInitialFrame.mTcw = cv::Mat::eye(4,4,CV_32F);
    mCurrentFrame.mTcw = cv::Mat::eye(4,4,CV_32F);
    Rcw.copyTo(mCurrentFrame.mTcw.rowRange(0,3).colRange(0,3));
    tcw.copyTo(mCurrentFrame.mTcw.rowRange(0,3).col(3));
    mCurrentFrame.mTcw = mCurrentFrame.mTcw * mInitialFrame.mTcw;

    // Create KeyFrames
    KeyFrame* pKFini = new KeyFrame(mInitialFrame,mpMap,mpKeyFrameDB);
    KeyFrame* pKFcur = new KeyFrame(mCurrentFrame,mpMap,mpKeyFrameDB);

    pKFini->ComputeBoW();
    pKFcur->ComputeBoW();

    // Insert KFs in the map
    mpMap->AddKeyFrame(pKFini);
    mpMap->AddKeyFrame(pKFcur);
    
    cv::Mat R1(3,3,CV_32F);
    cv::Mat T1(3,1,CV_32F);
    cv::Mat O1(3,1,CV_32F);
    
    R1 = mInitialFrame.mTcw.rowRange(0,3).colRange(0,3);
    T1 = mInitialFrame.mTcw.rowRange(0,3).col(3);
    O1 = -R1.t() * T1;
    
    cv::Mat R2(3,3,CV_32F);
    cv::Mat T2(3,1,CV_32F);
    cv::Mat O2(3,1,CV_32F);
    
    R2 = mCurrentFrame.mTcw.rowRange(0,3).colRange(0,3);
    T2 = mCurrentFrame.mTcw.rowRange(0,3).col(3);
    O2 = -R2.t() * T2;
    
    cv:Mat O12 = O2 - O1;
    double scale_s = cv::norm(O12);
    cout << "scale_s = " << scale_s << std::endl;

    // Create MapPoints and asscoiate to keyframes
    for(size_t i=0; i<mvIniMatches.size();i++)
    {
        if(mvIniMatches[i]<0)
            continue;

        //Create MapPoint.
        cv::Mat worldPos(mvIniP3D[i]);
        worldPos = scale_s * worldPos;
        worldPos = R1.t() * worldPos + O1; // scale and transform the triangelate point to the world frame

        MapPoint* pMP = new MapPoint(worldPos,pKFcur,mpMap);

        pKFini->AddMapPoint(pMP,i);
        pKFcur->AddMapPoint(pMP,mvIniMatches[i]);

        pMP->AddObservation(pKFini,i);
        pMP->AddObservation(pKFcur,mvIniMatches[i]);

        pMP->ComputeDistinctiveDescriptors();
        pMP->UpdateNormalAndDepth();

        //Fill Current Frame structure
        mCurrentFrame.mvpMapPoints[mvIniMatches[i]] = pMP;

        //Add to Map
        mpMap->AddMapPoint(pMP);

    }

    // Update Connections
    pKFini->UpdateConnections();
    pKFcur->UpdateConnections();

    // Bundle Adjustment
    //ROS_INFO("New Map created with %d points",mpMap->MapPointsInMap());

    cout << "Begin to global BundleAdjustment:"<<endl;
    Optimizer::GlobalBundleAdjustemnt(mpMap,20);
    //Optimizer::InitialBAUseIMU(pKFini , pKFcur , mpIMU);
    cout << "Finish global BundleAdjustment"<<endl;
  
    // Set median depth to 1
    float medianDepth = pKFini->ComputeSceneMedianDepth(2);
    float invMedianDepth = 1.0f/medianDepth;
    cout << "invMedianDepth = " << invMedianDepth << std::endl;

    if(medianDepth<0 || pKFcur->TrackedMapPoints()<40)
    {
       // ROS_INFO("Wrong initialization, reseting...");
        cout << "Wrong initialization, reseting ..."<<endl;
        cout<<"medianDepth = "<<medianDepth << ", pKFcur->TrackedMapPoints = "<<pKFcur->TrackedMapPoints()<<endl;
        Reset();
        return;
    }

    // Scale initial baseline
    cv::Mat Tc2w = pKFcur->GetPose();
    Tc2w.col(3).rowRange(0,3) = Tc2w.col(3).rowRange(0,3)*invMedianDepth;
    pKFcur->SetPose(Tc2w);

    // Scale points
    vector<MapPoint*> vpAllMapPoints = pKFini->GetMapPointMatches();
    for(size_t iMP=0; iMP<vpAllMapPoints.size(); iMP++)
    {
        if(vpAllMapPoints[iMP])
        {
            MapPoint* pMP = vpAllMapPoints[iMP];
            pMP->SetWorldPos(pMP->GetWorldPos()*invMedianDepth);
        }
    }
    
    cout << "we have no scale here"<<std::endl;
    mpLocalMapper->InsertKeyFrame(pKFini);
    mpLocalMapper->InsertKeyFrame(pKFcur);

    mCurrentFrame.mTcw = pKFcur->GetPose().clone();
    mLastFrame = Frame(mCurrentFrame);
    mnLastKeyFrameId=mCurrentFrame.mnId;
    mpLastKeyFrame = pKFcur;

    mvpLocalKeyFrames.push_back(pKFcur);
    mvpLocalKeyFrames.push_back(pKFini);
    mvpLocalMapPoints=mpMap->GetAllMapPoints();
    mpReferenceKF = pKFcur;

    mpMap->SetReferenceMapPoints(mvpLocalMapPoints);
    mpMapDrawer->SetCurrentCameraPose(pKFcur->GetPose());

    mState=WORKING;
    cout << "Finish CreateInitialMap"<<endl;
}



bool Tracking::TrackUseImu()
{
    bool ifKF = false;
    cout << "track use imu.............." << endl;
    if(mpLastKeyFrame->updateAfterBA)
    {
        mpIMU->AddKFframeimufactor(mCurrentFrame.mTimeStamp ,int(mCurrentFrame.mnId));
        ifKF = true;  
    }            
    mpIMU->Addframeimufactor(mCurrentFrame.mTimeStamp ,int(mCurrentFrame.mnId));   
    PreintegratedCombinedMeasurements* imu_measurement=mpIMU->Getframeimufactor();
    cv::Mat LastFrame_mTcw = mLastFrame.mTcw;
    gtsam::Pose3 LastFrame_pose = Optimizer::toGTPose3(LastFrame_mTcw);
    Vector3 Lastvel(Vector3(mLastFrame.mSpeedAndBias.vel[0], mLastFrame.mSpeedAndBias.vel[1], mLastFrame.mSpeedAndBias.vel[2]));
	imuBias::ConstantBias LastBias = imuBias::ConstantBias(Vector3(mLastFrame.mSpeedAndBias.accbias[0],mLastFrame.mSpeedAndBias.accbias[1],
		mLastFrame.mSpeedAndBias.accbias[2]), Vector3(mLastFrame.mSpeedAndBias.gyrobias[0],mLastFrame.mSpeedAndBias.gyrobias[1],mLastFrame.mSpeedAndBias.gyrobias[2]));
    NavState LastFrame_nav(LastFrame_pose.inverse() , Lastvel);

    NavState mCurrentFrame_nav = imu_measurement->predict(LastFrame_nav , LastBias);

    gtsam::Pose3 mCurrentFrame_pose = mCurrentFrame_nav.pose().inverse();
    gtsam::Vector val = mCurrentFrame_pose.rotation().quaternion();
	double pX = mCurrentFrame_pose.x();
	double pY = mCurrentFrame_pose.y();
	double pZ = mCurrentFrame_pose.z();
	Eigen::Quaterniond _q1(val(0) , val(1) , val(2) , val(3));
	Eigen::Vector3d _t1(pX,pY,pZ);
	g2o::Sim3 sim3_vec(_q1,_t1,1);
    cv::Mat Tcw1 = Converter::toCvMat(sim3_vec);
    Tcw1.copyTo(mCurrentFrame.mTcw);
    Vector3 Currentvel = mCurrentFrame_nav.v();
	Vector3 accbias = LastBias.accelerometer();
	Vector3 gyrobias = LastBias.gyroscope();
	for(int i=0;i<3;i++)
	{
		mCurrentFrame.mSpeedAndBias.vel[i]= Currentvel[i];
		mCurrentFrame.mSpeedAndBias.gyrobias[i]=gyrobias[i];
		mCurrentFrame.mSpeedAndBias.accbias[i]=accbias[i];
	}
    //cout << "predict end" << endl;

    ORBmatcher matcher(0.9,true);
    vector<MapPoint*> vpMapPointMatches;
    // Search first points at coarse scale levels to get a rough initial estimate
    int minOctave = 0;
    int maxOctave = mCurrentFrame.mvScaleFactors.size()-1;
    if(mpMap->KeyFramesInMap()>5)
        minOctave = maxOctave/2+1;

    //cout << "begin to match" << endl;
    int nmatches = matcher.WindowSearch(mLastFrame,mCurrentFrame,200,vpMapPointMatches,minOctave);//only search mLastFrame's keypoints that's level < minOctave in mCurrentFrame
    //cout << "begin to match2" << endl;
    // If not enough matches, search again without scale constraint
    if(nmatches<10)
    {
        nmatches = matcher.WindowSearch(mLastFrame,mCurrentFrame,100,vpMapPointMatches,0);
        if(nmatches<10)
        {
            vpMapPointMatches=vector<MapPoint*>(mCurrentFrame.mvpMapPoints.size(),static_cast<MapPoint*>(NULL));
            nmatches=0;
            cout<<"Note:Not enough nmatches!!"<<endl;
        }
        cout<<"Note:Not enough nmatches!!"<<endl;
    }
    //cout << "begin to match3" << endl;

    mCurrentFrame.mvpMapPoints=vpMapPointMatches;
    //cout << "begin to match4" << endl;

    // If enough correspondeces, optimize pose and project points from previous frame to search more correspondences
    if(nmatches>=10)
    {
        Optimizer::PoseOptimization(&mCurrentFrame);
        //cout << "first pose optimization use imu" << std::endl;
        //Optimizer::PoseOptimizationUseImu(mpLastKeyFrame,&mLastFrame,&mCurrentFrame,mpMap,mpIMU);

        for(size_t i =0; i<mCurrentFrame.mvbOutlier.size(); i++)
            if(mCurrentFrame.mvbOutlier[i])
            {
                mCurrentFrame.mvpMapPoints[i]=NULL;
                mCurrentFrame.mvbOutlier[i]=false;
                nmatches--;
            }
        // Search by projection with the estimated pose
        nmatches += matcher.SearchByProjection(mLastFrame,mCurrentFrame,15,vpMapPointMatches);
    }
    else //Last opportunity
        nmatches = matcher.SearchByProjection(mLastFrame,mCurrentFrame,50,vpMapPointMatches);

    mCurrentFrame.mvpMapPoints=vpMapPointMatches;

    if(nmatches<8) 
    {
    	cout <<"Note:nmatches < 8" <<endl;
    	nmatches = matcher.SearchByProjection(mLastFrame,mCurrentFrame,100,vpMapPointMatches);
    	if (nmatches < 8)
            return false;
    }
            // Optimize pose again with all correspondences
    cout << "second pose optimization use imu" << std::endl;
    Optimizer::PoseOptimizationUseImu(mpLastKeyFrame,&mLastFrame,&mCurrentFrame,mpMap,mpIMU,ifKF);

    // Discard outliers
    /*for(size_t i =0; i<mCurrentFrame.mvbOutlier.size(); i++)
        if(mCurrentFrame.mvbOutlier[i])
        {
            mCurrentFrame.mvpMapPoints[i]=NULL;
            mCurrentFrame.mvbOutlier[i]=false;
            nmatches--;
        }*/
    cout <<"Finish Track"<<endl;
    return nmatches>=8;            
}

bool Tracking::TrackUseImuPredict()
{
    //cout<<"Tranck Previous Frame use imu:"<<endl;
    ORBmatcher matcher(0.9,true);
    vector<MapPoint*> vpMapPointMatches;
    PreintegratedCombinedMeasurements* imu_measurement;
    fill(mCurrentFrame.mvpMapPoints.begin(),mCurrentFrame.mvpMapPoints.end(),static_cast<MapPoint*>(NULL));
    /////// Last KF predict LastFrame////////////
    /*if(mpLastKeyFrame->updateAfterBA)
    {
        cout << "mpLastKeyFrame updated ////////////////////////////////" << endl;
        imu_measurement=mpIMU->GetInstantKFAndFrameimufactor(mpLastKeyFrame->mTimeStamp, mLastFrame.mTimeStamp); 
        mpLastKeyFrame->updateAfterBA = false;
        cv::Mat LastKF_mTcw = mpLastKeyFrame->GetPose();
        gtsam::Pose3 LastKF_pose = Optimizer::toGTPose3(LastKF_mTcw);
        Vector3 LastKFvel(Vector3(mpLastKeyFrame->mSpeedAndBias.vel[0], mpLastKeyFrame->mSpeedAndBias.vel[1], mpLastKeyFrame->mSpeedAndBias.vel[2]));
        imuBias::ConstantBias LastKFBias = imuBias::ConstantBias(Vector3(mpLastKeyFrame->mSpeedAndBias.accbias[0],mpLastKeyFrame->mSpeedAndBias.accbias[1],
			 	mpLastKeyFrame->mSpeedAndBias.accbias[2]), Vector3(mpLastKeyFrame->mSpeedAndBias.gyrobias[0],mpLastKeyFrame->mSpeedAndBias.gyrobias[1],mpLastKeyFrame->mSpeedAndBias.gyrobias[2]));
        NavState LastKF_nav(LastKF_pose.inverse() , LastKFvel);
        NavState LastFrame_nav = imu_measurement->predict(LastKF_nav , LastKFBias);
        gtsam::Pose3 LastFrame_pose = LastFrame_nav.pose().inverse();
        cv::Mat Tcw1 = Optimizer::Pose3TocvMat(LastFrame_pose);
        Tcw1.copyTo(mLastFrame.mTcw);
        Vector3 Lastvel = LastFrame_nav.v();
	    Vector3 accbias = LastKFBias.accelerometer();
	    Vector3 gyrobias = LastKFBias.gyroscope();
        for(int i=0;i<3;i++)
	    {
		    mLastFrame.mSpeedAndBias.vel[i]= Lastvel[i];
		    mLastFrame.mSpeedAndBias.gyrobias[i]=gyrobias[i];
		    mLastFrame.mSpeedAndBias.accbias[i]=accbias[i];
	    }
        Optimizer::PoseOptimization(&mLastFrame);
        for(size_t i =0; i<mLastFrame.mvpMapPoints.size(); i++)
        {
        if(mLastFrame.mvpMapPoints[i])
        {
            if(mLastFrame.mvbOutlier[i])
            {
                mLastFrame.mvpMapPoints[i]=NULL;
                mLastFrame.mvbOutlier[i]=false;
            }
        }
        }
    }*/
    cout << "mpLastKeyFrame not updated ////////////////////////////////" << endl;
    ////////////Last Frame to predict current frame///////////////
    imu_measurement=mpIMU->GetInstantimufactorDebug2(mLastFrame.mTimeStamp, mCurrentFrame.mTimeStamp);   
    cv::Mat LastFrame_mTcw = mLastFrame.mTcw;
    gtsam::Pose3 LastFrame_pose = Optimizer::toGTPose3(LastFrame_mTcw);
    Vector3 Lastvel(Vector3(mLastFrame.mSpeedAndBias.vel[0], mLastFrame.mSpeedAndBias.vel[1], mLastFrame.mSpeedAndBias.vel[2]));
	imuBias::ConstantBias LastBias = imuBias::ConstantBias(Vector3(mLastFrame.mSpeedAndBias.accbias[0],mLastFrame.mSpeedAndBias.accbias[1],
		mLastFrame.mSpeedAndBias.accbias[2]), Vector3(mLastFrame.mSpeedAndBias.gyrobias[0],mLastFrame.mSpeedAndBias.gyrobias[1],mLastFrame.mSpeedAndBias.gyrobias[2]));
    NavState LastFrame_nav(LastFrame_pose.inverse() , Lastvel);
    NavState mCurrentFrame_nav = imu_measurement->predict(LastFrame_nav , LastBias);
    gtsam::Pose3 mCurrentFrame_pose = mCurrentFrame_nav.pose().inverse();
    cv::Mat Tcw1 = Optimizer::Pose3TocvMat(mCurrentFrame_pose);
    Tcw1.copyTo(mCurrentFrame.mTcw);
    Vector3 Currentvel = mCurrentFrame_nav.v();
	Vector3 accbias = LastBias.accelerometer();
	Vector3 gyrobias = LastBias.gyroscope();
	for(int i=0;i<3;i++)
	{
		mCurrentFrame.mSpeedAndBias.vel[i]= Currentvel[i];
		mCurrentFrame.mSpeedAndBias.gyrobias[i]=gyrobias[i];
		mCurrentFrame.mSpeedAndBias.accbias[i]=accbias[i];
	}

    int nmatches = matcher.SearchByProjection(mCurrentFrame, mLastFrame,50);

    if (nmatches < 20)
        return false;
    
    // Optimize pose again with all correspondences
    Optimizer::PoseOptimization(&mCurrentFrame);

    // Discard outliers
    for(size_t i =0; i<mCurrentFrame.mvpMapPoints.size(); i++)
    {
        if(mCurrentFrame.mvpMapPoints[i])
        {
            if(mCurrentFrame.mvbOutlier[i])
            {
                mCurrentFrame.mvpMapPoints[i]=NULL;
                mCurrentFrame.mvbOutlier[i]=false;
                nmatches--;
            }
        }
    }
    PreintegratedCombinedMeasurements::Params params = imu_measurement->p();
    Vector3 gravity = params.n_gravity;
    cv::Mat g(3,1,CV_32F);
    g.at<float>(0) = gravity(0);g.at<float>(1) = gravity(1);g.at<float>(2) = gravity(2);

    cv::Mat tk1 = mLastFrame.mTcw.rowRange(0,3).col(3);
    cv::Mat tk2 = mCurrentFrame.mTcw.rowRange(0,3).col(3);
    cv::Mat Rcw1 = mLastFrame.mTcw.rowRange(0,3).colRange(0,3);
    cv::Mat Rcw2 = mCurrentFrame.mTcw.rowRange(0,3).colRange(0,3);
    cv::Mat Rwc1 = Rcw1.t();
    cv::Mat Rwc2 = Rcw2.t();
    cv::Mat Pk1 = - Rwc1 *tk1;
    cv::Mat Pk2 = - Rwc2 *tk2;

    double dt = imu_measurement->deltaTij();
    Vector3 dvv = imu_measurement->deltaVij();
    cv::Mat dv(3,1,CV_32F);
    dv.at<float>(0) = dvv(0); dv.at<float>(1) = dvv(1); dv.at<float>(2) = dvv(2);
    Vector3 dpv = imu_measurement->deltaPij();
    cv::Mat dp(3,1,CV_32F);
	dp.at<float>(0) = dpv(0); dp.at<float>(1) = dpv(1); dp.at<float>(2) = dpv(2);
    cv::Mat gt2 = 1/2 * g * dt *dt;
    cv::Mat vk1 = (Pk2 - Pk1 - gt2 - Rwc1*dp) /dt;
    cv::Mat vk2 = vk1 + g * dt + Rwc1 * dv;
    for(int i=0;i<3;i++)
	{
        mLastFrame.mSpeedAndBias.vel[i] = vk1.at<float>(i);
		mCurrentFrame.mSpeedAndBias.vel[i]= vk2.at<float>(i);
        mCurrentFrame.mSpeedAndBias.accbias[i] = mLastFrame.mSpeedAndBias.accbias[i];
        mCurrentFrame.mSpeedAndBias.gyrobias[i] = mLastFrame.mSpeedAndBias.gyrobias[i];
	}

    ///////////one more time///////////////
    Vector3 ini_velocity1(mLastFrame.mSpeedAndBias.vel[0],
                            mLastFrame.mSpeedAndBias.vel[1],
                            mLastFrame.mSpeedAndBias.vel[2]);
    NavState mInitialFrame_nav1(LastFrame_pose.inverse() , ini_velocity1);
    NavState mCurrentFrame_nav1 = imu_measurement->predict(mInitialFrame_nav1 , LastBias);
    gtsam::Pose3 mCurrentFrame_pose1 = mCurrentFrame_nav1.pose().inverse();
    Tcw1 = Optimizer::Pose3TocvMat(mCurrentFrame_pose1);
    Tcw1.copyTo(mCurrentFrame.mTcw);

    fill(mCurrentFrame.mvpMapPoints.begin(),mCurrentFrame.mvpMapPoints.end(),static_cast<MapPoint*>(NULL));

    nmatches = matcher.SearchByProjection(mCurrentFrame, mLastFrame,50);
    // Optimize pose again with all correspondences
    Optimizer::PoseOptimization(&mCurrentFrame);

    // Discard outliers
    for(size_t i =0; i<mCurrentFrame.mvbOutlier.size(); i++)
        if(mCurrentFrame.mvbOutlier[i])
        {
            mCurrentFrame.mvpMapPoints[i]=NULL;
            mCurrentFrame.mvbOutlier[i]=false;
            nmatches--;
        }
    
    tk2 = mCurrentFrame.mTcw.rowRange(0,3).col(3);

    Rcw2 = mCurrentFrame.mTcw.rowRange(0,3).colRange(0,3);
    
    Rwc2 = Rcw2.t();

    Pk2 = - Rwc2 *tk2;

    vk1 = (Pk2 - Pk1 - gt2 - Rwc1*dp) /dt;
    vk2 = vk1 + g * dt + Rwc1 * dv;
    for(int i=0;i<3;i++)
	{
		mCurrentFrame.mSpeedAndBias.vel[i]= vk2.at<float>(i);
        mCurrentFrame.mSpeedAndBias.accbias[i] = mLastFrame.mSpeedAndBias.accbias[i];
        mCurrentFrame.mSpeedAndBias.gyrobias[i] = mLastFrame.mSpeedAndBias.gyrobias[i];
	}
    cout << " nmatches remains " << nmatches << endl;
    if(nmatches <= 50)
    {
        cout << "matches < 50 ..................../////////////////////////////" << endl;
        fill(mCurrentFrame.mvpMapPoints.begin(),mCurrentFrame.mvpMapPoints.end(),static_cast<MapPoint*>(NULL));
        nmatches = matcher.SearchByProjection(mCurrentFrame, mpLastKeyFrame,50);
        cout << "match number :" << nmatches << "..................." << endl;
        Optimizer::PoseOptimization(&mCurrentFrame);
        for(size_t i =0; i<mCurrentFrame.mvbOutlier.size(); i++)
        if(mCurrentFrame.mvbOutlier[i])
        {
            mCurrentFrame.mvpMapPoints[i]=NULL;
            mCurrentFrame.mvbOutlier[i]=false;
            nmatches--;
        }
    
    tk2 = mCurrentFrame.mTcw.rowRange(0,3).col(3);

    Rcw2 = mCurrentFrame.mTcw.rowRange(0,3).colRange(0,3);
    
    Rwc2 = Rcw2.t();

    Pk2 = - Rwc2 *tk2;

    vk1 = (Pk2 - Pk1 - gt2 - Rwc1*dp) /dt;
    vk2 = vk1 + g * dt + Rwc1 * dv;
    for(int i=0;i<3;i++)
	{
		mCurrentFrame.mSpeedAndBias.vel[i]= vk2.at<float>(i);
        mCurrentFrame.mSpeedAndBias.accbias[i] = mLastFrame.mSpeedAndBias.accbias[i];
        mCurrentFrame.mSpeedAndBias.gyrobias[i] = mLastFrame.mSpeedAndBias.gyrobias[i];
	}
    cout << " nmatches remains " << nmatches << endl;
    }

    
    //cout <<"Finish Track"<<endl;
    return nmatches>=10;
}

bool Tracking::TrackPreviousFrame()
{
    cout<<"Tranck Previous Frame:"<<endl;
    ORBmatcher matcher(0.9,true);
    vector<MapPoint*> vpMapPointMatches;
    // Search first points at coarse scale levels to get a rough initial estimate
    int minOctave = 0;
    int maxOctave = mCurrentFrame.mvScaleFactors.size()-1;
    if(mpMap->KeyFramesInMap()>5)
        minOctave = maxOctave/2+1;

    int nmatches = matcher.WindowSearch(mLastFrame,mCurrentFrame,200,vpMapPointMatches,minOctave);
    cout << "mmatches = " << nmatches << std::endl;

    // If not enough matches, search again without scale constraint
    if(nmatches<10)
    {
        nmatches = matcher.WindowSearch(mLastFrame,mCurrentFrame,100,vpMapPointMatches,0);
        if(nmatches<10)
        {
            vpMapPointMatches=vector<MapPoint*>(mCurrentFrame.mvpMapPoints.size(),static_cast<MapPoint*>(NULL));
            nmatches=0;
            cout<<"Note:Not enough nmatches!!"<<endl;
        }
        cout<<"Note:Not enough nmatches!!"<<endl;
    }

    mLastFrame.mTcw.copyTo(mCurrentFrame.mTcw);
    mCurrentFrame.mvpMapPoints=vpMapPointMatches;

    // If enough correspondeces, optimize pose and project points from previous frame to search more correspondences
    if(nmatches>=10)
    {
        // Optimize pose with correspondences
        Optimizer::PoseOptimization(&mCurrentFrame);

        for(size_t i =0; i<mCurrentFrame.mvbOutlier.size(); i++)
            if(mCurrentFrame.mvbOutlier[i])
            {
                mCurrentFrame.mvpMapPoints[i]=NULL;
                mCurrentFrame.mvbOutlier[i]=false;
                nmatches--;
            }

        // Search by projection with the estimated pose
        nmatches += matcher.SearchByProjection(mLastFrame,mCurrentFrame,15,vpMapPointMatches);
    }
    else //Last opportunity
        nmatches = matcher.SearchByProjection(mLastFrame,mCurrentFrame,50,vpMapPointMatches);


    mCurrentFrame.mvpMapPoints=vpMapPointMatches;

	//cout <<"nmatches = " << nmatches << endl;
    if(nmatches<8) 
    {
    	cout <<"Note:nmatches < 8" <<endl;
    	nmatches = matcher.SearchByProjection(mLastFrame,mCurrentFrame,100,vpMapPointMatches);
    	if (nmatches < 8)
        return false;
    }
    // Optimize pose again with all correspondences
    Optimizer::PoseOptimization(&mCurrentFrame);

    // Discard outliers
    for(size_t i =0; i<mCurrentFrame.mvbOutlier.size(); i++)
        if(mCurrentFrame.mvbOutlier[i])
        {
            mCurrentFrame.mvpMapPoints[i]=NULL;
            mCurrentFrame.mvbOutlier[i]=false;
            nmatches--;
        }
    cout << " nmatches remains " << nmatches << endl;
    //cout <<"Finish Track"<<endl;
    return nmatches>=8;
}

bool Tracking::TrackWithMotionModel()
{
    //cout <<"Track With Motion Model:"<<endl;
    ORBmatcher matcher(0.9,true);
    vector<MapPoint*> vpMapPointMatches;

    // Compute current pose by motion model
    mCurrentFrame.mTcw = mVelocity*mLastFrame.mTcw;

    fill(mCurrentFrame.mvpMapPoints.begin(),mCurrentFrame.mvpMapPoints.end(),static_cast<MapPoint*>(NULL));

    // Project points seen in previous frame
    int nmatches = matcher.SearchByProjection(mCurrentFrame,mLastFrame,15);

    if(nmatches<20){
        //cout << "nmatches = " << nmatches<<endl;
        //cout << "in Frame " << mCurrentFrame.mnId << " we got lost without enough matches"<<endl;
        return false;
    }
       

    // Optimize pose with all correspondences
    Optimizer::PoseOptimization(&mCurrentFrame);

    // Discard outliers
    for(size_t i =0; i<mCurrentFrame.mvpMapPoints.size(); i++)
    {
        if(mCurrentFrame.mvpMapPoints[i])
        {
            if(mCurrentFrame.mvbOutlier[i])
            {
                mCurrentFrame.mvpMapPoints[i]=NULL;
                mCurrentFrame.mvbOutlier[i]=false;
                nmatches--;
            }
        }
    }

    //cout <<"Finish Track"<<endl;
    return nmatches>=10;
}

bool Tracking::TrackLocalMap()
{
    // Tracking from previous frame or relocalisation was succesfull and we have an estimation
    // of the camera pose and some map points tracked in the frame.
    // Update Local Map and Track

    // Update Local Map
    //cout << "Track Local Map:"<<endl;
    UpdateReference();

    // Search Local MapPoints
    SearchReferencePointsInFrustum();

    // Optimize Pose
    mnMatchesInliers = Optimizer::PoseOptimization(&mCurrentFrame);

    // Update MapPoints Statistics
    for(size_t i=0; i<mCurrentFrame.mvpMapPoints.size(); i++)
        if(mCurrentFrame.mvpMapPoints[i])
        {
            if(!mCurrentFrame.mvbOutlier[i])
                mCurrentFrame.mvpMapPoints[i]->IncreaseFound();
        }

    // Decide if the tracking was succesful
    // More restrictive if there was a relocalization recently
    //cout << "Finish Track"<<endl;
    //cout << "Track local Map with mnMatchesInliers = " << mnMatchesInliers << endl << endl; 
    if(mCurrentFrame.mnId<mnLastRelocFrameId+mMaxFrames && mnMatchesInliers<30)
        return false;

    if(mnMatchesInliers<30)
        return false;
    else
        return true;
}


bool Tracking::NeedNewKeyFrame()
{
    // If Local Mapping is freezed by a Loop Closure do not insert keyframes
    if(mpLocalMapper->isStopped() || mpLocalMapper->stopRequested())
        return false;

    // Not insert keyframes if not enough frames from last relocalisation have passed
    if(mCurrentFrame.mnId<mnLastRelocFrameId+mMaxFrames && mpMap->KeyFramesInMap()>mMaxFrames)
        return false;

    // Reference KeyFrame MapPoints
    int nRefMatches = mpReferenceKF->TrackedMapPoints();

    // Local Mapping accept keyframes?
    bool bLocalMappingIdle = mpLocalMapper->AcceptKeyFrames();

    // Condition 1a: More than "MaxFrames" have passed from last keyframe insertion
    const bool c1a = mCurrentFrame.mnId>=mnLastKeyFrameId+mMaxFrames;
    // Condition 1b: More than "MinFrames" have passed and Local Mapping is idle
    const bool c1b = (mCurrentFrame.mnId>=mnLastKeyFrameId+mMinFrames) && bLocalMappingIdle;
    // Condition 2: Less than 90% of points than reference keyframe and enough inliers
    const bool c2 = mnMatchesInliers<nRefMatches*0.9 && mnMatchesInliers>15;
    
    cout << "In Frame " << mCurrentFrame.mnId << ", c1a = " << c1a << ", c1b = " << c1b << ", c2 = " << c2 << ", bLocalMappingIdle = " << bLocalMappingIdle << std::endl;
    //cout << "mCurrentFrame.mnId = " << mCurrentFrame.mnId << ", mnLastKeyFrameId = " << mnLastKeyFrameId << ", mMinFrames = " << mMinFrames << std::endl;
    if((c1a||c1b)&&c2)
    {
        // If the mapping accepts keyframes insert, otherwise send a signal to interrupt BA, but not insert yet
        if(bLocalMappingIdle)
        {
            return true;
        }
        else
        {
            mpLocalMapper->InterruptBA();
            return false;
        }
    }
    else
        return false;
}

void Tracking::CreateNewKeyFrame()
{

    KeyFrame* pKF = new KeyFrame(mCurrentFrame,mpMap,mpKeyFrameDB);
    keyframeNum ++;
    cout << "Insert a KeyFrame with FrameId : "<< pKF->mnFrameId<<endl;
    mpLocalMapper->InsertKeyFrame(pKF);
    //cout << "_debug_info = "<<_debug_info<<endl;
    if(_debug_info == 0){
        mpIMU->Addimufactor(pKF->mTimeStamp , pKF->mnFrameId);
        //cout << "Enter mpIMU->Addimufactor"<<endl;
    }
    else{
        //mpIMU->AddimufactorDebug(pKF->mTimeStamp , pKF->mnFrameId);
        mpIMU->AddimufactorDebug(pKF->mTimeStamp , pKF->mnId);

        //cout << "Enter mpIMU->AddimufactorDebug"<<endl;
    }
    
    mnLastKeyFrameId = mCurrentFrame.mnId;
    mpLastKeyFrame = pKF;
}

void Tracking::SearchReferencePointsInFrustum()
{
    // Do not search map points already matched
    for(vector<MapPoint*>::iterator vit=mCurrentFrame.mvpMapPoints.begin(), vend=mCurrentFrame.mvpMapPoints.end(); vit!=vend; vit++)
    {
        MapPoint* pMP = *vit;
        if(pMP)
        {
            if(pMP->isBad())
            {
                *vit = NULL;
            }
            else
            {
                pMP->IncreaseVisible();
                pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                pMP->mbTrackInView = false;
            }
        }
    }

    mCurrentFrame.UpdatePoseMatrices();

    int nToMatch=0;

    // Project points in frame and check its visibility
    for(vector<MapPoint*>::iterator vit=mvpLocalMapPoints.begin(), vend=mvpLocalMapPoints.end(); vit!=vend; vit++)
    {
        MapPoint* pMP = *vit;
        if(pMP->mnLastFrameSeen == mCurrentFrame.mnId)
            continue;
        if(pMP->isBad())
            continue;        
        // Project (this fills MapPoint variables for matching)
        if(mCurrentFrame.isInFrustum(pMP,0.5))
        {
            pMP->IncreaseVisible();
            nToMatch++;
        }
    }    


    if(nToMatch>0)
    {
        ORBmatcher matcher(0.8);
        int th = 1;
        // If the camera has been relocalised recently, perform a coarser search
        if(mCurrentFrame.mnId<mnLastRelocFrameId+2)
            th=5;
        matcher.SearchByProjection(mCurrentFrame,mvpLocalMapPoints,th);
    }
}

void Tracking::UpdateReference()
{    
    // This is for visualization
    mpMap->SetReferenceMapPoints(mvpLocalMapPoints);

    // Update
    UpdateReferenceKeyFrames();
    UpdateReferencePoints();
}

void Tracking::UpdateReferencePoints()
{
    mvpLocalMapPoints.clear();

    for(vector<KeyFrame*>::iterator itKF=mvpLocalKeyFrames.begin(), itEndKF=mvpLocalKeyFrames.end(); itKF!=itEndKF; itKF++)
    {
        KeyFrame* pKF = *itKF;
        vector<MapPoint*> vpMPs = pKF->GetMapPointMatches();

        for(vector<MapPoint*>::iterator itMP=vpMPs.begin(), itEndMP=vpMPs.end(); itMP!=itEndMP; itMP++)
        {
            MapPoint* pMP = *itMP;
            if(!pMP)
                continue;
            if(pMP->mnTrackReferenceForFrame==mCurrentFrame.mnId)
                continue;
            if(!pMP->isBad())
            {
                mvpLocalMapPoints.push_back(pMP);
                pMP->mnTrackReferenceForFrame=mCurrentFrame.mnId;
            }
        }
    }
}


void Tracking::UpdateReferenceKeyFrames()
{
    // Each map point vote for the keyframes in which it has been observed
    map<KeyFrame*,int> keyframeCounter;
    for(size_t i=0, iend=mCurrentFrame.mvpMapPoints.size(); i<iend;i++)
    {
        if(mCurrentFrame.mvpMapPoints[i])
        {
            MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
            if(!pMP->isBad())
            {
                map<KeyFrame*,size_t> observations = pMP->GetObservations();
                for(map<KeyFrame*,size_t>::iterator it=observations.begin(), itend=observations.end(); it!=itend; it++)
                    keyframeCounter[it->first]++;
            }
            else
            {
                mCurrentFrame.mvpMapPoints[i]=NULL;
            }
        }
    }

    int max=0;
    KeyFrame* pKFmax=NULL;

    mvpLocalKeyFrames.clear();
    mvpLocalKeyFrames.reserve(3*keyframeCounter.size());

    // All keyframes that observe a map point are included in the local map. Also check which keyframe shares most points
    for(map<KeyFrame*,int>::iterator it=keyframeCounter.begin(), itEnd=keyframeCounter.end(); it!=itEnd; it++)
    {
        KeyFrame* pKF = it->first;

        if(pKF->isBad())
            continue;

        if(it->second>max)
        {
            max=it->second;
            pKFmax=pKF;
        }

        mvpLocalKeyFrames.push_back(it->first);
        pKF->mnTrackReferenceForFrame = mCurrentFrame.mnId;
    }


    // Include also some not-already-included keyframes that are neighbors to already-included keyframes
    for(vector<KeyFrame*>::iterator itKF=mvpLocalKeyFrames.begin(), itEndKF=mvpLocalKeyFrames.end(); itKF!=itEndKF; itKF++)
    {
        // Limit the number of keyframes
        if(mvpLocalKeyFrames.size()>80)
            break;

        KeyFrame* pKF = *itKF;

        vector<KeyFrame*> vNeighs = pKF->GetBestCovisibilityKeyFrames(10);

        for(vector<KeyFrame*>::iterator itNeighKF=vNeighs.begin(), itEndNeighKF=vNeighs.end(); itNeighKF!=itEndNeighKF; itNeighKF++)
        {
            KeyFrame* pNeighKF = *itNeighKF;
            if(!pNeighKF->isBad())
            {
                if(pNeighKF->mnTrackReferenceForFrame!=mCurrentFrame.mnId)
                {
                    mvpLocalKeyFrames.push_back(pNeighKF);
                    pNeighKF->mnTrackReferenceForFrame=mCurrentFrame.mnId;
                    break;
                }
            }
        }

    }

    mpReferenceKF = pKFmax;
}

bool Tracking::Relocalisation()
{
    // Compute Bag of Words Vector
    cout << "Relocalisation:"<<endl;
    mCurrentFrame.ComputeBoW();

    // Relocalisation is performed when tracking is lost and forced at some stages during loop closing
    // Track Lost: Query KeyFrame Database for keyframe candidates for relocalisation
    vector<KeyFrame*> vpCandidateKFs;
    if(!RelocalisationRequested())
        vpCandidateKFs= mpKeyFrameDB->DetectRelocalisationCandidates(&mCurrentFrame);
    else // Forced Relocalisation: Relocate against local window around last keyframe
    {
        boost::mutex::scoped_lock lock(mMutexForceRelocalisation);
        mbForceRelocalisation = false;
        vpCandidateKFs.reserve(10);
        vpCandidateKFs = mpLastKeyFrame->GetBestCovisibilityKeyFrames(9);
        vpCandidateKFs.push_back(mpLastKeyFrame);
    }

    if(vpCandidateKFs.empty())
        return false;

    const int nKFs = vpCandidateKFs.size();

    // We perform first an ORB matching with each candidate
    // If enough matches are found we setup a PnP solver
    ORBmatcher matcher(0.75,true);

    vector<PnPsolver*> vpPnPsolvers;
    vpPnPsolvers.resize(nKFs);

    vector<vector<MapPoint*> > vvpMapPointMatches;
    vvpMapPointMatches.resize(nKFs);

    vector<bool> vbDiscarded;
    vbDiscarded.resize(nKFs);

    int nCandidates=0;

    for(size_t i=0; i<vpCandidateKFs.size(); i++)
    {
        KeyFrame* pKF = vpCandidateKFs[i];
        if(pKF->isBad())
            vbDiscarded[i] = true;
        else
        {
            int nmatches = matcher.SearchByBoW(pKF,mCurrentFrame,vvpMapPointMatches[i]);
            if(nmatches<15)
            {
                vbDiscarded[i] = true;
                continue;
            }
            else
            {
                PnPsolver* pSolver = new PnPsolver(mCurrentFrame,vvpMapPointMatches[i]);
                pSolver->SetRansacParameters(0.99,10,300,4,0.5,5.991);
                vpPnPsolvers[i] = pSolver;
                nCandidates++;
            }
        }        
    }

    // Alternatively perform some iterations of P4P RANSAC
    // Until we found a camera pose supported by enough inliers
    bool bMatch = false;
    ORBmatcher matcher2(0.9,true);

    while(nCandidates>0 && !bMatch)
    {
        for(size_t i=0; i<vpCandidateKFs.size(); i++)
        {
            if(vbDiscarded[i])
                continue;

            // Perform 5 Ransac Iterations
            vector<bool> vbInliers;
            int nInliers;
            bool bNoMore;

            PnPsolver* pSolver = vpPnPsolvers[i];
            cv::Mat Tcw = pSolver->iterate(5,bNoMore,vbInliers,nInliers);

            // If Ransac reachs max. iterations discard keyframe
            if(bNoMore)
            {
                vbDiscarded[i]=true;
                nCandidates--;
            }

            // If a Camera Pose is computed, optimize
            if(!Tcw.empty())
            {
                Tcw.copyTo(mCurrentFrame.mTcw);

                set<MapPoint*> sFound;

                for(size_t j=0; j<vbInliers.size(); j++)
                {
                    if(vbInliers[j])
                    {
                        mCurrentFrame.mvpMapPoints[j]=vvpMapPointMatches[i][j];
                        sFound.insert(vvpMapPointMatches[i][j]);
                    }
                    else
                        mCurrentFrame.mvpMapPoints[j]=NULL;
                }

                int nGood = Optimizer::PoseOptimization(&mCurrentFrame);

                if(nGood<10)
                    continue;

                for(size_t io =0, ioend=mCurrentFrame.mvbOutlier.size(); io<ioend; io++)
                    if(mCurrentFrame.mvbOutlier[io])
                        mCurrentFrame.mvpMapPoints[io]=NULL;

                // If few inliers, search by projection in a coarse window and optimize again
                if(nGood<50)
                {
                    int nadditional =matcher2.SearchByProjection(mCurrentFrame,vpCandidateKFs[i],sFound,10,100);

                    if(nadditional+nGood>=50)
                    {
                        nGood = Optimizer::PoseOptimization(&mCurrentFrame);

                        // If many inliers but still not enough, search by projection again in a narrower window
                        // the camera has been already optimized with many points
                        if(nGood>30 && nGood<50)
                        {
                            sFound.clear();
                            for(size_t ip =0, ipend=mCurrentFrame.mvpMapPoints.size(); ip<ipend; ip++)
                                if(mCurrentFrame.mvpMapPoints[ip])
                                    sFound.insert(mCurrentFrame.mvpMapPoints[ip]);
                            nadditional =matcher2.SearchByProjection(mCurrentFrame,vpCandidateKFs[i],sFound,3,64);

                            // Final optimization
                            if(nGood+nadditional>=50)
                            {
                                nGood = Optimizer::PoseOptimization(&mCurrentFrame);

                                for(size_t io =0; io<mCurrentFrame.mvbOutlier.size(); io++)
                                    if(mCurrentFrame.mvbOutlier[io])
                                        mCurrentFrame.mvpMapPoints[io]=NULL;
                            }
                        }
                    }
                }


                // If the pose is supported by enough inliers stop ransacs and continue
                if(nGood>=50)
                {                    
                    bMatch = true;
                    break;
                }
            }
        }
    }

    cout << "Finish Relocalisation"<<endl;
    if(!bMatch)
    {
        return false;
    }
    else
    {
        mnLastRelocFrameId = mCurrentFrame.mnId;
        return true;
    }

}

void Tracking::ForceRelocalisation()
{
    boost::mutex::scoped_lock lock(mMutexForceRelocalisation);
    mbForceRelocalisation = true;
    mnLastRelocFrameId = mCurrentFrame.mnId;
}

bool Tracking::RelocalisationRequested()
{
    boost::mutex::scoped_lock lock(mMutexForceRelocalisation);
    return mbForceRelocalisation;
}


void Tracking::Reset()
{
    {
        boost::mutex::scoped_lock lock(mMutexReset);
        mbPublisherStopped = false;
        mbReseting = true;
    }

    // Wait until publishers are stopped
    //ros::Rate r(500);
    while(1)
    {
        {
            boost::mutex::scoped_lock lock(mMutexReset);
            if(mbPublisherStopped)
                break;
        }
       // r.sleep();
       cout << "waiting reset ..."<<endl;
        usleep(2000);
    }

    // Reset Local Mapping
    cout << "mpLocalMapper->RequestReset()" << std::endl;
    mpLocalMapper->RequestReset();
    // Reset Loop Closing
    cout << "mpLoopClosing->RequestReset()" << std::endl;
    mpLoopClosing->RequestReset();
    // Clear BoW Database
    cout << "mpKeyFrameDB->clear()" << std::endl;
    mpKeyFrameDB->clear();
    // Clear Map (this erase MapPoints and KeyFrames)
    cout << "mpMap->clear()" << std::endl;
    mpMap->clear();
    cout << "reseting other..." << std::endl;

    KeyFrame::nNextId = 0;
    Frame::nNextId = 0;
    mState = NOT_INITIALIZED;

    {
        boost::mutex::scoped_lock lock(mMutexReset);
        mbReseting = false;
    }
    cout << "finish reseting"<< std::endl;
}

void Tracking::CheckResetByPublishers()
{
    bool bReseting = false;

    {
        boost::mutex::scoped_lock lock(mMutexReset);
        bReseting = mbReseting;
    }

    if(bReseting)
    {
        boost::mutex::scoped_lock lock(mMutexReset);
        mbPublisherStopped = true;
    }

    // Hold until reset is finished
  //  ros::Rate r(500);
    while(1)
    {
        {
            boost::mutex::scoped_lock lock(mMutexReset);
            if(!mbReseting)
            {
                mbPublisherStopped=false;
                break;
            }
        }
        //r.sleep();
        usleep(2000);
    }
}


void Tracking::loadTimestample(char* time_file){
    _timestample.clear();
    double _t;
    int _num;
    long double vv;
    FILE *fp = fopen(time_file , "r");
    fscanf(fp , "%d" , &_num);
    for(int i=0;i<_num;i++){
        fscanf(fp, "%Lf" , &vv);
        _t = vv / (1000000000);
        _timestample.push_back(_t);
    }
    fclose(fp);
    _debug_info = 1;
}

} //namespace ORB_SLAM
