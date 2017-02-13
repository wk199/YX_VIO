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

#include<iostream>
#include<fstream>
//#include<ros/ros.h>
//#include<ros/package.h>
#include<boost/thread.hpp>

#include<opencv2/core/core.hpp>

#include "Tracking.h"
//#include "FramePublisher.h"
#include "FrameDrawer.h"
#include "MapDrawer.h"
#include "Map.h"
//#include "MapPublisher.h"
#include "LocalMapping.h"
#include "LoopClosing.h"
#include "KeyFrameDatabase.h"
#include "ORBVocabulary.h"
#include "Viewer.h"

#include "Converter.h"
#include "IMUMeasurement.h"

using namespace std;


int main(int argc, char **argv)
{
  //  ros::init(argc, argv, "ORB_SLAM");
  //  ros::start();

    cout << endl << "ORB-SLAM Copyright (C) 2014 Raul Mur-Artal" << endl <<
            "This program comes with ABSOLUTELY NO WARRANTY;" << endl  <<
            "This is free software, and you are welcome to redistribute it" << endl <<
            "under certain conditions. See LICENSE.txt." << endl;
/*
    if(argc != 3)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM ORB_SLAM path_to_vocabulary path_to_settings (absolute or relative to package directory)" << endl;
  //      ros::shutdown();
        return 1;
    }
*/
    // Load Settings and Check
    //string strSettingsFile = ros::package::getPath("ORB_SLAM")+"/"+argv[2];
    string strSettingsFile = "../Data/Settings_2.yaml";

    cv::FileStorage fsSettings(strSettingsFile.c_str(), cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
     //   ROS_ERROR("Wrong path to settings. Path must be absolut or relative to ORB_SLAM package directory.");
      //  ros::shutdown();
    	cout <<"error"<<endl;
        return 1;
    }

    //Create Frame Publisher for image_view
    //ORB_SLAM::FramePublisher FramePub;

    //Load ORB Vocabulary
  //  string strVocFile = ros::package::getPath("ORB_SLAM")+"/"+argv[1];
    string strVocFile = "../Data/ORBvoc.yml";
    cout << endl << "Loading ORB Vocabulary. This could take a while." << endl;
    cv::FileStorage fsVoc(strVocFile.c_str(), cv::FileStorage::READ);
    if(!fsVoc.isOpened())
    {
    //    cerr << endl << "Wrong path to vocabulary. Path must be absolut or relative to ORB_SLAM package directory." << endl;
     //   ros::shutdown();
        cout <<"error"<<endl;
        return 1;
    }
    ORB_SLAM::ORBVocabulary Vocabulary;
    Vocabulary.load(fsVoc);

    cout << "Vocabulary loaded!" << endl << endl;

    //Create KeyFrame Database
    ORB_SLAM::KeyFrameDatabase Database(Vocabulary);

    //Create the map
    ORB_SLAM::Map World;
    //  FramePub.SetMap(&World);

    //Create Map Publisher for Rviz
    //ORB_SLAM::MapPublisher MapPub(&World);
    //Create Drawers. These are used by the Viewer
    ORB_SLAM::FrameDrawer mpFrameDrawer(&World);
    ORB_SLAM::MapDrawer mpMapDrawer(&World, strSettingsFile);
    //begin imu thread
    float m_data[4][4] = {0.0148655429818, -0.999880929698, 0.00414029679422, -0.0216401454975,
         0.999557249008, 0.0149672133247, 0.025715529948, -0.064676986768,
        -0.0257744366974, 0.00375618835797, 0.999660727178, 0.00981073058949,
         0.0, 0.0, 0.0, 1.0};
    Mat IMU_Camera_CalibMatrix(4,4,CV_32F , m_data);
    //system("pause");
    cout << "IMU _ Camera _ CalibMatrix :"<<IMU_Camera_CalibMatrix<<endl;
    char* imu_file = "../Data/mav0/imu/imu.txt";
    IMUMeasurementList imu(IMU_Camera_CalibMatrix.inv(), imu_file);
    //haoliliang add
    char* video_file = "../Data/mav0/camera/cam01.avi";
    char* video_time_file = "../Data/mav0/camera/timestamples.txt";
    
    cv::VideoCapture cap;
    cap.open(video_file);
    ORB_SLAM::Tracking Tracker(&Vocabulary,&mpFrameDrawer, &mpMapDrawer,&World, strSettingsFile, cap);
    Tracker.loadTimestample(video_time_file);    
   //boost::thread TrackingThread(&ORB_SLAM::Tracking::Run,&Tracker);
    Tracker.SetKeyFrameDatabase(&Database);
    //Initialize the Local Mapping Thread and launch
    ORB_SLAM::LocalMapping LocalMapper(&World);

    //Initialize the Loop Closing Thread and launch
    ORB_SLAM::LoopClosing LoopCloser(&World, &Database, &Vocabulary);
 
     ORB_SLAM::Viewer mpViewer(&mpFrameDrawer,&mpMapDrawer,&Tracker,strSettingsFile);
    //Set pointers between thread
    Tracker.SetViewer(&mpViewer);
    Tracker.SetIMU(&imu);
    Tracker.SetLocalMapper(&LocalMapper);
    Tracker.SetLoopClosing(&LoopCloser);

    LocalMapper.SetTracker(&Tracker);
    LocalMapper.SetLoopCloser(&LoopCloser);
    LocalMapper.SetIMU(&imu);

    LoopCloser.SetTracker(&Tracker);
    LoopCloser.SetLocalMapper(&LocalMapper);

    boost::thread IMUThread(&IMUMeasurementList::Run, &imu);
    boost::thread TrackingThread(&ORB_SLAM::Tracking::Run,&Tracker);
    boost::thread loopClosingThread(&ORB_SLAM::LoopClosing::Run, &LoopCloser);
    boost::thread localMappingThread(&ORB_SLAM::LocalMapping::Run,&LocalMapper);
    //Initialize the Viewer thread and launch
    boost::thread mptViewerThread(&ORB_SLAM::Viewer::Run, &mpViewer);
 
    //This "main" thread will show the current processed frame and publish the map
    float fps = fsSettings["Camera.fps"];
    if(fps==0)
        fps=30;

    //ros::Rate r(fps);

    while (true)
    {
    //     FramePub.Refresh();
    //    MapPub.Refresh();
        Tracker.CheckResetByPublishers();
        //r.sleep();
    	usleep(fps);
    }
#if 0
    // Save keyframe poses at the end of the execution
    ofstream f;

    vector<ORB_SLAM::KeyFrame*> vpKFs = World.GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),ORB_SLAM::KeyFrame::lId);

    cout << endl << "Saving Keyframe Trajectory to KeyFrameTrajectory.txt" << endl;
    string strFile = ros::package::getPath("ORB_SLAM")+"/"+"KeyFrameTrajectory.txt";
    f.open(strFile.c_str());
    f << fixed;

    for(size_t i=0; i<vpKFs.size(); i++)
    {
        ORB_SLAM::KeyFrame* pKF = vpKFs[i];

        if(pKF->isBad())
            continue;

        cv::Mat R = pKF->GetRotation().t();
        vector<float> q = ORB_SLAM::Converter::toQuaternion(R);
        cv::Mat t = pKF->GetCameraCenter();
        f << setprecision(6) << pKF->mTimeStamp << setprecision(7) << " " << t.at<float>(0) << " " << t.at<float>(1) << " " << t.at<float>(2)
          << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;

    }
    f.close();
    cout <<"finish" << endl;
    ros::shutdown();
#endif
	return 0;
}
