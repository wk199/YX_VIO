
#include <iostream>
#include <vector>
#include <string>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/contrib/contrib.hpp>

using namespace std;
using namespace cv;

static void saveXYZ(const char* filename, const Mat& mat)
{
    const double max_z = 1.0e4;
    FILE* fp = fopen(filename, "wt");
    for(int y = 0; y < mat.rows; y++)
    {
        for(int x = 0; x < mat.cols; x++)
        {
            Vec3f point = mat.at<Vec3f>(y, x);
            if(fabs(point[2] - max_z) < FLT_EPSILON || fabs(point[2]) > max_z) continue;
            fprintf(fp, "%f %f %f\n", point[0], point[1], point[2]);
        }
    }
    fclose(fp);
}


int main(){
    
    string strSettingFile = "../Data/Settings_2.yaml";
    cv::FileStorage fsSettings(strSettingFile.c_str() , cv::FileStorage::READ);
    
    if(!fsSettings.isOpened()){
        std::cout << "Failed to open file " << strSettingFile << std::endl;
        return -1;
    }
    
    float fx = fsSettings["Camera.fx"];
    float fy = fsSettings["Camera.fy"];
    float cx = fsSettings["Camera.cx"];
    float cy = fsSettings["Camera.cy"];
    
    cv::Mat K = cv::Mat::eye(3,3,CV_64F);
    K.at<double>(0,0) = fx;
    K.at<double>(1,1) = fy;
    K.at<double>(0,2) = cx;
    K.at<double>(1,2) = cy;
    
    cv::Mat DistCoef(5,1,CV_64F);
    DistCoef.at<double>(0) = fsSettings["Camera.k1"];
    DistCoef.at<double>(1) = fsSettings["Camera.k2"];
    DistCoef.at<double>(2) = fsSettings["Camera.p1"];
    DistCoef.at<double>(3) = fsSettings["Camera.p2"];
    DistCoef.at<double>(4) = 0.0;
    
    string leftImageFile = "stereo_data/img_18.png";
    string rightImageFile = "stereo_data/img_17.png";
    
    cv::Mat leftImage , rightImage;
    
    leftImage = cv::imread(leftImageFile.c_str() , 1);
    rightImage = cv::imread(rightImageFile.c_str() , 1);
    
    //cv::imshow("inputL" , leftImage);
    //cv::imshow("rightL" , rightImage);
    //cv::waitKey(0);
    cv::Mat P1(4,4,CV_64F);
    cv::Mat P2(4,4,CV_64F);
    
    string poseFile = "stereo_data/pose.txt";
    FILE *fp = fopen(poseFile.c_str() , "r");
    for(int i=0;i<4;i++){
        for(int j=0;j<4;j++){
            fscanf(fp, "%lf", &(P2.at<double>(i,j)));
        }
    }  
    for(int i=0;i<4;i++){
        for(int j=0;j<4;j++){
            fscanf(fp, "%lf" , &(P1.at<double>(i,j)));
        }
    }
    fclose(fp);
    
    cout << "P1 = " << P1 << std::endl;
    cout << "P2 = " << P2 << std::endl;
    
    
    cv::Mat dP = P2 * P1.inv();
    cv::Mat R = dP.rowRange(0,3).colRange(0,3);
    cv::Mat T = dP.rowRange(0,3).col(3);
    
    cout << "R = " << R << ", T = " << T << std::endl;
    cv::Mat R1,sP1,R2,sP2;
    cv::Mat Q;
    cv::Rect roi1 , roi2;
    cv::Size img_size = leftImage.size();
    cout << "K = " << K << ", DistCoef = " << DistCoef << std::endl;

    cv::stereoRectify(K , DistCoef , K , DistCoef , img_size , R, T , R1, R2 , sP1 , sP2 , Q ,0 /*CALIB_ZERO_DISPARITY*/, -1 , img_size , & roi1 , &roi2);
    
    //cout << "K = " << K << ", DistCoef = " << DistCoef << std::endl;
    cout << "R1 = " << R1 << endl;
    cout << "R2 = " << R2 << endl;
    cout << "sP1 = " << sP1 << endl;
    cout << "sP2 = " << sP2 << endl;
    cout << "img_size = " << img_size << endl;
    cv::Mat map11, map12 , map21, map22;
    cv::initUndistortRectifyMap(K , DistCoef , R1 , sP1 , img_size , CV_16SC2 , map11 , map12);
    cv::initUndistortRectifyMap(K , DistCoef , R2 , sP2 , img_size , CV_16SC2 , map21 , map22);
    
    cv::Mat leftImager, rightImager;
    cv::remap(leftImage , leftImager , map11 , map12 , INTER_LINEAR);
    cv::remap(rightImage, rightImager, map21 , map22 , INTER_LINEAR);
    
    cv::imshow("left" , leftImager);
    cv::imshow("right" , rightImager);
    if(waitKey() == 'q'){
        //break;
    }
    cv::imwrite("stereo_data/left_remap.png" , leftImager);
    cv::imwrite("stereo_data/right_remap.png" , rightImager);
    /////////////////////////////////////////////////////////////////////////////
    
    int SADWindowSize = 0 , numberOfDisparities = 0;
    numberOfDisparities = numberOfDisparities > 0 ? numberOfDisparities : ((img_size.width/8) + 15) & -16;
    
    cv::StereoSGBM sgbm;
    sgbm.preFilterCap = 63;
    sgbm.SADWindowSize = SADWindowSize > 0 ? SADWindowSize : 3;
    int cn = leftImage.channels();
    sgbm.P1 = 8*cn*sgbm.SADWindowSize*sgbm.SADWindowSize;
    sgbm.P2 = 32*cn*sgbm.SADWindowSize*sgbm.SADWindowSize;
    sgbm.minDisparity = 0;
    sgbm.numberOfDisparities = numberOfDisparities;
    sgbm.uniquenessRatio = 10;
    sgbm.speckleWindowSize = 100;//bm.state->speckleWindowSize;
    sgbm.speckleRange = 32;//bm.state->speckleRange;
    sgbm.disp12MaxDiff = 1;
    sgbm.fullDP = false;//alg == STEREO_HH;
    
    cv::Mat disp; // matching
    
    sgbm(leftImager , rightImager , disp);
    cv::imwrite("stereo_data/disparity.png" , disp);
    cout << "Finish sgbm" << std::endl;
    ///////////////////////////////////////////////////////////////////////////////
    cv::Mat xyz;
    cv::reprojectImageTo3D(disp , xyz , Q , true);
    
    char* point_cloud_file = "stereo_data/pc.pcd";
    saveXYZ(point_cloud_file  , xyz);
    
    ////////////////////////////////////////////////////////////////////////////////
    
    return 0;
    
}
