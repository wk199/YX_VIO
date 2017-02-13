

#include <iostream>
#include <vector>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>

using namespace std;
using namespace cv;
using namespace pcl;

int 
main (int argc, char** argv)
{
    float w = 0.2;
    float h = 1.0 * w;
    float z = 0.8 * w;
    float cam_data[20] = {
        w,w,-w,-w,0,
        h,-h,-h,h,0,
        z,z,z,z,0,
        1,1,1,1,1
    };
    cv::Mat camPointMat(4,5,CV_32F,cam_data);
    cout << camPointMat<<endl;
    
    ////////////////////////////////////////////////
    // read the orb mat data
    char *orb_file = "../Data/orb_1.txt";
    FILE *fp = fopen(orb_file , "r");
    vector<cv::Mat> orbPoseMat;
    int orb_num = 0;
    fscanf(fp , "%d" , &orb_num);
    for(int k=0;k<orb_num;k++){
        float vv;
        cv::Mat _t_pose(4,4,CV_32F);
        for(int i=0;i<4;i++){
            for(int j=0;j<4;j++){
                fscanf(fp, "%f" , &vv);
                _t_pose.at<float>(i,j) = vv;
            }
        }
        
        orbPoseMat.push_back(_t_pose);
        //fscanf(fp, "%f" , &vv);
    }
    fclose(fp);
    // finish read the data
    ////////////////////////////////////////////////
    
    ////////////////////////////////////////////////
    // read the ISAM2 mat data
    char* isam2_file = "../Data/orb_2.txt";
    fp = fopen(isam2_file , "r");
    vector<cv::Mat> isam2PoseMat;
    int isam2_num = 0;
    fscanf(fp , "%d" , &isam2_num);
    for(int k=0;k<isam2_num;k++){
        float vv;
        cv::Mat _t_pose(4,4,CV_32F);
        for(int i=0;i<4;i++){
            for(int j=0;j<4;j++){
                fscanf(fp, "%f" , &vv);
                _t_pose.at<float>(i,j) = vv;
            }
        }
        isam2PoseMat.push_back(_t_pose);
    }
    fclose(fp);
    // finish read the data
    ////////////////////////////////////////////////
    
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr basic_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
    
    uint8_t r(255), g(15), b(15);
    for(int i=0;i<orbPoseMat.size();i++){
        cv::Mat pose_i = orbPoseMat[i];
        cv::Mat pointMat = pose_i.inv() * camPointMat;
        //cout << "Frame " << i << endl;
        //cout << "Pose " << endl;
        //cout << pose_i << endl;
        //cout << pointMat << endl;
        for(int k=0;k<5;k++){
            cv::Mat p_k = pointMat.rowRange(0,3).col(k);
            cout << p_k << endl;
            pcl::PointXYZRGB _p;
            _p.x = p_k.at<float>(0);
            _p.y = p_k.at<float>(1);
            _p.z = p_k.at<float>(2);
            uint32_t rgb = (static_cast<uint32_t>(r) << 16 |
              static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));
            _p.rgb = rgb;
            point_cloud_ptr->points.push_back(_p);
        }
    }
    for(int i=0;i<isam2PoseMat.size();i++){
        cv::Mat pose_i = isam2PoseMat[i];
        cv::Mat pointMat = pose_i.inv() * camPointMat;
        for(int k=0;k<5;k++){
            cv::Mat p_k = pointMat.rowRange(0,3).col(k);
            cout << p_k << endl;
            pcl::PointXYZRGB _p;
            _p.x = p_k.at<float>(0);
            _p.y = p_k.at<float>(1);
            _p.z = p_k.at<float>(2);
            uint32_t rgb = (static_cast<uint32_t>(r) << 16 |
              static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));
            _p.rgb = rgb;
            point_cloud_ptr->points.push_back(_p);
        }
    }
    ////////////////////////////////////////////////////
    
    point_cloud_ptr->width = (int)point_cloud_ptr->points.size();
    point_cloud_ptr->height = 1;
    
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new 
                pcl::visualization::PCLVisualizer ("3D Viewer"));
    
    viewer->setBackgroundColor(0, 0, 0);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> _rgb(point_cloud_ptr);
    viewer->addPointCloud<pcl::PointXYZRGB> (point_cloud_ptr , _rgb, "sample cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();
    
    //////////////////////////////////////////////////////
    // draw the Cameras
    //////////////////////////////////////////////////////
    int line_num = 0;
    double rr,gg,bb;
    rr = 1.0; gg = 15.0 / 255; bb = 15.0 / 255;
    double rr1,gg1,bb1;
    rr1 = 15.0 / 255.0; gg1 = 15.0 / 255.0; bb1 = 1.0;
    char buf[255];
    for(int i=0;i<orbPoseMat.size();i++){
        int basic_index = i * 5;
        sprintf(buf, "line%d" , line_num ++);
        viewer->addLine<pcl::PointXYZRGB>(
            point_cloud_ptr->points[basic_index],
            point_cloud_ptr->points[basic_index + 4],
            rr,gg,bb,
            buf
        );
        sprintf(buf, "line%d" , line_num ++);
        viewer->addLine<pcl::PointXYZRGB>(
            point_cloud_ptr->points[basic_index + 1],
            point_cloud_ptr->points[basic_index + 4],
            rr,gg,bb,
            buf
        );
        sprintf(buf, "line%d" , line_num ++);
        viewer->addLine<pcl::PointXYZRGB>(
            point_cloud_ptr->points[basic_index + 2],
            point_cloud_ptr->points[basic_index + 4],
            rr,gg,bb,
            buf
        );
        sprintf(buf, "line%d" , line_num ++);
        viewer->addLine<pcl::PointXYZRGB>(
            point_cloud_ptr->points[basic_index + 3],
            point_cloud_ptr->points[basic_index + 4],
            rr,gg,bb,
            buf
        );
        sprintf(buf, "line%d" , line_num ++);
        viewer->addLine<pcl::PointXYZRGB>(
            point_cloud_ptr->points[basic_index],
            point_cloud_ptr->points[basic_index + 1],
            rr,gg,bb,
            buf
        );
        sprintf(buf, "line%d" , line_num ++);
        viewer->addLine<pcl::PointXYZRGB>(
            point_cloud_ptr->points[basic_index + 1],
            point_cloud_ptr->points[basic_index + 2],
            rr,gg,bb,
            buf
        );
        sprintf(buf, "line%d" , line_num ++);
        viewer->addLine<pcl::PointXYZRGB>(
            point_cloud_ptr->points[basic_index + 2],
            point_cloud_ptr->points[basic_index + 3],
            rr,gg,bb,
            buf
        );
        sprintf(buf, "line%d" , line_num ++);
        viewer->addLine<pcl::PointXYZRGB>(
            point_cloud_ptr->points[basic_index + 3],
            point_cloud_ptr->points[basic_index ],
            rr,gg,bb,
            buf
        );
        if(i > 0){
            sprintf(buf, "line%d" , line_num ++);
            viewer->addLine<pcl::PointXYZRGB>(
                point_cloud_ptr->points[basic_index + 4],
                point_cloud_ptr->points[basic_index - 1],
                rr,gg,bb,
                buf
            );           
        }
        
    } 

    for(int i=0;i<isam2PoseMat.size();i++){
        int basic_index = i * 5 + orb_num * 5;
        sprintf(buf, "line%d" , line_num ++);
        viewer->addLine<pcl::PointXYZRGB>(
            point_cloud_ptr->points[basic_index],
            point_cloud_ptr->points[basic_index + 4],
            rr1,gg1,bb1,
            buf
        );
        sprintf(buf, "line%d" , line_num ++);
        viewer->addLine<pcl::PointXYZRGB>(
            point_cloud_ptr->points[basic_index + 1],
            point_cloud_ptr->points[basic_index + 4],
            rr1,gg1,bb1,
            buf
        );
        sprintf(buf, "line%d" , line_num ++);
        viewer->addLine<pcl::PointXYZRGB>(
            point_cloud_ptr->points[basic_index + 2],
            point_cloud_ptr->points[basic_index + 4],
            rr1,gg1,bb1,
            buf
        );
        sprintf(buf, "line%d" , line_num ++);
        viewer->addLine<pcl::PointXYZRGB>(
            point_cloud_ptr->points[basic_index + 3],
            point_cloud_ptr->points[basic_index + 4],
            rr1,gg1,bb1,
            buf
        );
        sprintf(buf, "line%d" , line_num ++);
        viewer->addLine<pcl::PointXYZRGB>(
            point_cloud_ptr->points[basic_index],
            point_cloud_ptr->points[basic_index + 1],
            rr1,gg1,bb1,
            buf
        );
        sprintf(buf, "line%d" , line_num ++);
        viewer->addLine<pcl::PointXYZRGB>(
            point_cloud_ptr->points[basic_index + 1],
            point_cloud_ptr->points[basic_index + 2],
            rr1,gg1,bb1,
            buf
        );
        sprintf(buf, "line%d" , line_num ++);
        viewer->addLine<pcl::PointXYZRGB>(
            point_cloud_ptr->points[basic_index + 2],
            point_cloud_ptr->points[basic_index + 3],
            rr1,gg1,bb1,
            buf
        );
        sprintf(buf, "line%d" , line_num ++);
        viewer->addLine<pcl::PointXYZRGB>(
            point_cloud_ptr->points[basic_index + 3],
            point_cloud_ptr->points[basic_index ],
            rr1,gg1,bb1,
            buf
        );
        if(i > 0){
            sprintf(buf, "line%d" , line_num ++);
            viewer->addLine<pcl::PointXYZRGB>(
                point_cloud_ptr->points[basic_index + 4],
                point_cloud_ptr->points[basic_index - 1],
                rr1,gg1,bb1,
                buf
            );           
        }
        
    }     
    while(!viewer->wasStopped()){
        viewer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }
    
    
    return 0;
    
}