
#include <iostream>
#include <fstream>
#include <Sensor.h>
#include <SensorFusion.h>

using namespace std;

void getdata(SensorFusion* g_sen, float* metrix){
    
    for(int i=0;i<9;i++){
        metrix[i] = g_sen->RotateMetrix[i];
    }
}

int main(){
    
    SensorFusion* g_sensorFusion = new SensorFusion();
    long double tp;
    double acc_x, acc_y, acc_z , gyr_x, gyr_y , gyr_z;
    char* imu_file = "imu.txt";
    FILE *fp = fopen(imu_file , "r");
    fscanf(fp, "%Lf %lf %lf %lf %lf %lf %lf" , &tp,
    &gyr_x ,& gyr_y , &gyr_z , & acc_x , &acc_y , &acc_z);
    std::ofstream fout;
    fout.open("imu_pose.txt");
    fout.setf(ios::fixed, ios::floatfield);
    fout.precision(8);
    double t_first = tp / 1000000000;
    cout << "begin to sensorfusion" << endl;
    cout << t_first << " " << gyr_x << " " << gyr_y << " " << gyr_z << " " << 
    acc_x << " " << acc_y << " " << acc_z << endl;    
    g_sensorFusion->SensorPretreatment(gyr_x , gyr_y , gyr_z);
    g_sensorFusion->handlemessage(gyr_x , gyr_y , gyr_z , acc_x , acc_y , acc_z , 0.005);
    //InitSensorFusion(gyr_x , gyr_y , gyr_z , acc_x , acc_y , acc_z , 0.005);
    float rM[9];
    getdata(g_sensorFusion , rM);
    //GetRotateMetrix(rM);
    fout << t_first << " " << gyr_x << " " << gyr_y << " " << gyr_z << " " << 
    acc_x << " " << acc_y << " " << acc_z << endl;
    for(int i=0;i<9;i++){
        fout << rM[i] << " ";
    }
    fout << endl;

    while(fscanf(fp, "%Lf %lf %lf %lf %lf %lf %lf" , &tp,
    &gyr_x ,& gyr_y , &gyr_z , & acc_x , &acc_y , &acc_z) != EOF){
        cout << "begin to sensorfusion" << endl;
        double tp1 = tp / 1000000000;
        double delta_t = tp1 - t_first;
        t_first = tp1;
        //InitSensorFusion(gyr_x , gyr_y , gyr_z , acc_x , acc_y , acc_z , delta_t);
        g_sensorFusion->handlemessage(gyr_x , gyr_y , gyr_z , acc_x , acc_y , acc_z , 0.005);

        //GetRotateMetrix(rM);
        getdata(g_sensorFusion , rM);
        fout << t_first << " " << gyr_x << " " << gyr_y << " " << gyr_z << " " << 
        acc_x << " " << acc_y << " " << acc_z << endl;
        for(int i=0;i<9;i++){
            fout << rM[i] << " ";
        }
        fout << endl;
    }
    fclose(fp);
    fout.close();
    
    
    return 0;
    
}