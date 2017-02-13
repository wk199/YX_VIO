#include <iostream>
#include <stdlib.h>
#include <fstream>
#include <unistd.h>
#include <time.h>
#include <pthread.h>
#include "MPU9250.h"

using namespace std;
sensor_6axis_t my_data[SENSOR_BUF_SIZE];
