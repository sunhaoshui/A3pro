#ifndef PARAMETER
#define PARAMETER

#include <cstdlib>
#include <stdlib.h>
#include <iostream>
#include <stdio.h>
#include <fstream>
#include <math.h>
#include "string"
#include <time.h>
#include <queue>
#include <vector>


using namespace std;
class parameter{

public://位置环就是一个p，速度环就是一个pi
	float x_p;
	float x_i;
	float x_d;
	float x_f;

	float y_p;
	float y_i;
	float y_d;
	float y_f;

	float z_p;
	float z_i;
	float z_d;
	float z_f;

	float vx_p;
	float vx_i;
	float vx_d;
	float vx_f;

	float vy_p;
	float vy_i;
	float vy_d;
	float vy_f;

  float vz_p;
  float vz_i;
  float vz_d;
  float vz_f;

 	bool readParam(const char* addr);

};

bool parameter::readParam(const char* addr)
{
    std::ifstream fs;

    std::string name = "";
    float value[8];

    fs.open(addr);

    if (!fs)
    {
        std::cout << "parameter file err" << std::endl;
        return 0;
    }

    while (!fs.eof())
    {
        fs >> name >> value[0] >> value[1] >> value[2] >> value[3];

        if (name == "!")
        {
            x_p = value[0];
            x_i = value[1];
            x_d = value[2];
            x_f = value[3];
        }

        if (name == "@")
        {
            y_p = value[0];
            y_i = value[1];
            y_d = value[2];
            y_f = value[3];
        }
        if (name == "#")
        {
            z_p = value[0];
            z_i = value[1];
            z_d = value[2];
            z_f = value[3];

        }
        if(name == "$")
        {
            vx_p = value[0];
            vx_i = value[1];
            vx_d = value[2];
            vx_f = value[3];
            
        }
        if (name == "^")
        {
            vy_p = value[0];
            vy_i = value[1];
            vy_d = value[2];
            vy_f = value[3];
            
        }
        if (name == "&")
        {
            vz_p = value[0];
            vz_i = value[1];
            vz_d = value[2];
            vz_f = value[3];
            
        }

    }
    std::cout << "read config file successfully!"<<std::endl;

    fs.close();

    return 1;
}

#endif
