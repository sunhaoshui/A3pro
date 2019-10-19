#ifndef PIDH
#define PIDH

#include <cstdlib>
#include <utility>
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
class PID
{

public:
    //构造函数
    PID()
    {
        error = 0;
        error_list.push_back(make_pair(0.0,0.0));
        //D_input = 0;
        P_Out = 0;
        I_Out = 0;
        D_Out = 0;
        Output = 0;
        Kf = 0;
        start_intergrate_flag = 0;
    }



    //float D_input, D_input0;

    float Kp;                          //参数P
    float Ki;                          //参数I
    float Kd;                          //参数D
    float Kf;                          //微分项的滤波增益

    float error;                       //误差量 = 实际值 - 期望值
    float delta_time;                  //时间间隔dt

    vector<pair<float, float>> error_list; //误差表,用作计算微分项 平滑窗口 [2nd data, 1st time]

    float P_Out;                       //P环节输出
    float I_Out;                       //I环节输出
    float D_Out;                       //D环节输出
    float Output;                      //输出

    bool start_intergrate_flag;    //是否积分标志[进入offboard(启控)后,才开始积分]
    float Imax;                        //积分上限
    float Output_max;                  //输出最大值
    float errThres;                    //误差死区(if error<errThres, error<0)

    //设置PID参数函数[Kp Ki Kd Kf]
    void setPID(float p_value, float i_value, float d_value, float f_value);

    //输入 误差 和 当前时间
    bool add_error(float input_error, float curtime);
    void show_pid(void);
    //PID输出结果
    void pid_output(void);

    //设置积分上限 控制量最大值 误差死区
    void set_sat(float i_max, float con_max, float thres)
    {
        Output_max = con_max;
        Imax = i_max;
        errThres = thres;
    }

    //饱和函数
    float satfunc(float data, float Max, float Thres)
    {
        if (abs(data)<Thres)
                return 0;
        else if(abs(data)>Max){
                return (data>0)?Max:-Max;
        }
        else{
                return data;
        }
    }

};



void PID::setPID(float p_value, float i_value, float d_value, float f_value)
{
    Kp = p_value;
    Ki = i_value;
    Kd = d_value;
    Kf = f_value;
}


void PID::pid_output(void)
{
    P_Out = Kp * error;                          //P环节输出值
    I_Out = I_Out + Ki *error*delta_time;        //I环节输出值
    I_Out = satfunc(I_Out, Imax, 0);             //I环节限幅[I_Out<=Imax]


    if(start_intergrate_flag == 0)
    {
            I_Out = 0;
    }


    if (error_list.size() < 3 || Kd == 0)
    {
            D_Out = 0; //initiral process
    }
    else
    {
        vector<pair<float, float>>::reverse_iterator error_k = error_list.rbegin();
        vector<pair<float, float>>::reverse_iterator error_k_1 = error_k + 1;
        if(Kf == 0) {
                D_Out = (error_k->second - error_k_1->second)/delta_time * Kd;
        }
        else{
                D_Out = (1- Kf*delta_time) * Kd *D_Out + Kd * Kf *(error_k->second - error_k_1->second);
        }
    }

    Output = P_Out + I_Out + D_Out;
    Output = satfunc(Output, Output_max, errThres);
}
void PID::show_pid(void){
        // cout <<"error_size  "<< error_list.size()<<" delta_time  " << delta_time <<endl;
        cout << "cur_time: " << error_list.end()->first << "  error: "<< error_list.end()->second << "  PID: ["<< P_Out<<"   "<< I_Out<<"   "<<D_Out<<"] "<<std::endl;
}

bool PID::add_error(float input_error, float curtime)
{
    error = input_error;
    if(error_list.size() == 1)
    {
            delta_time = curtime;
    }
    else{
            delta_time = curtime - error_list.rbegin()->first;
            // cout<< "1  "<<"last time  "<< error_list.end()->first <<"delta_time: "<< delta_time<<endl;
    }


    if(error_list.size()<10){
            error_list.push_back(make_pair(curtime, error));
    }
    else{
            // cout<<"*---------*"<<endl;

            vector<pair<float, float>>::iterator k_beg = error_list.begin();
            error_list.erase(k_beg);
            std::pair<float,float > p1(curtime, error);
            // error_list.emplace_back(std::move(p1));
            error_list.emplace_back(p1);
            // float tmp_first = error_list.end()->first;
            /*error_list.end()->first = curtime;
            error_list.end()->second = error;*/
            // printf("time: %f, last time: %f\n", curtime, error_list.rbegin()->first);
    }

    return true;
}


#endif
