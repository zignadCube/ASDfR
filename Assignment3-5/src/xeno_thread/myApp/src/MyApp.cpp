#include "MyApp.h"


MyApp::MyApp() 
{
    printf("%s: Constructing rampio\n", __FUNCTION__);
}

MyApp::~MyApp()
{
    printf("%s: Destructing rampio\n", __FUNCTION__);
}

int MyApp::preProc()
{
    //evl_printf("enc_1: %d, enc_2: %d\n", FpgaInput.channel1, FpgaInput.channel2);
    next_enc1 = 16383-FpgaInput.channel1;
    next_enc2 = FpgaInput.channel2;

    if (next_enc1 > 16000 && prev_enc1 < 300){
        wrap_count1--;
    }else if (next_enc1 < 300 && prev_enc1 > 16000){
        wrap_count1++;
    }
    if (next_enc2 > 16000 && prev_enc2 < 300){
        wrap_count2--;
    }else if (next_enc2 < 300 && prev_enc2 > 16000){
        wrap_count2++;
    }

    XenoData.y = next_enc1 + wrap_count1*16383;
    XenoData.x = next_enc2 + wrap_count2*16383;

    u[0] = ((XenoData.x%63816)*6.2832/63816.0);
    u[1] = ((XenoData.y%63816)*6.2832/63816.0);

    u[2] = RosData.x;//RosData.x*6.2832/63816.0;
    u[3] = RosData.y;//RosData.y*6.2832/63816.0;

    evl_printf("PosLeft: %f, PosRight: %f\n", u[0], u[1]);
    //evl_printf("XenoData.x: %d, XenoData.y: %d\n", XenoData.x, XenoData.y);
    evl_printf("SetVelLeft: %f, SetVelRight: %f\n", u[2], u[3]);
    prev_enc1 = next_enc1;
    prev_enc2 = next_enc2;
    return 0;
}


int MyApp::postProc()
{  
    if(y[1] > 100){
        FpgaOutput.pwm1 = -2047;
    }else{
        FpgaOutput.pwm1 = -int(y[1]*20.47);
    }
    if(y[0] > 100){
        FpgaOutput.pwm2 = 2047;
    }else{
        FpgaOutput.pwm2 = int(y[0]*20.47);
    }
    
    // FpgaOutput.pwm1 = -int(RosData.y*500.0);
    // FpgaOutput.pwm2 = int(RosData.x*500.0);
    
    evl_printf("SteerLeft: %f, SteerRight: %f\n", y[0], y[1]);
    return 0;
}