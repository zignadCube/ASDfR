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
    evl_printf("enc_1: %d, enc_2: %d\n", FpgaInput.channel1, FpgaInput.channel2);
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

    u[0] = XenoData.x;
    u[1] = XenoData.y;

    u[2] = RosData.x;
    u[3] = RosData.y;

    evl_printf("enc_1 wrap: %d, enc_2 wrap: %d\n", XenoData.x, XenoData.y);
    prev_enc1 = next_enc1;
    prev_enc2 = next_enc2;
    return 0;
}


int MyApp::postProc()
{  
    FpgaOutput.pwm1 = -y[1];
    FpgaOutput.pwm2 = y[0];
    evl_printf("Rosdata x: %f, y: %f\n", RosData.x, RosData.y);
    // FpgaOutput.pwm1 = 0;
    // FpgaOutput.pwm2 = 0;
    return 0;
}