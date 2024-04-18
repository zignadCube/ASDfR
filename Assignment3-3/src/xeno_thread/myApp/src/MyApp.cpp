#include "MyApp.h"


MyApp::MyApp() 
{
    printf("%s: Constructing rampio\n", _FUNCTION_);
}

MyApp::~MyApp()
{
    printf("%s: Destructing rampio\n", _FUNCTION_);
}

int MyApp::preProc()
{
    evl_printf("enc_1: %d, enc_2: %d\n", FpgaInput.channel1, FpgaInput.channel2);
    XenoData.x = 16383-FpgaInput.channel1;
    XenoData.y = FpgaInput.channel2;
    return 0;
}


int MyApp::postProc()
{  
    FpgaOutput.pwm1 = -int(RosData.x * 2047);
    FpgaOutput.pwm2 = int(RosData.y * 2047);
    evl_printf("Rosdata x: %f, y: %f\n", RosData.x, RosData.y);
    // FpgaOutput.pwm1 = 0;
    // FpgaOutput.pwm2 = 0;
    return 0;
}