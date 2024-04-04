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
    return 0;
}


int MyApp::postProc()
{  

    return 0;
}