#pragma once

#include "Native20Sim.h"



class MyApp : public Native20Sim
{
public:
    MyApp();
    ~MyApp();
    
private:
    int wrap_count1 = 0;
    int wrap_count2 = 0;
    int next_enc1 = 0;
    int prev_enc1 = 0;
    int next_enc2 = 0;
    int prev_enc2 = 0;

protected:
    int preProc() override;
    int postProc() override;
};
