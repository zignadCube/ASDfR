#pragma once

#include "Native20Sim.h"

class MyApp : public Native20Sim
{
public:
    MyApp();
    ~MyApp();
    
private:
    struct Rosdata{
        float64 x;
        float64 y;
    }

protected:
    int preProc() override;
    int postProc() override;
};
