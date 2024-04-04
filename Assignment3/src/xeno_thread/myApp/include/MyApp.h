#pragma once

#include "Native20Sim.h"

class MyApp : public Native20Sim
{
public:
    MyApp();
    ~MyApp();
    
private:

protected:
    int preProc() override;
    int postProc() override;
};
