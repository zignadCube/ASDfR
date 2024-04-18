#pragma once

#include "NativeBase.h"
#include "LoopController.h"

class Native20Sim : public NativeBase
{
public:
    Native20Sim();
    ~Native20Sim();
    
private:
   //Varaibles
   LoopController controller;

protected:
   //Variables
   double u[SIZE_U];
   double y[SIZE_Y];

   //Functions
   int calc() override; 
};
