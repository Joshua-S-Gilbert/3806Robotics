#include <string>
#include <stdio.h>
#include <cstdlib>
#include <vector>
#include <ros/ros.h>

class AbstractSimulator
{
private:
    /* data */
public:
    virtual AbstractSimulator(std::string name);

    virtual bool SetPos(double &x, double y);
    virtual bool GetPos(double &x, double y);

    std::string modelName;
};

virtual AbstractSimulator::AbstractSimulator(std::string name){
    modelName = name;
}


