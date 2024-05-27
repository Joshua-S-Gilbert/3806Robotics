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

    virtual void SetPos(double &x, double y);
    virtual void GetPos(double &x, double y);

    std::string modelName;
};




