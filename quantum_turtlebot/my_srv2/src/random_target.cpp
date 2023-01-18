#include "ros/ros.h"
#include "my_srv2/Random.h"


// create random numbers
 double randMToN(double M, double N)
{ return M+(rand()/(RAND_MAX/(N-M)));}

// setting random numbers each for x and y
bool random_target_creater(my_srv2::Random::Request &req, my_srv2::Random::Response &res){ 
    res.x = randMToN(req.min, req.max);
    res.y = randMToN(req.min, req.max);
}
int main(int argc, char **argv)
{ 
    // initialization
    ros::init(argc, argv, "random_target_server");
    ros::NodeHandle n;
    // set server for sending to assignment1
    ros::ServiceServer service= n.advertiseService("/random_target",random_target_creater); 
    ros::spin();
    return 0;
} 
