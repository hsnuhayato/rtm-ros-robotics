#include "fitting.h"

#include <ros/node_handle.h>
#include <ros/master.h>

int main(int argc, char ** argv)
{
    ros::init(argc,argv,"openrave_tfsender");
    if( !ros::master::check() ) {
        return 1;
    }

    DetectEllipseNodePtr node = CreateDetectEllipseNode();
    ros::spin();
    node.reset();
    return 0;
}
