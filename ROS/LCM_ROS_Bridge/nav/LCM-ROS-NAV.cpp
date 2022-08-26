#include "ros/ros.h"
#include <lcm/lcm-cpp.hpp>
// include ros message type
#include "nav.msg"
// include lcm message type
#include "acfrlcm/auv_acfr_nav_t"

class Handler 
{
    public:
        Handler(ros::NodeHandle n, ros::Publisher pub);
        ~Handler() {};
        void handleMessage(const lcm::ReceiveBuffer* rbuf,
                const std::string& chan, 
                const acfrlcm::auv_acfr_nav_t* msg); 
    
    private:
        ros::NodeHandle _node;
        ros::Publisher _pub;
};

void Handler::Handler(ros::NodeHandle n, ros::Publisher pub)
    : _node(n), _pub(pub)
{
    // nothing else happens
}


void Handler::handleMessage(
    const lcm::ReceiveBuffer* rbuf,
    const std::string& chan, 
    const acfrlcm::auv_acfr_nav_t* msg) 

    {


        // still the default value needs to be fixed
        //int i;
        //printf("Received message on channel \"%s\":\n", chan.c_str());
        //printf("  timestamp   = %lld\n", (long long)msg->timestamp);
        //printf("  position    = (%f, %f, %f)\n",
        //        msg->position[0], msg->position[1], msg->position[2]);
        //printf("  orientation = (%f, %f, %f, %f)\n",
        //        msg->orientation[0], msg->orientation[1], 
        //        msg->orientation[2], msg->orientation[3]);
        //printf("  ranges:");
        //for(i = 0; i < msg->num_ranges; i++)
        //    printf(" %d", msg->ranges[i]);
        //printf("\n");
        //printf("  name        = '%s'\n", msg->name.c_str());
        //printf("  enabled     = %d\n", msg->enabled);
    }




int main (int argc, char ** argv)
{

lcm::LCM lcm;
if (!lcm.good()){ return 1; }



ros::init(argc, argv, "NAV_BRIDGE");
ros::NodeHandle node;
ros::Publisher nav_pub = node.advertise<LCM-ROSS-NAV::nav>("NAV_B_OUTPUT",1000)

Handler handlerObject = new Handler(ros_node, ros_pub_nav);
lcm.subscribe("WAMV.ACFR_NAV", &Handler::handleMessage, &handlerObject);

// no need to do ros::Rate, sleep or anything like that
// it will trigger on a message from the LCM

}

