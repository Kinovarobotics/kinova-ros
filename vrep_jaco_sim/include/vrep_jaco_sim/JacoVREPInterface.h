#ifndef JACO_VREP_INTERFACE_H
#define JACO_VREP_INTERFACE_H

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

class JacoVREPInterface {
    protected:
        ros::NodeHandle nh;
        ros::Publisher jointPosPub[6];
        ros::Publisher jointVelPub[6];
        ros::Subscriber jointSub[6];
        ros::Publisher controlPub;

        struct VelPosData {
            float position;
            float velocity;
            float offset;
            VelPosData() : position(0.0), velocity(0.0), offset(0.0) {}

            void cb(sensor_msgs::JointStateConstPtr msg) {
                assert(msg->position.size()>0);
                assert(msg->velocity.size()>0);
                position = (msg->position[0])/M_PI*180.0+offset;
                velocity = (msg->velocity[0])/M_PI*180.0;
            }
        };

        VelPosData vp[6];
        bool positionControl;

    public:
        JacoVREPInterface();
        ~JacoVREPInterface() {}

       bool getPositions(float &p0, float &p1, float &p2, float &p3,
               float &p4, float &p5) {
           p0 = vp[0].position;
           p1 = vp[1].position;
           p2 = vp[2].position;
           p3 = vp[3].position;
           p4 = vp[4].position;
           p5 = vp[5].position;
           return true;
       }
       bool getVelocities(float &v1, float &v2, float &v3,
               float &v4, float &v5, float &v6) {
           v1 = vp[0].velocity;
           v2 = vp[1].velocity;
           v3 = vp[2].velocity;
           v4 = vp[3].velocity;
           v5 = vp[4].velocity;
           v6 = vp[5].velocity;
           return true;
       }

       bool setPositionControl();

       bool setVelocityControl();
       bool publishControl(float a1, float a2, float a3,
               float a4, float a5, float a6);
};




#endif // JACO_VREP_INTERFACE_H
