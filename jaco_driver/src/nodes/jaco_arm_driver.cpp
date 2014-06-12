////============================================================================
//// Name        : jaco_arm_driver.cpp
//// Author      : WPI, Clearpath Robotics
//// Version     : 0.5
//// Copyright   : BSD
//// Description : A ROS driver for controlling the Kinova Jaco robotic manipulator arm
////============================================================================


//#include "jaco_driver/jaco_arm.h"
//#include "jaco_driver/jaco_pose_action.h"
//#include "jaco_driver/jaco_angles_action.h"
//#include "jaco_driver/jaco_fingers_action.h"

//float get_xmlrpc_value(XmlRpc::XmlRpcValue &value)
//{
//    if (value.getType() == XmlRpc::XmlRpcValue::TypeDouble)
//    {
//        return static_cast<double>(value);
//    }
//    else if (value.getType() == XmlRpc::XmlRpcValue::TypeInt)
//    {
//        return (float) static_cast<int>(value);
//    }

//    throw std::string("Parameter 'home_position' must contain only numerical values");
//    return 0.0;
//}

//jaco::JacoAngles get_home_position(ros::NodeHandle &nh)
//{
//    std::string key;
//    jaco::JacoAngles home;

//    //if (nh.searchParam("~home_position", key))
//    if (ros::param::has("~home_position"))
//    {
//        XmlRpc::XmlRpcValue joints_list;
//        ros::param::get("~home_position", joints_list);
//        ROS_ASSERT(joints_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
//        ROS_ASSERT(joints_list.size() == 6);

//        try
//        {
//            home.Actuator1 = get_xmlrpc_value(joints_list[0]);
//            home.Actuator2 = get_xmlrpc_value(joints_list[1]);
//            home.Actuator3 = get_xmlrpc_value(joints_list[2]);
//            home.Actuator4 = get_xmlrpc_value(joints_list[3]);
//            home.Actuator5 = get_xmlrpc_value(joints_list[4]);
//            home.Actuator6 = get_xmlrpc_value(joints_list[5]);

//            return home;
//        }
//        catch (std::string msg)
//        {
//            ROS_ERROR("%s", msg.c_str());
//            // use default home from below
//        }
//    }

//    // typical home position for a "right-handed" arm
//    home.Actuator1 = 282.8;
//    home.Actuator2 = 154.4;
//    home.Actuator3 = 43.1;
//    home.Actuator4 = 230.7;
//    home.Actuator5 = 83.0;
//    home.Actuator6 = 78.1;

//    return home;
//}

//int main(int argc, char **argv) {
//    ros::init(argc, argv, "jaco_arm_driver");
//    ros::NodeHandle nh;


//    ROS_INFO("Initializing the Arm");
//    jaco::JacoComm comm(get_home_position(nh));
////    JoystickCommand virtualCommand;
////    virtualCommand.Rotate = 0;

////    comm.homeArm();
////    comm.initFingers();

////    virtualCommand.Rotate = 1;

//    //create the arm object
//    // TODO: Change this to use composition instead of little bits haphazardly flying in formation
//    jaco::JacoArm jaco(comm, nh);
//    jaco::JacoPoseActionServer pose_server(comm, nh);
//    jaco::JacoAnglesActionServer angles_server(comm, nh);
//    jaco::JacoFingersActionServer fingers_server(comm, nh);

//    ros::spin();
//    return 0;
//}






#include <stdio.h>
#include <string.h>
#include <iostream>
#include <dlfcn.h> //Ubuntu
#include <vector>
#include "KinovaTypes.h"
#include "Kinova.API.CommLayerUbuntu.h"
#include "Kinova.API.UsbCommandLayerUbuntu.h"
//Note that under windows, you may/will have to perform other #include

using namespace std;

template<class T>
void printRawBytes(const T& something) {
    void const *qs = static_cast<void const *>(&something);
    unsigned char const *p = static_cast<unsigned char const *>(qs);
    for (size_t i=0; i<sizeof(something); i++) {
        printf("%02x ", p[i]);
    }
    putchar('\n');
    fflush(stdout);
}

/**
 * @example Example of the function GetQuickStatus().
 */
int main()
{
    int result;

    cout << "GetQuickStatus function example" << endl;

    //Handle for the library's command layer.
    void * commandLayer_handle;

    //Function pointers to the functions we need
    int (*MyInitAPI)();
    int (*MyCloseAPI)();
    int (*MyGetQuickStatus)(QuickStatus &Response);
    int (*MyGetGripperStatus)(Gripper &);
    int (*MyGetGeneralInformations)(GeneralInformations &);
    int (*MyStartControlAPI)();
    int (*MyMoveHome)(int &);
    int (*MySendJoystickCommand)(JoystickCommand command);
    int (*MyGetAngularPosition)(AngularPosition &);
    int (*MyGetDevices)(vector<KinovaDevice> &, int &);
    int (*MyGetAPIVersion)(std::vector<int> &);
    int (*MySetActiveDevice)(KinovaDevice);

    //We load the library (Under Windows, use the function LoadLibrary)
    commandLayer_handle = dlopen("Kinova.API.USBCommandLayerUbuntu.so",RTLD_NOW|RTLD_GLOBAL);
    std::cout << COMMAND_LAYER_VERSION << "\n";

    //We load the functions from the library (Under Windows, use GetProcAddress)
    MyInitAPI = (int (*)()) dlsym(commandLayer_handle,"InitAPI");
    MyCloseAPI = (int (*)()) dlsym(commandLayer_handle,"CloseAPI");
    MyGetQuickStatus = (int (*)(QuickStatus &)) dlsym(commandLayer_handle,"GetQuickStatus");
    MyGetGripperStatus = (int (*)(Gripper &)) dlsym(commandLayer_handle,"GetGripperStatus");
    MyGetGeneralInformations = (int (*)(GeneralInformations &)) dlsym(commandLayer_handle,"GetGeneralInformations");
    MyStartControlAPI = (int (*)()) dlsym(commandLayer_handle,"StartControlAPI");
    MyMoveHome = (int (*)(int &)) dlsym(commandLayer_handle,"MoveHome");
    MySendJoystickCommand = (int (*)(JoystickCommand)) dlsym(commandLayer_handle,"SendJoystickCommand");
    MyGetAngularPosition = (int (*)(AngularPosition &)) dlsym(commandLayer_handle,"GetAngularPosition");
    MyGetDevices = (int (*)(vector<KinovaDevice> &, int &)) dlsym(commandLayer_handle,"GetDevices");
    MySetActiveDevice = (int (*)(KinovaDevice)) dlsym(commandLayer_handle,"SetActiveDevice");
    MyGetAPIVersion = (int (*)(std::vector<int> &)) dlsym(commandLayer_handle,"GetAPIVersion");

    //If the was loaded correctly
    if((MyInitAPI == NULL) || (MyCloseAPI == NULL) || (MyGetQuickStatus == NULL)
            || (MyGetGripperStatus == NULL) || (MyGetGeneralInformations == NULL)
            || (MyMoveHome == NULL) || (MySendJoystickCommand == NULL)
            || (MyGetAngularPosition == NULL) || (MyGetAPIVersion == NULL))
    {
        cout << "Unable to initialize the command layer." << endl;
    }
    else
    {
        cout << "The command has been initialized correctly." << endl << endl;


        std::vector<int> version;
        result = (*MyGetAPIVersion)(version);
        cout << "API's version is : " << version[0] << "." << version[1] << "." << version[2] << endl;



        cout << "Calling the method InitAPI()" << endl;
        result = (*MyInitAPI)();
        cout << "result of InitAPI() = " << result << endl << endl;


        QuickStatus status;
        memset(&status, 0, sizeof(status));  // zero structure

        printf("Quick status: ");
        printRawBytes(status);

        result = (*MyGetQuickStatus)(status);

        printf("Quick status: ");
        printRawBytes(status);



        //A vector that holds all devices found on the USB bus.
        vector<KinovaDevice> devicesList;

        AngularPosition angular_pos;

        int resultGetList = NO_ERROR_KINOVA;
        if(result == NO_ERROR_KINOVA)
        {
            //We get the list of devices found on the USB bus.
            (*MyGetDevices)(devicesList, result);
            if(result == NO_ERROR_KINOVA)
            {
                //For each device found
                for(int deviceCount = 0; deviceCount < devicesList.size(); deviceCount++)
                {
                    //We set the current item of the list as the active device
                    (*MySetActiveDevice)(devicesList[deviceCount]);
                    //We get the angular position on the active device
                    result = (*MyGetAngularPosition)(angular_pos);
                    if(result == NO_ERROR_KINOVA)
                    {
                        cout << "DEVICE #" << deviceCount << endl;
                        cout << "Actuator 1 : " << angular_pos.Actuators.Actuator1 << "°" << endl;
                        cout << "Actuator 2 : " << angular_pos.Actuators.Actuator2 << "°" << endl;
                        cout << "Actuator 3 : " << angular_pos.Actuators.Actuator3 << "°" << endl;
                        cout << "Actuator 4 : " << angular_pos.Actuators.Actuator4 << "°" << endl;
                        cout << "Actuator 5 : " << angular_pos.Actuators.Actuator5 << "°" << endl;
                        cout << "Actuator 6 : " << angular_pos.Actuators.Actuator6 << "°" << endl;
                        cout << "Position Finger 1 : " << angular_pos.Fingers.Finger1 << endl;
                        cout << "Position Finger 2 : " << angular_pos.Fingers.Finger2 << endl;
                        cout << "Position Finger 3 : " << angular_pos.Fingers.Finger3 << endl << endl << endl << endl;
                    }
                    else
                    {
                        cout << "Error code = " << result << endl;
                    }
                }
            }
        }

        //        Gripper gripper;
        //        memset(&gripper, 0, sizeof(gripper));  // zero structure

        //        printf("Gripper: ");
        //        printRawBytes(gripper);

        //        result = (*MyGetGripperStatus)(gripper);

        //        printf("Gripper: ");
        //        printRawBytes(gripper);

        //        cout << "Gripper's model : " << gripper.Model << endl;

        //        for(int i = 0; i < 3; i++)
        //        {
        //            cout << "Finger #" << (i + 1) << endl;
        //            cout << "Finger #" << (i + 1) << "'s actual acceleration : " << gripper.Fingers[i].ActualAcceleration << endl;
        //            cout << "Finger #" << (i + 1) << "'s actual average current : " << gripper.Fingers[i].ActualAverageCurrent << endl;
        //            cout << "Finger #" << (i + 1) << "'s actual command : " << gripper.Fingers[i].ActualCommand << endl;
        //            cout << "Finger #" << (i + 1) << "'s actual current : " << gripper.Fingers[i].ActualCurrent << endl;
        //            cout << "Finger #" << (i + 1) << "'s actual force : " << gripper.Fingers[i].ActualForce << endl;
        //            cout << "Finger #" << (i + 1) << "'s actual position : " << gripper.Fingers[i].ActualPosition << endl;
        //            cout << "Finger #" << (i + 1) << "'s actual speed : " << gripper.Fingers[i].ActualSpeed << endl;
        //            cout << "Finger #" << (i + 1) << "'s actual temperature : " << gripper.Fingers[i].ActualTemperature << endl;
        //            cout << "Finger #" << (i + 1) << "'s code version : " << gripper.Fingers[i].CodeVersion << endl;
        //            cout << "Finger #" << (i + 1) << "'s communication errors : " << gripper.Fingers[i].CommunicationErrors << endl;
        //            cout << "Finger #" << (i + 1) << "'s cycle count : " << gripper.Fingers[i].CycleCount << endl;
        //            cout << "Finger #" << (i + 1) << "'s device ID : " << gripper.Fingers[i].DeviceID << endl;
        //            cout << "Finger #" << (i + 1) << "'s finger address : " << gripper.Fingers[i].FingerAddress << endl;
        //            cout << "Finger #" << (i + 1) << "'s ID : " << gripper.Fingers[i].ID << endl;
        //            cout << "Finger #" << (i + 1) << "'s index : " << gripper.Fingers[i].Index << endl;
        //            cout << "Finger #" << (i + 1) << "'s is finger connected : " << gripper.Fingers[i].IsFingerConnected << endl;
        //            cout << "Finger #" << (i + 1) << "'s is finger init : " << gripper.Fingers[i].IsFingerInit << endl;
        //            cout << "Finger #" << (i + 1) << "'s max acceleration : " << gripper.Fingers[i].MaxAcceleration << endl;
        //            cout << "Finger #" << (i + 1) << "'s max angle : " << gripper.Fingers[i].MaxAngle << endl;
        //            cout << "Finger #" << (i + 1) << "'s max current : " << gripper.Fingers[i].MaxCurrent << endl;
        //            cout << "Finger #" << (i + 1) << "'s max force : " << gripper.Fingers[i].MaxForce << endl;
        //            cout << "Finger #" << (i + 1) << "'s max speed : " << gripper.Fingers[i].MaxSpeed << endl;
        //            cout << "Finger #" << (i + 1) << "'s min angle : " << gripper.Fingers[i].MinAngle << endl;
        //            cout << "Finger #" << (i + 1) << "'s oscillator tuning value : " << gripper.Fingers[i].OscillatorTuningValue << endl;
        //            cout << "Finger #" << (i + 1) << "'s peak current : " << gripper.Fingers[i].PeakCurrent << endl;
        //            cout << "Finger #" << (i + 1) << "'s peak max temp : " << gripper.Fingers[i].PeakMaxTemp << endl;
        //            cout << "Finger #" << (i + 1) << "'s peak min temp : " << gripper.Fingers[i].PeakMinTemp << endl;
        //            cout << "Finger #" << (i + 1) << "'s run time : " << gripper.Fingers[i].RunTime << endl << endl;
        //        }


        GeneralInformations info;
        memset(&info, 0, sizeof(info));  // zero structure

        //        printf("General information: ");
        //        printRawBytes(info);

        result = (*MyGetGeneralInformations)(info);

        //        printf("General information: ");
        //        printRawBytes(info);


        cout << "Joystick command: "
             << "  0: " << info.ActualJoystickCommand.ButtonValue[0]
             << ", 1: " << info.ActualJoystickCommand.ButtonValue[1]
             << ", 2: " << info.ActualJoystickCommand.ButtonValue[2]
             << ", 3: " << info.ActualJoystickCommand.ButtonValue[3]
             << ", 4: " << info.ActualJoystickCommand.ButtonValue[4]
             << ", 5: " << info.ActualJoystickCommand.ButtonValue[5]
             << ", 6: " << info.ActualJoystickCommand.ButtonValue[6] << "\n";

        cout << "Controller: " << info.Controller << endl;
        cout << "We take control of the robotic arm." << endl;
        result = (*MyStartControlAPI)();
        cout << "Controller: " << info.Controller << endl;




        //We prepare the virtual joystick command that will be sent to the robotic arm.
        JoystickCommand virtualCommand;
        //Initializing the command.
        for(int i = 0; i < JOYSTICK_BUTTON_COUNT; i++)
        {
            virtualCommand.ButtonValue[i] = 0;
        }
//        virtualCommand.ButtonValue[1] = 1;
        virtualCommand.InclineForwardBackward = 0;
        virtualCommand.InclineLeftRight = 0;
        virtualCommand.MoveForwardBackward = 0;
        virtualCommand.MoveLeftRight = 0;
        virtualCommand.PushPull = 0;
        virtualCommand.Rotate = 0;
        virtualCommand.Rotate = 0;
        int home_int = 0;
        (*MyMoveHome)(home_int);
//         usleep(10000);
        (*MySendJoystickCommand)(virtualCommand);
        virtualCommand.Rotate = 1;
        cout << "Sending the command to the robotic arm." << endl;
        //Every 1 ms we send a new virtual joystick command
        for(int i = 0; i < 400; i++)
        {
            //(*MyMoveHome)(home_int);
            (*MySendJoystickCommand)(virtualCommand);
            usleep(1000);
        }
        //Waiting 3 seconds
        cout << "The robotic arm will now move backward from where it comes." << endl;
        //we send the command backward
        virtualCommand.Rotate = -1;
        cout << "Sending the command backward to the robotic arm." << endl;
        //Every 1 ms we send a new virtual joystick command
        for(int i = 0; i < 400; i++)
        {
            (*MySendJoystickCommand)(virtualCommand);
            usleep(1000);
        }
        virtualCommand.Rotate = 0;
        (*MySendJoystickCommand)(virtualCommand);



        result = (*MyGetQuickStatus)(status);

        printf("Quick status: ");
        printRawBytes(status);

        //        //Sending the robot to the READY(HOME) position.
        //        cout << "Moving home\n";
        //        int data;
        //        result = (*MyMoveHome)(data);

        cout << endl << "Calling the method CloseAPI()" << endl;
        result = (*MyCloseAPI)();
        cout << "result of CloseAPI() = " << result << endl;
    }

    return 0;
}

