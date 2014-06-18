//============================================================================
// Name        : jaco_arm_driver.cpp
// Author      : WPI, Clearpath Robotics
// Version     : 0.5
// Copyright   : BSD
// Description : A ROS driver for controlling the Kinova Jaco robotic manipulator arm
//============================================================================


#include "jaco_driver/jaco_arm.h"
#include "jaco_driver/jaco_pose_action.h"
#include "jaco_driver/jaco_angles_action.h"
#include "jaco_driver/jaco_fingers_action.h"


float getXmlrpcValue(XmlRpc::XmlRpcValue &value)
{
    ROS_ASSERT_MSG((value.getType() == XmlRpc::XmlRpcValue::TypeDouble)
                   || (value.getType() == XmlRpc::XmlRpcValue::TypeInt),
                   "Parameter home_position_degrees must contain only numerical values");

    if (value.getType() == XmlRpc::XmlRpcValue::TypeDouble)
    {
        return static_cast<double>(value);
    }
    else
    {
        return (float)static_cast<int>(value);
    }

    return 0.0f;
}

jaco::JacoAngles getHomePosition(ros::NodeHandle &nh)
{
    // Typical home position for a "right-handed" arm
    jaco::JacoAngles home;
    home.Actuator1 = 282.8;
    home.Actuator2 = 154.4;
    home.Actuator3 = 43.1;
    home.Actuator4 = 230.7;
    home.Actuator5 = 83.0;
    home.Actuator6 = 78.1;

    XmlRpc::XmlRpcValue joints_list;
    if (nh.getParam("home_position_degrees", joints_list))
    {
        ROS_ASSERT_MSG(joints_list.getType() == XmlRpc::XmlRpcValue::TypeArray,
                       "Attempted to get the home position from the parameter server and did not get the "
                       "correct type. Check launch file or other places that may set parameter values.");
        ROS_ASSERT_MSG(joints_list.size() == 6, "Home position on parameter server does not have six (6) elements.");

        home.Actuator1 = getXmlrpcValue(joints_list[0]);
        home.Actuator2 = getXmlrpcValue(joints_list[1]);
        home.Actuator3 = getXmlrpcValue(joints_list[2]);
        home.Actuator4 = getXmlrpcValue(joints_list[3]);
        home.Actuator5 = getXmlrpcValue(joints_list[4]);
        home.Actuator6 = getXmlrpcValue(joints_list[5]);

        ROS_INFO("Loaded home_position_degrees from file: %.2f, %.2f, %.2f, %.2f, %.2f, %.2f",
                 home.Actuator1, home.Actuator2, home.Actuator3, home.Actuator4, home.Actuator5, home.Actuator6);
    } else {
        ROS_INFO("Using default home_position_degrees: %.2f, %.2f, %.2f, %.2f, %.2f, %.2f",
                 home.Actuator1, home.Actuator2, home.Actuator3, home.Actuator4, home.Actuator5, home.Actuator6);
    }

    return home;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "jaco_arm_driver");
    ros::NodeHandle nh("~");

    jaco::JacoComm comm(getHomePosition(nh));
    //    JoystickCommand virtualCommand;
    //    virtualCommand.Rotate = 0;

    //    comm.homeArm();
    //    comm.initFingers();

    //    virtualCommand.Rotate = 1;

    //create the arm object
    // TODO: Change this to use composition instead of little bits flying in formation
    jaco::JacoArm jaco(comm, nh);
    jaco::JacoPoseActionServer pose_server(comm, nh);
    jaco::JacoAnglesActionServer angles_server(comm, nh);
    jaco::JacoFingersActionServer fingers_server(comm, nh);

    ros::spin();
    return 0;
}






//#include <stdio.h>
//#include <string.h>
//#include <iostream>
//#include <dlfcn.h> //Ubuntu
//#include <vector>
//#include "kinova/KinovaTypes.h"
//#include "kinova/Kinova.API.CommLayerUbuntu.h"
//#include "kinova/Kinova.API.UsbCommandLayerUbuntu.h"
////Note that under windows, you may/will have to perform other #include

//using namespace std;


///////////// Angular
//int main()
//{
//    int result;
//    AngularPosition data;
//    cout << "GetAngularPosition function example" << endl;
//    //Handle for the library's command layer.
//    void * commandLayer_handle;
//    //Function pointers to the functions we need
//    int (*MyInitAPI)();
//    int (*MyCloseAPI)();
//    int (*MyGetAngularPosition)(AngularPosition &);
//    int (*MyGetDevices)(vector<KinovaDevice> &, int &);
//    int (*MySetActiveDevice)(KinovaDevice);
//    //We load the library (Under Windows, use the function LoadLibrary)
//    commandLayer_handle = dlopen("Kinova.API.USBCommandLayerUbuntu.so",RTLD_NOW|RTLD_GLOBAL);
//    //A vector that holds all devices found on the USB bus.
//    vector<KinovaDevice> devicesList;
//    //We load the functions from the library (Under Windows, use GetProcAddress)
//    MyInitAPI = (int (*)()) dlsym(commandLayer_handle,"InitAPI");
//    MyCloseAPI = (int (*)()) dlsym(commandLayer_handle,"CloseAPI");
//    MyGetAngularPosition = (int (*)(AngularPosition &)) dlsym(commandLayer_handle,"GetAngularPosition");
//    MyGetDevices = (int (*)(vector<KinovaDevice> &, int &)) dlsym(commandLayer_handle,"GetDevices");
//    MySetActiveDevice = (int (*)(KinovaDevice)) dlsym(commandLayer_handle,"SetActiveDevice");
//    //If the was loaded correctly
//    if((MyInitAPI == NULL) || (MyCloseAPI == NULL) || (MyGetAngularPosition == NULL))
//    {
//        cout << "Unable to initialize the command layer." << endl;
//    }
//    else
//    {
//        cout << "The command has been initialized correctly." << endl << endl;
//        cout << "Calling the method InitAPI()" << endl;
//        //initialise the API
//        result = (*MyInitAPI)();
//        int resultGetList = NO_ERROR_KINOVA;
//        if(result == NO_ERROR_KINOVA)
//        {
//            //We get the list of devices found on the USB bus.
//            (*MyGetDevices)(devicesList, result);
//            if(result == NO_ERROR_KINOVA)
//            {
//                //For each device found
//                for(int deviceCount = 0; deviceCount < devicesList.size(); deviceCount++)
//                {
//                    //We set the current item of the list as the active device
//                    (*MySetActiveDevice)(devicesList[deviceCount]);
//                    //We get the angular position on the active device
//                    result = (*MyGetAngularPosition)(data);
//                    if(result == NO_ERROR_KINOVA)
//                    {
//                        cout << "DEVICE #" << deviceCount << endl;
//                        cout << "Actuator 1 : " << data.Actuators.Actuator1 << "°" << endl;
//                        cout << "Actuator 2 : " << data.Actuators.Actuator2 << "°" << endl;
//                        cout << "Actuator 3 : " << data.Actuators.Actuator3 << "°" << endl;
//                        cout << "Actuator 4 : " << data.Actuators.Actuator4 << "°" << endl;
//                        cout << "Actuator 5 : " << data.Actuators.Actuator5 << "°" << endl;
//                        cout << "Actuator 6 : " << data.Actuators.Actuator6 << "°" << endl;
//                        cout << "Position Finger 1 : " << data.Fingers.Finger1 << endl;
//                        cout << "Position Finger 2 : " << data.Fingers.Finger2 << endl;
//                        cout << "Position Finger 3 : " << data.Fingers.Finger3 << endl << endl << endl << endl;
//                    }
//                    else
//                    {
//                        cout << "Error code = " << result << endl;
//                    }
//                }
//            }
//        }
//        cout << endl << "Calling the method CloseAPI()" << endl;
//        result = (*MyCloseAPI)();
//        cout << "result of CloseAPI() = " << result << endl;
//    }
//    return 0;
//}


////////// Cartesian
//int main()
//{
//    int result;
//    CartesianPosition data;
//    cout << "GetCartesianPosition function example" << endl;
//    //Handle for the library's command layer.
//    void * commandLayer_handle;
//    //Function pointers to the functions we need
//    int (*MyInitAPI)();
//    int (*MyCloseAPI)();
//    int (*MyGetCartesianPosition)(CartesianPosition &);
//    //We load the library (Under Windows, use the function LoadLibrary)
//    commandLayer_handle = dlopen("Kinova.API.USBCommandLayerUbuntu.so",RTLD_NOW|RTLD_GLOBAL);
//    //We load the functions from the library (Under Windows, use GetProcAddress)
//    MyInitAPI = (int (*)()) dlsym(commandLayer_handle,"InitAPI");
//    MyCloseAPI = (int (*)()) dlsym(commandLayer_handle,"CloseAPI");
//    MyGetCartesianPosition = (int (*)(CartesianPosition &)) dlsym(commandLayer_handle,"GetCartesianPosition");
//    //If the was loaded correctly
//    if((MyInitAPI == NULL) || (MyCloseAPI == NULL) || (MyGetCartesianPosition == NULL))
//    {
//        cout << "Unable to initialize the command layer." << endl;
//    }
//    else
//    {
//        cout << "The command has been initialized correctly." << endl << endl;
//        cout << "Calling the method InitAPI()" << endl;
//        result = (*MyInitAPI)();
//        cout << "result of InitAPI() = " << result << endl;
//        result = (*MyGetCartesianPosition)(data);
//        cout << " Position X : " << data.Coordinates.X << endl;
//        cout << " Position Y : " << data.Coordinates.Y << endl;
//        cout << " Position Z : " << data.Coordinates.Z << endl;
//        cout << " Position ThetaX : " << data.Coordinates.ThetaX << endl;
//        cout << " Position ThetaY : " << data.Coordinates.ThetaY << endl;
//        cout << " Position ThetaZ : " << data.Coordinates.ThetaZ << endl;
//        cout << "Position Finger 1 : " << data.Fingers.Finger1 << endl;
//        cout << "Position Finger 2 : " << data.Fingers.Finger2 << endl;
//        cout << "Position Finger 3 : " << data.Fingers.Finger3 << endl;
//        cout << endl << "Calling the method CloseAPI()" << endl;
//        result = (*MyCloseAPI)();
//        cout << "result of CloseAPI() = " << result << endl;
//    }
//    return 0;
//}


//template<class T>
//void printRawBytes(const T& something) {
//    void const *qs = static_cast<void const *>(&something);
//    unsigned char const *p = static_cast<unsigned char const *>(qs);
//    for (size_t i=0; i<sizeof(something); i++) {
//        printf("%02x ", p[i]);
//    }
//    putchar('\n');
//    fflush(stdout);
//}

///**
// * @example Example of the function GetQuickStatus().
// */
//int main()
//{
//    int result;

//    cout << "GetQuickStatus function example" << endl;

//    //Handle for the library's command layer.
//    void * commandLayer_handle;

//    //Function pointers to the functions we need
//    int (*MyInitAPI)();
//    int (*MyCloseAPI)();
//    int (*MyGetQuickStatus)(QuickStatus &Response);
//    int (*MyGetGripperStatus)(Gripper &);
//    int (*MyGetGeneralInformations)(GeneralInformations &);
//    int (*MyStartControlAPI)();
//    int (*MyStopControlAPI)();
//    int (*MyMoveHome)(int &);
//    int (*MySendJoystickCommand)(JoystickCommand command);
//    int (*MyGetAngularPosition)(AngularPosition &);
//    int (*MyGetDevices)(vector<KinovaDevice> &, int &);
//    int (*MyGetAPIVersion)(std::vector<int> &);
//    int (*MySetActiveDevice)(KinovaDevice);
//    int (*MyGetSystemErrorCount)(unsigned int &);
//    int (*MyGetSystemError)(unsigned int, SystemError &);

//    int (*MyClearErrorLog)();

//    //We load the library (Under Windows, use the function LoadLibrary)
//    commandLayer_handle = dlopen("Kinova.API.USBCommandLayerUbuntu.so",RTLD_NOW|RTLD_GLOBAL);
//    std::cout << COMMAND_LAYER_VERSION << "\n";

//    //We load the functions from the library (Under Windows, use GetProcAddress)
//    MyInitAPI = (int (*)()) dlsym(commandLayer_handle,"InitAPI");
//    MyCloseAPI = (int (*)()) dlsym(commandLayer_handle,"CloseAPI");
//    MyGetQuickStatus = (int (*)(QuickStatus &)) dlsym(commandLayer_handle,"GetQuickStatus");
//    MyGetGripperStatus = (int (*)(Gripper &)) dlsym(commandLayer_handle,"GetGripperStatus");
//    MyGetGeneralInformations = (int (*)(GeneralInformations &)) dlsym(commandLayer_handle,"GetGeneralInformations");
//    MyStartControlAPI = (int (*)()) dlsym(commandLayer_handle,"StartControlAPI");
//    MyStopControlAPI = (int (*)()) dlsym(commandLayer_handle,"StopControlAPI");
//    MyMoveHome = (int (*)(int &)) dlsym(commandLayer_handle,"MoveHome");
//    MySendJoystickCommand = (int (*)(JoystickCommand)) dlsym(commandLayer_handle,"SendJoystickCommand");
//    MyGetAngularPosition = (int (*)(AngularPosition &)) dlsym(commandLayer_handle,"GetAngularPosition");
//    MyGetDevices = (int (*)(vector<KinovaDevice> &, int &)) dlsym(commandLayer_handle,"GetDevices");
//    MySetActiveDevice = (int (*)(KinovaDevice)) dlsym(commandLayer_handle,"SetActiveDevice");
//    MyGetAPIVersion = (int (*)(std::vector<int> &)) dlsym(commandLayer_handle,"GetAPIVersion");
//    MyGetSystemErrorCount = (int (*)(unsigned int &)) dlsym(commandLayer_handle,"GetSystemErrorCount");
//    MyGetSystemError = (int (*)(unsigned int, SystemError &)) dlsym(commandLayer_handle,"GetSystemError");

//    MyClearErrorLog = (int (*)()) dlsym(commandLayer_handle,"ClearErrorLog");

//    //If the was loaded correctly
//    if((MyInitAPI == NULL) || (MyCloseAPI == NULL) || (MyGetQuickStatus == NULL)
//            || (MyGetGripperStatus == NULL) || (MyGetGeneralInformations == NULL)
//            || (MyMoveHome == NULL) || (MySendJoystickCommand == NULL)
//            || (MyGetAngularPosition == NULL) || (MyGetAPIVersion == NULL))
//    {
//        cout << "Unable to initialize the command layer." << endl;
//    }
//    else
//    {
//        cout << "The command has been initialized correctly." << endl << endl;


//        std::vector<int> version;
//        result = (*MyGetAPIVersion)(version);
//        cout << "API's version is : " << version[0] << "." << version[1] << "." << version[2] << endl;



//        cout << "Calling the method InitAPI()" << endl;
//        result = (*MyInitAPI)();
//        cout << "result of InitAPI() = " << result << endl << endl;


//        //        QuickStatus status;
//        //        memset(&status, 0, sizeof(status));  // zero structure


////        result = (*MyStopControlAPI)();

////        usleep(5000000);

////        GeneralInformations info;
////        memset(&info, 0, sizeof(info));  // zero structure

//        //        printf("General information: ");
//        //        printRawBytes(info);

////        result = (*MyGetGeneralInformations)(info);

//        //        printf("General information: ");
//        //        printRawBytes(info);


////        cout << "Controller: " << info.Controller << endl;
//        cout << "We take control of the robotic arm." << endl;
////        result = (*MyStartControlAPI)();

//        unsigned int ErrorCount;

//        SystemError errorTemp;

//        result = (*MyGetSystemErrorCount)(ErrorCount);
//        cout << "System error count : " << ErrorCount << endl << endl << endl;

//        for(unsigned int i = 0; i < ErrorCount; i++)
//        {
//            result = (*MyGetSystemError)(i, errorTemp);
//            cout << "System error #" << i << " type : " << errorTemp.ErrorType << endl;
//        }

//        result = (*MyClearErrorLog)();
//       cout << "The error log has been deleted " << result << endl;



//        //        usleep(5000000);

//        //        result = (*MyGetGeneralInformations)(info);
//        //        cout << "Controller: " << info.Controller << endl;

//        //        cout << "Sending home\n";
//        //        //Sending the robot to the READY(HOME) position.
//        //        int home_data;
//        //        for(int i = 0; i < 400; i++)
//        //        {
//        //            result = (*MyMoveHome)(home_data);
//        //            cout << result << "      ";
//        //            result = (*MyGetSystemErrorCount)(ErrorCount);
//        //           cout << "System error count : " << ErrorCount << endl;
//        //            usleep(1000);
//        //        }
//        //        cout << "done home thing\n";

//        //        //We prepare the virtual joystick command that will be sent to the robotic arm.
//        //        JoystickCommand virtualCommand;
//        //        //Initializing the command.
//        //        for(int i = 0; i < JOYSTICK_BUTTON_COUNT; i++)
//        //        {
//        //            virtualCommand.ButtonValue[i] = 0;
//        //        }
//        ////        virtualCommand.ButtonValue[1] = 1;
//        //        virtualCommand.InclineForwardBackward = 0;
//        //        virtualCommand.InclineLeftRight = 0;
//        //        virtualCommand.MoveForwardBackward = 0;
//        //        virtualCommand.MoveLeftRight = 0;
//        //        virtualCommand.PushPull = 0;
//        //        virtualCommand.Rotate = 0;
//        //        virtualCommand.Rotate = 0;
//        //        int home_int = 0;
//        //        (*MyMoveHome)(home_int);
//        ////         usleep(10000);
//        //        (*MySendJoystickCommand)(virtualCommand);
//        //        virtualCommand.Rotate = 1;
//        //        cout << "Sending the command to the robotic arm." << endl;
//        //        //Every 1 ms we send a new virtual joystick command
//        //        for(int i = 0; i < 400; i++)
//        //        {
//        //            //(*MyMoveHome)(home_int);
//        //            (*MySendJoystickCommand)(virtualCommand);
//        //            usleep(1000);
//        //        }
//        //        //Waiting 3 seconds
//        //        cout << "The robotic arm will now move backward from where it comes." << endl;
//        //        //we send the command backward
//        //        virtualCommand.Rotate = -1;
//        //        cout << "Sending the command backward to the robotic arm." << endl;
//        //        //Every 1 ms we send a new virtual joystick command
//        //        for(int i = 0; i < 400; i++)
//        //        {
//        //            (*MySendJoystickCommand)(virtualCommand);
//        //            usleep(1000);
//        //        }
//        //        virtualCommand.Rotate = 0;
//        //        (*MySendJoystickCommand)(virtualCommand);



//        //        result = (*MyGetQuickStatus)(status);

//        //        printf("Quick status: ");
//        //        printRawBytes(status);

//        //        //Sending the robot to the READY(HOME) position.
//        //        cout << "Moving home\n";
//        //        int data;
//        //        result = (*MyMoveHome)(data);

//        cout << endl << "Calling the method CloseAPI()" << endl;
//        result = (*MyCloseAPI)();
//        cout << "result of CloseAPI() = " << result << endl;
//    }

//    return 0;
//}

