/*
 * Kinova.DLL.CommLayerUbuntu.h
 *
 *  Created on: Oct 16, 2012
 *      Author: H. Lamontagne, Kinova
 */

#ifndef KINOVA_DLL_COMMLAYERUBUNTU_H_
#define KINOVA_DLL_COMMLAYERUBUNTU_H_

#ifdef KINOVADLLCOMMLAYER_EXPORTS
#define KINOVADLLCOMMLAYER_API __declspec(dllexport)
#else
#define KINOVADLLCOMMLAYER_API __declspec(dllimport)
#endif

#include <vector>

// ***** E R R O R   C O D E S ******

//No error, everything is fine.
#define NO_ERROR_KINOVA 1

//Unable to load the USB library.
#define ERROR_LOAD_USB_LIBRARY 1001

//Unable to access the Open method from the USB library.
#define ERROR_OPEN_METHOD  1002

//Unable to access the Write method from the USB library.
#define ERROR_WRITE_METHOD  1003

//Unable to access the Read method from the USB library.
#define ERROR_READ_METHOD  1004

//Unable to access the Read Int method from the USB library.
#define ERROR_READ_INT_METHOD  1005

//Unable to access the Free Library method from the USB library.
#define ERROR_FREE_LIBRARY  1006

//There is a problem with the USB connection between the device and the computer.
#define ERROR_JACO_CONNECTION 1007

//Unable to claim the USB interface.
#define ERROR_CLAIM_INTERFACE 1008

//Unknown type of device.
#define ERROR_UNKNOWN_DEVICE 1009

//The functionality you are trying to use has not been initialized.
#define ERROR_NOT_INITIALIZED 1010

//The USB library cannot find the device.
#define ERROR_LIBUSB_NO_DEVICE 1011

//The USB Library is bussy and could not perform the action.
#define ERROR_LIBUSB_BUSY 1012

//The functionality you are trying to perform is not supported by the version installed.
#define ERROR_LIBUSB_NOT_SUPPORTED 1013

//Unknown error while sending a packet.
#define ERROR_SENDPACKET_UNKNOWN 1014

//Cannot find the requested device.
#define ERROR_NO_DEVICE_FOUND 1015

// ***** E N D  O F  E R R O R   C O D E S ******


//Total size of our packet in bytes.
#define PACKET_SIZE 64

//Data's size of a single packet.
#define PACKET_DATA_SIZE 56

//Header's size of a packet.
#define PACKET_HEADER_SIZE 8

#define COMM_LAYER_VERSION 10001

//Max character count in our string.
#define SERIAL_LENGTH 20

//The maximum devices count that the API can control.
#define MAX_KINOVA_DEVICE 20

//That represents a packet. As a developper you don't have to use this structure
struct Packet
{
	short IdPacket;
	short TotalPacketCount;
	short IdCommand;
	short TotalDataSize;
	unsigned char Data[PACKET_DATA_SIZE];
};

//That is simply a list of packet
struct PacketList
{
	std::vector<Packet> packets;
};

//That is a device you can communicate with via this library.
struct KinovaDevice
{
	//The serial number of the device. If you are communicating with more than 1 device, this will be used to identify
	//the devices.
	char SerialNumber[SERIAL_LENGTH];

	//The model of the device.
	char Model[SERIAL_LENGTH];

	//Those variables represents the code version - Major.Minor.Release
	int VersionMajor;
	int VersionMinor;
	int VersionRelease;

	//The type of the device.
	int DeviceType;

	//This is a device ID used by the API. User should not use it.
	int DeviceID;
};

//Functions available outside of this library.
extern "C" __attribute__ ((visibility ("default"))) int InitCommunication(void);

extern "C" __attribute__ ((visibility ("default"))) int CloseCommunication(void);

extern "C" __attribute__ ((visibility ("default"))) int GetDeviceCount(int &result);

extern "C" __attribute__ ((visibility ("default"))) Packet SendPacket(Packet &packetOut, Packet &packetIn, int &result);

extern "C" __attribute__ ((visibility ("default"))) int ScanForNewDevice();

extern "C" __attribute__ ((visibility ("default"))) int GetDevices(KinovaDevice list[MAX_KINOVA_DEVICE], int &result);

extern "C" __attribute__ ((visibility ("default"))) int SetActiveDevice(KinovaDevice device);

extern "C" __attribute__ ((visibility ("default"))) int GetActiveDevice(KinovaDevice &device);

#endif /* KINOVA_DLL_COMMLAYERUBUNTU_H_ */
