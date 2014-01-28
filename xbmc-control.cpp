// Copyright (C) 2013 Thalmic Labs Inc.
// Confidential and not for redistribution. See LICENSE.txt.
#define _USE_MATH_DEFINES
#include <cmath>
#include <iostream>
#include <iomanip>
#include <stdexcept>
#include <string>
#include <vector>
#include <WinSock2.h>
#include <WS2tcpip.h>
#include <stdio.h>
#include <string.h>
#include <TCHAR.H>

// The only file that needs to be included to use the Myo C++ SDK is myo.hpp.
#include <myo.hpp>

#define UPDATES_PER_SECOND 30

#pragma comment(lib, "Ws2_32.lib")
#define DEFAULT_BUFLEN 512

bool ConnectToXBMC(int PortNo, char* IPAddress);
int sendToXBMC(std::string command);
int ProcessXBMCReply();
void showXBMCNotification(std::string title, std::string message);
void xbmcPlayPause();
void xbmcNext();
void xbmcPrev();


WSADATA wsaData;
SOCKET xbmcSocket; //Socket handle

enum XBMCState
{
	PLAYBACK,
	MENU,
	UNKNOWN
};

XBMCState state = UNKNOWN;

const myo::Pose actionPose = myo::Pose::fist;
const myo::Pose originPose = myo::Pose::fingers_spread;
//const myo::Pose movementPose = myo::Pose::none;

//Interesting: the Myo MAC addresses are all reversed to what you would normally read. IE:
// A MAC that appears as DE:AD:BE:EF would be written here as efbeadde
#define ALLDRITT_MYO "f1601c7dffc6" 
#define NAUSE_MYO "96607cde75ee"

const std::string selectString = "{\"jsonrpc\":\"2.0\",\"method\":\"Input.Select\",\"id\":\"NULL\"}";
const std::string leftString = "{\"jsonrpc\":\"2.0\",\"method\":\"Input.Left\",\"id\":\"NULL\"}";
const std::string upString = "{\"jsonrpc\":\"2.0\",\"method\":\"Input.Up\",\"id\":\"NULL\"}";
const std::string downString = "{\"jsonrpc\":\"2.0\",\"method\":\"Input.Down\",\"id\":\"NULL\"}";
const std::string rightString = "{\"jsonrpc\":\"2.0\",\"method\":\"Input.Right\",\"id\":\"NULL\"}";
const std::string backString = "{\"jsonrpc\":\"2.0\",\"method\":\"Input.Back\",\"id\":\"NULL\"}";
const std::string getCurrentWindow = "{\"jsonrpc\":\"2.0\",\"method\":\"XBMC.GetInfoLabels\",\"params\":{\"labels\":[\"System.CurrentWindow\"]},\"id\":\"NULL\"}";
const std::string volumeUp = "{\"jsonrpc\":\"2.0\",\"method\":\"Application.SetVolume\",\"params\":{\"volume\":\"increment\"},\"id\":\"NULL\"}";
const std::string volumeDown = "{\"jsonrpc\":\"2.0\",\"method\":\"Application.SetVolume\",\"params\":{\"volume\":\"decrement\"},\"id\":\"NULL\"}";


// Classes that inherit from myo::DeviceListener can be used to receive events from Myo devices. DeviceListener
// provides several virtual functions for handling different kinds of events. If you do not override an event, the
// default behavior is to do nothing.
class DataCollector : public myo::DeviceListener {
public:
	DataCollector()
		: roll_w(0), pitch_w(0), yaw_w(0), currentPose(), active(false), settingOrigin(false), mouseIsDown(false)
	{
		resX = ::GetSystemMetrics(SM_CXSCREEN);
		resY = ::GetSystemMetrics(SM_CYSCREEN);

		dx = 0.0;
		dy = 0.0;
		dz = 0.0;

		x = 0.0;
		y = 0.0;
		z = 0.0;

		world_x = 0.0;
		world_y = 0.0;
		world_z = 0.0;

		start_roll = 0.0;
		start_pitch = 0.0;
		start_yaw = 0.0;
		
		previousPose = myo::Pose::none;
		currentPose = myo::Pose::none;

		rollVertical = -1;
		rollHorizontal = -1;

		loopCount = 0;
		loopsPerEvent = UPDATES_PER_SECOND / 2;
		longGestureInProgress = false;
	}
	// onOrientationData() is called whenever the Myo device provides its current orientation, which is represented
	// as a quaternion.
	void onOrientationData(myo::Myo* myo, uint64_t timestamp, const myo::Quaternion<float>& quat)
	{
		using std::atan2f;
		using std::asinf;
		using std::sqrtf;

		// Calculate the normalized quaternion.
		float norm = sqrtf(quat.x() * quat.x() + quat.y() * quat.y() + quat.z() * quat.z() + quat.w() * quat.w());
		myo::Quaternion<float> normalized(quat.x()/norm, quat.y()/norm, quat.z()/norm, quat.w()/norm);

		// Calculate Euler angles (roll, pitch, and yaw) from the normalized quaternion.
		roll = atan2f(2.0f * (normalized.w() * normalized.x() + normalized.y() * normalized.z()),
			1.0f - 2.0f * (normalized.x() * normalized.x() + normalized.y() * normalized.y()));
		pitch = asinf(2.0f * (normalized.w() * normalized.y() - normalized.z() * normalized.x()));
		yaw = atan2f(2.0f * (normalized.w() * normalized.z() + normalized.x() * normalized.y()),
			1.0f - 2.0f * (normalized.y() * normalized.y() + normalized.z() * normalized.z()));

		// Convert the floating point angles in radians to a scale from 0 to 20.
		roll_w = static_cast<int>((roll + (float)M_PI)/(M_PI * 2.0f) * 18);
		pitch_w = static_cast<int>((pitch + (float)M_PI/2.0f)/M_PI * 18);
		yaw_w = static_cast<int>((yaw + (float)M_PI)/(M_PI * 2.0f) * 18);
	}

	void onAccelerometerData(myo::Myo *myo, uint64_t timestamp, const myo::Vector3< float > &accel)
	{
		dx = x - accel[0];
		dy = y - accel[1];
		dz = z - accel[2];

		x = accel[0];
		y = accel[1];
		z = accel[2];

		world_x = (x * cosf(roll)) + (-1 * z * cosf(1.57079633 - roll));
	}

	//Call this as often as possible to run constant-use commands while the event loop is going
	//Stuff like volume control which needs both a pose and rotation data
	int loopCount;
	int loopsPerEvent;
	bool waitedBeforeStart;
	bool longGestureInProgress;

	void ProcessEvent()
	{
		if(currentPose == myo::Pose::none) //No repeat actions are allowed to occur on a "none" pose
		{
			waitedBeforeStart = false;
			loopCount = 0;
			return;
		}
		
		if(loopCount + 10 < loopsPerEvent)
		{
			loopCount++;
			return;
		}

		//Wait one cycle before starting the repeated events
		if(waitedBeforeStart == false)
		{
			waitedBeforeStart = true;
			loopCount = 0;
			return;
		}

		//Perform event handling here
		if(currentPose == myo::Pose::fingers_spread)
		{
			//Send data to XBMC
			sendToXBMC(backString);
		}
		else if(currentPose == myo::Pose::wave_in)
		{
			//Send data to XBMC
			if(abs(roll - rollVertical) > abs(roll - rollHorizontal)) //Closer to the horizontal config point
			{
				printf("LEFT\n");
				sendToXBMC(leftString);
			}
			else
			{
				printf("DOWN\n");
				sendToXBMC(downString);
			}
		}
		else if ( currentPose == myo::Pose::wave_out )
		{
			//Send data to XBMC
			if(abs(roll - rollVertical) > abs(roll - rollHorizontal)) //Closer to the horizontal config point
			{
				printf("RIGHT\n");
				sendToXBMC(rightString);
			}
			else
			{
				printf("UP\n");
				sendToXBMC(upString);
			}
		}
		else if ( currentPose == myo::Pose::fist && state == PLAYBACK)
		{
			//Volume control event
			if(start_roll - roll < -0.3)
			{
				std::cout << "VOLUME DOWN" << std::endl;
				sendToXBMC(volumeDown);
			}
			else if(start_roll - roll > 0.3)
			{
				std::cout << "VOLUME UP" << std::endl;
				sendToXBMC(volumeUp);
			}

			std::cout << start_roll - roll << std::endl;
		}

		longGestureInProgress = true;
		loopCount = 0;
	}

	// onPose() is called whenever the Myo detects that the person wearing it has changed their pose, for example,
	// making a fist, or not making a fist anymore.
	void onPose(myo::Myo* myo, uint64_t timestamp, myo::Pose pose)
	{
		previousPose = currentPose;
		currentPose = pose;

		//Calibration mode!
		if(rollHorizontal == -1 || rollVertical == -1)
		{
			if(currentPose == myo::Pose::fist && previousPose == myo::Pose::none)
			{
				rollHorizontal = roll;
				printf("Horizontal Point Registered %f!\n", rollHorizontal);
			}
			else if(currentPose == myo::Pose::none && previousPose == myo::Pose::fist)
			{
				rollVertical = roll;
				printf("Vertical Point Registered %f!\n", rollVertical);
			}

			return;
		}

		if(pose == myo::Pose::fingers_spread)
		{
			//Send data to XBMC
			sendToXBMC(backString);
		}
		else if (pose == myo::Pose::fist) 
		{ 
			//Send data to XBMC
			if(state == MENU)
				sendToXBMC(selectString);

			//Register the starting position for the roll axis in case the user holds down their fist to initiate a volume change gesture
			else if(state == PLAYBACK)
				start_roll = roll;
		}
		else if ( pose == myo::Pose::wave_in )
		{
			if(state == MENU)
			{
				//Send data to XBMC
				if(abs(roll - rollVertical) > abs(roll - rollHorizontal)) //Closer to the horizontal config point
				{
					printf("LEFT\n");
					sendToXBMC(leftString);
				}
				else
				{
					printf("DOWN\n");
					sendToXBMC(downString);
				}
			}
			else if(state == PLAYBACK)
			{
				std::cout << "Sending Prev" << std::endl;
				xbmcPrev();
			}
		}
		else if ( pose == myo::Pose::wave_out )
		{
			if(state == MENU)
			{
				//Send data to XBMC
				if(abs(roll - rollVertical) > abs(roll - rollHorizontal)) //Closer to the horizontal config point
				{
					printf("RIGHT\n");
					sendToXBMC(rightString);
				}
				else
				{
					printf("UP\n");
					sendToXBMC(upString);
				}
			}
			else if(state == PLAYBACK)
			{
				std::cout << "Sending Next" << std::endl;
				xbmcNext();
			}
		}
		else if ( pose == myo::Pose::none )
		{
			if(previousPose == myo::Pose::fist && state == PLAYBACK && !longGestureInProgress)
			{
				std::cout << "PLAY/PAUSE" << std::endl;
				xbmcPlayPause();
			}

			//We are at a "none" pose, a gesture can no longer be in progress
			if(longGestureInProgress)
				longGestureInProgress = false;
		}
	}

	int roll_w, pitch_w, yaw_w;
	float start_roll, start_pitch, start_yaw;
	float roll, yaw, pitch;
	float x, y, z,  dx, dy, dz, world_x, world_y, world_z;
	myo::Pose currentPose;
	myo::Pose previousPose;
	long resX, resY;
	bool active;
	bool settingOrigin;
	bool mouseIsDown;

	float rollVertical;
	float rollHorizontal;
};

int main(int argc, char** argv)
{
	// We catch any exceptions that might occur below -- see the catch statement for more details.
	try {
		// First, we create a Hub. The Hub provides access to one or more Myos.
		myo::Hub hub;

		std::cout << "Attempting to find a Myo..." << std::endl;

		// Next, we try to find a Myo (any Myo) that's nearby and connect to it. waitForAnyMyo() takes a timeout
		// value in milliseconds. In this case we will try to find a Myo for 10 seconds, and if that fails, the function
		// will return a null pointer.
		myo::Myo* myo = hub.waitForAnyMyo(10000);

		// If waitForAnyMyo() returned a null pointer, we failed to find a Myo, so exit with an error message.
		if (!myo) {
			throw std::runtime_error("Unable to find a Myo!");
		}

		// We've found a Myo, let's output its MAC address.
		std::cout << "Connected to " << std::hex << std::setfill('0') << std::setw(12) << myo->macAddress() << std::dec << "." << std::endl << std::endl;

		DataCollector collector;
		hub.addListener(&collector);

		std::string xbmcIpAddr = "127.0.0.1";

		std::cout << "Please enter the IP address of the computer running XBMC (default is the local machine): ";
		getline(std::cin, xbmcIpAddr);

		if(!ConnectToXBMC(9090, xbmcIpAddr == "" ? "127.0.0.1" : xbmcIpAddr.c_str()))
		{
			std::cout << "Error: could not connect to XBMC. Shutting down." << std::endl;
			system("pause");
			return -1;
		}

		int loopCounter = 0;
		const int loopsPerEvent = 30;

		// Finally we enter our main loop.
		while (1) 
		{
			hub.run(1000/UPDATES_PER_SECOND);
			collector.ProcessEvent();
			
			if(loopCounter++ >= (UPDATES_PER_SECOND / 2))
				sendToXBMC(getCurrentWindow);

			ProcessXBMCReply();
		}
	} 
	catch (const std::exception& e) 
	{
		std::cerr << "Error: " << e.what() << std::endl;
		std::cerr << "Press enter to continue.";
		std::cin.ignore();
		return 1;
	}

	return 0;
}

bool ConnectToXBMC(int PortNo, char* IPAddress)
{
	int iResult;
	// Initialize Winsock
	iResult = WSAStartup(MAKEWORD(2,2), &wsaData);
	if (iResult != 0) 
	{
		printf("WSAStartup failed: %d\n", iResult);
		abort();
	}

	std::cout << "Winsock initialized" << std::endl;

    //Fill out the information needed to initialize a socket…
    SOCKADDR_IN target; //Socket address information

    target.sin_family = AF_INET; // address family Internet
    target.sin_port = htons (PortNo); //Port to connect on
    target.sin_addr.s_addr = inet_addr (IPAddress); //Target IP

    xbmcSocket = socket (AF_INET, SOCK_STREAM, IPPROTO_TCP); //Create socket
    if (xbmcSocket == INVALID_SOCKET)
    {
        return false; //Couldn't create the socket
    }  

    //Try connecting...

    if (connect(xbmcSocket, (SOCKADDR *)&target, sizeof(target)) == SOCKET_ERROR)
    {
        return false; //Couldn't connect
    }
    else
	{
		//Set the socket to non-blocking mode
		u_long iMode=1;
		ioctlsocket(xbmcSocket,FIONBIO,&iMode);

		//Send an initialization packet to XBMC to let it know we're here
		showXBMCNotification("Myo Remote", "A Myo has been connected as a remote");
		return true; //Success
	}
}

void CloseConnection (SOCKET s)
{
    //Close the socket if it exists
    if (s)
        closesocket(s);
}

int ProcessXBMCReply()
{
	char recvbuf[DEFAULT_BUFLEN];
	int iResult, iSendResult;
	int recvbuflen = DEFAULT_BUFLEN;
	iResult = recv(xbmcSocket, recvbuf, recvbuflen, 0);
	
    if (iResult > 0)
	{
		char* recvStr = new char[iResult+1];
		char* testStringA = "System.CurrentWindow";
		strncpy(recvStr, recvbuf, iResult);
		recvStr[iResult] = '\0';

		std::string stdRecv(recvStr);

		if(stdRecv.find("System.CurrentWindow") != std::string::npos)
		{
			if(stdRecv.find("Audio visualization") != std::string::npos)
			{
				state = PLAYBACK;
			}
			else if(stdRecv.find("Fullscreen video") != std::string::npos)
			{
				state = PLAYBACK;
			}
			else
			{
				state = MENU;
			}
		}

		delete recvStr;
    }
	else if (iResult == 0)
	{
        std::cout << "Connection closing..." << std::endl;
		return -2;
	}
    else
	{
        return -1;
    }

	return 0;
}

int sendToXBMC(std::string command)
{
	int iSendResult = send(xbmcSocket, command.c_str(), strlen(command.c_str()), 0);
	if (iSendResult == SOCKET_ERROR)
	{
		printf("xbmc send failed: %d\n", WSAGetLastError());
	}

	return iSendResult;
}

void showXBMCNotification(std::string title, std::string message)
{
	std::string command = "{\"jsonrpc\":\"2.0\",\"method\":\"GUI.ShowNotification\",\"id\":\"NULL\",\"params\":{\"title\":\"" + title + "\",\"message\":\"" + message + "\"}}";
	sendToXBMC(command);
}

void xbmcPlayPause()
{
	std::string command = "{\"jsonrpc\":\"2.0\",\"method\":\"Player.PlayPause\",\"params\":{\"playerid\":0},\"id\":\"NULL\"}";
	sendToXBMC(command);

	//Send this to deal with the difference between video players and music players
	command = "{\"jsonrpc\":\"2.0\",\"method\":\"Player.PlayPause\",\"params\":{\"playerid\":1},\"id\":\"NULL\"}";
	sendToXBMC(command);
}

void xbmcNext()
{
	std::string command = "{\"jsonrpc\":\"2.0\",\"method\":\"Player.GoTo\",\"params\":{\"playerid\":0,\"to\":\"next\"},\"id\":\"NULL\"}";
	sendToXBMC(command);

	//Send this to deal with the difference between video players and music players
	command = "{\"jsonrpc\":\"2.0\",\"method\":\"Player.GoTo\",\"params\":{\"playerid\":1,\"to\":\"next\"},\"id\":\"NULL\"}";
	sendToXBMC(command);
}

void xbmcPrev()
{
	std::string command = "{\"jsonrpc\":\"2.0\",\"method\":\"Player.GoTo\",\"params\":{\"playerid\":0,\"to\":\"previous\"},\"id\":\"NULL\"}";
	sendToXBMC(command);

	//Send this to deal with the difference between video players and music players
	command = "{\"jsonrpc\":\"2.0\",\"method\":\"Player.GoTo\",\"params\":{\"playerid\":1,\"to\":\"previous\"},\"id\":\"NULL\"}";
	sendToXBMC(command);
}