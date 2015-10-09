//Driver for Dynamixel servo modules
//
//Aleksandr Kushleyev
//University of Pennsylvania
//June 2009 
//akushley@seas.upenn.edu

#ifndef DYNAMIXEL_HH
#define DYNAMIXEL_HH

#include "iostream"
#include <string>
#include <list>
#include "Timer.h"
#include <vector>
#include "SerialDevice.h"
#include <iomanip>
#include <string>
#include <stdint.h>

#define DYNAMIXEL_DEFAULT_MODULE_ID         0x00
#define DYNAMIXEL_PACKET_HEADER_LENGTH      4
#define DYNAMIXEL_DEFAULT_TIMEOUT           100000
#define DYNAMIXEL_MAX_NUM_OTHER_MESSAGES    20

#define DYNAMIXEL_RESPONSE_NO_ERROR         0x00

#define DYNAMIXEL_READ_DATA_INSTRUCTION     0x02
#define DYNAMIXEL_WRITE_DATA_INSTRUCTION    0x03

#define DYNAMIXEL_MODEL_NUMBER_ADDRESS      0x00
#define DYNAMIXEL_PRESENT_POSITION_ADDRESS  0x24
#define DYNAMIXEL_GOAL_POSITION_ADDRESS     0x1E

#define DYNAMIXEL_AX12_MAX_RPM              114

#define DYNAMIXEL_DATA_LENGTH_BYTE_POS      2
#define DYNAMIXEL_CMD_BYTE_POS              3
#define DYNAMIXEL_PARAM_BYTE_POS            4


#define DYNAMIXEL_MIN_ANGLE                -150
#define DYNAMIXEL_MAX_ANGLE                 150

#define DYNAMIXEL_UNKNOWN_PARAMS           -100

#define DYNAMIXEL_STOP_UNIT_RETRIES        10
#define DYNAMIXEL_STARTUP_RETRIES          5
#define DYNAMIXEL_DEF_BUFFER_LENGTH        1024
#define DYNAMIXEL_DEFAULT_VELOCITY         100
#define DYNAMIXEL_DEFAULT_ACCELERATION     300
#define DYNAMIXEL_DEFAULT_MIN_ANGLE        (-7.5-30)
#define DYNAMIXEL_DEFAULT_MAX_ANGLE        (-7.5+30)
#define DYNAMIXEL_DEFAULT_REVERSE_POINT    5

using namespace Upenn;
using namespace std;

namespace Upenn
{
  struct ErrorInfo
  {
    unsigned char cmd;
    unsigned char code;
    double t;  
  };


  enum { DYNAMIXEL_STATUS_OK,
         DYNAMIXEL_STATUS_WARNING,
         DYNAMIXEL_STATUS_ERROR,
         DYNAMIXEL_STATUS_UNKNOWN};

  //error codes that may be stored into this->lastDriverError.
  enum { DYNAMIXEL_ERROR_OK,
         DYNAMIXEL_ERROR_TIMEOUT,
         DYNAMIXEL_ERROR_BAD_CHECKSUM,
         DYNAMIXEL_ERROR_NOT_ENOUGH_SPACE,
         DYNAMIXEL_ERROR_INCOMPLETE_PACKET};

  //Class ShunkDriver implements functions for the SerialDeviceReader class
  //It will read packets of specified length and maintain a circular buffer of desired length
  class Dynamixel
  {	
    //constructor 
    public: Dynamixel();

    //destructor
    public: ~Dynamixel();

    //connect to the device
    public: int Connect(string device, int baudRate, int moduleId);

    //disconnect from the device
    public: int Disconnect();
    
    //start up the device
    public: int StartDevice();
	
    //shutdown the device
    public: int StopDevice();

    //get the status
    public: int GetStatus();

    //calculate the checksum
    private: unsigned char CalcCheckSum(unsigned char * buf, bool writeToBuf = true);

    //get the last stored error code that came from device
    private: unsigned char GetLastDeviceError();

    //get the last stored error code, set by this driver
    private: unsigned char GetLastDriverError();

    //send the stop command
    private: int SendStopCmd();

    //convert angle in degrees to position command
    private: int AngleDeg2AngleVal(float angle, uint16_t &val);

    //convert the position feedback to degrees
    private: int AngleVal2AngleDeg(uint16_t val, float &angle);

    //convert velocity in degrees per second to velocity command
    private: int VelocityDeg2VelocityVal(float velocity, uint16_t &val);

    //convert velocity feedback to degrees/second
    private: int VelocityVal2VelocityDeg(uint16_t val, float &velocity);

    //stop the unit
    private: int StopUnit();

    //send a command to request motion to a position using provided velocity and stored acceleration
    public: int MoveToPos(float position, float velocity);

    //request position feedback
    public: int GetPosition(float & position);

    //get command confirmation
    private: int GetCmdConfirmation(uint16_t &status);
    
    //get the device information (model number)
    private: int GetDeviceInfo(string & info);


    //handle an unexpected message
    private: int HandleOtherMessage(char * packet, unsigned int packetLength);
    
    //print the packet in 
    private: void PrintPacket(char * buf, int length);
    
    //read a packet and verify the checksum
    private: int ReadPacket(char * packet, unsigned int maxLength, 
                          unsigned int timeoutUs = DYNAMIXEL_DEFAULT_TIMEOUT);

    private: int WritePacket(char * packet, unsigned int length);

    //pack up the data for shipping out to the device (adds header, length, checksum)
    private: int CreateOutgoingPacket(void * payload, unsigned int length, 
                                      char * packet, unsigned int maxLength);

    
    private: SerialDevice * sd;
    private: bool connected;
    private: unsigned char moduleId;          //module id - used to construct the packet
    private: list<ErrorInfo> errors;          //list of errors
    private: Timer timer0;                    //just a timer
    private: unsigned char lastDeviceError;   //last error reported by the device
    private: unsigned char lastDriverError;   //last error reported by the driver
    private: static const float DYNAMIXEL_AX12_MAX_VEL = (DYNAMIXEL_AX12_MAX_RPM/60.0*360.0);
  };
}

#endif //DYNAMIXEL_HH
