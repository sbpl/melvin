#ifndef ERRORMESSAGE_HH
#define ERRORMESSAGE_HH

#include <stdlib.h>
#include <string.h>
#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
#include <pthread.h>
#include "Timer.h"

#define FANCY_ERROR_MESSAGE 

#ifdef FANCY_ERROR_MESSAGE

#define PRINT_INFO( msg )    { std::ostringstream msgStream; msgStream <<"["<<__PRETTY_FUNCTION__<<" : "<<__LINE__<<"] : "<< msg << std::flush; Upenn::ErrorMessage::PrintInfo(msgStream.str()); }
#define PRINT_INFO_RAW( msg )    { std::ostringstream msgStream; msgStream <<"["<<__PRETTY_FUNCTION__<<" : "<<__LINE__<<"] : "<< msg << std::flush; Upenn::ErrorMessage::PrintInfo(msgStream.str()); }
#define PRINT_WARNING( msg ) { std::ostringstream msgStream; msgStream <<"["<<__PRETTY_FUNCTION__<<" : "<<__LINE__<<"] : "<< msg << std::flush; Upenn::ErrorMessage::PrintWarning(msgStream.str()); }
#define PRINT_ERROR( msg )   { std::ostringstream msgStream; msgStream <<"["<<__PRETTY_FUNCTION__<<" : "<<__LINE__<<"] : "<< msg << std::flush; Upenn::ErrorMessage::PrintError(msgStream.str()); }

/*
#define PRINT_INFO( msg )    { std::ostringstream msgStream; msgStream <<"["<<__FUNCTION__<<" : "<<__LINE__<<"] : "<< msg << std::flush; Upenn::ErrorMessage::PrintInfo(msgStream.str()); }
#define PRINT_INFO_RAW( msg )    { std::ostringstream msgStream; msgStream <<"["<<__FUNCTION__<<" : "<<__LINE__<<"] : "<< msg << std::flush; Upenn::ErrorMessage::PrintInfo(msgStream.str()); }
#define PRINT_WARNING( msg ) { std::ostringstream msgStream; msgStream <<"["<<__FUNCTION__<<" : "<<__LINE__<<"] : "<< msg << std::flush; Upenn::ErrorMessage::PrintWarning(msgStream.str()); }
#define PRINT_ERROR( msg )   { std::ostringstream msgStream; msgStream <<"["<<__FUNCTION__<<" : "<<__LINE__<<"] : "<< msg << std::flush; Upenn::ErrorMessage::PrintError(msgStream.str()); }
*/

#define LOG_MESSAGE( msg ) { }

#define PRINT_INFO_ON Upenn::ErrorMessage::PrintInfoOn()
#define PRINT_INFO_OFF Upenn::ErrorMessage::PrintInfoOff()
#define PRINT_WARNINGS_ON Upenn::ErrorMessage::PrintWarningsOn()
#define PRINT_WARNINGS_OFF Upenn::ErrorMessage::PrintWarningsOff()
#define PRINT_ERRORS_ON Upenn::ErrorMessage::PrintErrorsOn()
#define PRINT_ERRORS_OFF Upenn::ErrorMessage::PrintErrorsOff()

#define LOG_INFO_ON Upenn::ErrorMessage::LogInfoOn()
#define LOG_INFO_OFF Upenn::ErrorMessage::LogInfoOff()
#define LOG_WARNINGS_ON Upenn::ErrorMessage::LogWarningsOn()
#define LOG_WARNINGS_OFF Upenn::ErrorMessage::LogWarningsOff()
#define LOG_ERRORS_ON Upenn::ErrorMessage::LogErrorsOn()
#define LOG_ERRORS_OFF Upenn::ErrorMessage::LogErrosOff()

#else

#define PRINT_INFO( msg ) { std::cout << msg <<std::endl; }
#define PRINT_INFO_RAW( msg ) { std::cout << msg <<std::endl; }
#define PRINT_WARNING( msg ) { std::cout << msg <<std::endl; }
#define PRINT_ERROR( msg ) { std::cout << msg <<std::endl; }

#define PRINT_INFO_ON
#define PRINT_INFO_OFF
#define PRINT_WARNINGS_ON
#define PRINT_WARNINGS_OFF
#define PRINT_ERRORS_ON
#define PRINT_ERRORS_OFF

#define LOG_INFO_ON
#define LOG_INFO_OFF
#define LOG_WARNINGS_ON
#define LOG_WARNINGS_OFF
#define LOG_ERRORS_ON
#define LOG_ERRORS_OFF

#endif

using namespace std;

namespace Upenn 
{
  class ErrorMessage
  {
    //constructor
    public: ErrorMessage();
    
    //destructor
    public: ~ErrorMessage();

    //print out info message
    public: static int PrintInfo(string msg);
    
    //print out warning message
    public: static int PrintWarning(string msg);
    
    //print out error message
    public: static int PrintError(string msg);

    public: static int PrintInfoOn();
    public: static int PrintInfoOff();
    public: static int PrintWarningsOn();
    public: static int PrintWarningsOff();
    public: static int PrintErrorsOn();
    public: static int PrintErrorsOff();

    public: static int LogInfoOn();
    public: static int LogInfoOff();
    public: static int LogWarningsOn();
    public: static int LogWarningsOff();
    public: static int LogErrorsOn();
    public: static int LogErrorsOff();
    
    //initialize
    private: static int Initialize();

    private: static int LockErrorMutex();
    private: static int UnlockErrorMutex();

    private: static int InitializeInfoLog();
    private: static int InitializeWarningLog();
    private: static int InitializeErrorLog();

    private: static bool logInfo;
    private: static bool logWarnings;
    private: static bool logErrors;
    private: static bool printInfo;
    private: static bool printWarnings;
    private: static bool printErrors;

    private: static bool initialized;
    private: static bool initializedInfoLog;
    private: static bool initializedWarningLog;
    private: static bool initializedErrorLog;
    private: static string logInfoFileName;
    private: static string logWarningFileName;
    private: static string logErrorFileName;

    private: static ofstream * logInfoStream;
    private: static ofstream * logWarningStream;
    private: static ofstream * logErrorStream;
    private: static bool threadSafe;
    private: static pthread_mutex_t errorMutex;
    private: static Upenn::Timer timer0;
    
  };
}



#endif //ERRORMESSAGE_HH 
