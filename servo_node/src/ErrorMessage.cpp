#include <servo_node/ErrorMessage.h>

using namespace Upenn;
using namespace std;

bool ErrorMessage::printInfo     = true;
bool ErrorMessage::printWarnings = true;
bool ErrorMessage::printErrors   = true;
bool ErrorMessage::initialized           = false;
bool ErrorMessage::initializedInfoLog    = false;
bool ErrorMessage::initializedWarningLog = false;
bool ErrorMessage::initializedErrorLog   = false;
bool ErrorMessage::logInfo     = false;
bool ErrorMessage::logWarnings = false;
bool ErrorMessage::logErrors   = false;
bool ErrorMessage::threadSafe  = true;

string ErrorMessage::logInfoFileName    = string();
string ErrorMessage::logWarningFileName = string();
string ErrorMessage::logErrorFileName   = string();

ofstream * ErrorMessage::logInfoStream    = NULL;
ofstream * ErrorMessage::logWarningStream = NULL;
ofstream * ErrorMessage::logErrorStream   = NULL;

pthread_mutex_t ErrorMessage::errorMutex = PTHREAD_MUTEX_INITIALIZER;
Timer ErrorMessage::timer0;

ErrorMessage::ErrorMessage()
{
}

ErrorMessage::~ErrorMessage()
{
}

int ErrorMessage::PrintInfo(string msg)
{
  LockErrorMutex();

  if (!initialized)
    Initialize();

  double t = timer0.GetAbsoluteTime();

  if (printInfo)
    cout<<"INFO("<<fixed<<t<<scientific<<"): "<<msg;

  if (logInfo)
  {
    if (!initializedInfoLog)
      InitializeInfoLog();
  
    if (logInfoStream)
    {
      *logInfoStream<<"INFO("<<fixed<<t<<scientific<<"): "<<msg;
      logInfoStream->flush();
    }
    else
    {
      printf("ErrorMessage::PrintInfo: ERROR: info log is enabled, initialized but the stream ptr is NULL\n");
      exit(1);
    }
  }
  
  UnlockErrorMutex();
  return 0;
}

int ErrorMessage::PrintWarning(string msg)
{
  LockErrorMutex();

  if (!initialized)
    Initialize();

  double t = timer0.GetAbsoluteTime();

  if (printWarnings)
    cout<<"WARNING("<<fixed<<t<<scientific<<"): "<<msg;

  if (logWarnings)
  {
    if (!initializedWarningLog)
      InitializeWarningLog();

    if (logWarningStream)
    {
      *logWarningStream<<"WARNING("<<fixed<<t<<scientific<<"): "<<msg;
      logWarningStream->flush();
    }
    else
    {
      printf("ErrorMessage::PrintWarning: ERROR: warning log is enabled, initialized but the stream ptr is NULL\n");
      exit(1);
    }
  }

  UnlockErrorMutex();
  return 0;
}

int ErrorMessage::PrintError(string msg)
{
  LockErrorMutex();

  if (!initialized)
    Initialize();

  double t = timer0.GetAbsoluteTime();

  if (printErrors)
    cout<<"ERROR("<<fixed<<t<<scientific<<"): "<<msg;

  if (logErrors)
  {
    if (!initializedErrorLog)
      InitializeErrorLog();

    if (logErrorStream)
    {
      *logErrorStream<<"ERROR("<<fixed<<t<<scientific<<"): "<<msg;
      logErrorStream->flush();
    }
    else
    {
      printf("ErrorMessage::PrintWarning: ERROR: warning log is enabled, initialized but the stream ptr is NULL\n");
      exit(1);
    }
  }

  UnlockErrorMutex();
  return 0;
}

int ErrorMessage::InitializeInfoLog()
{
  if (initializedInfoLog)
  {
    printf("ErrorMessage::InitializeInfoLog: ERROR: already initialized\n");
    exit(1);
  }
  
  if (logInfoFileName.empty())
    logInfoFileName = string("InfoLog.txt");

  if ( (logInfoFileName == logWarningFileName) && (initializedWarningLog) && (logWarningStream) )
    logInfoStream = logWarningStream;
  else if ( (logInfoFileName == logErrorFileName) && (initializedErrorLog) && (logErrorStream) )
    logInfoStream = logErrorStream;
  else
  {
    logInfoStream = new ofstream();
    if (!logInfoStream)
    {
      printf("ErrorMessage::InitializeInfoLog: ERROR: could not create instance of ofstream  for info logging\n");
      exit(1);
    }

    logInfoStream->open(logInfoFileName.c_str(), ios::out);
    if (logInfoStream->fail())
    {
      printf("ErrorMessage::InitializeInfoLog: ERROR: could not open the output info file for logging\n");
      exit(1);
    }
  }

  initializedInfoLog = true;
  return 0;
}

int ErrorMessage::InitializeWarningLog()
{
  if (initializedWarningLog)
    return 0;

  if (logWarningFileName.empty())
    logWarningFileName = string("WarningLog.txt");

  if ( (logWarningFileName == logInfoFileName) && (initializedInfoLog) && (logInfoStream) )
    logWarningStream = logInfoStream;
  else if ( (logWarningFileName == logErrorFileName) && (initializedErrorLog) && (logErrorStream) )
    logWarningStream = logErrorStream;
  else
  {
    logWarningStream = new ofstream();
    if (!logWarningStream)
    {
      printf("ErrorMessage::InitializeWarningLog: ERROR: could not create instance of ofstream  for warning logging\n");
      exit(1);
    }
    logWarningStream->open(logWarningFileName.c_str(), ios::out);
    if (logWarningStream->fail())
    {
      printf("ErrorMessage::InitializeWarningLog: ERROR: could not open the output warning file for logging\n");
      exit(1);
    }
  }

  initializedWarningLog = true;
  return 0;
}

int ErrorMessage::InitializeErrorLog()
{
  if (initializedErrorLog)
    return 0;

  if (logErrorFileName.empty())
    logErrorFileName = string("ErrorLog.txt");

  if ( (logErrorFileName == logInfoFileName) && (initializedInfoLog) && (logInfoStream) )
    logErrorStream = logInfoStream;
  else if ( (logErrorFileName == logWarningFileName) && (initializedWarningLog) && (logWarningStream) )
    logErrorStream = logWarningStream;
  else
  {
    logErrorStream = new ofstream();
    if (!logErrorStream)
    {
      printf("ErrorMessage::InitializeErrorLog: ERROR: could not create instance of ofstream  for error logging\n");
      exit(1);
    }
    logErrorStream->open(logErrorFileName.c_str(), ios::out);
    if (logErrorStream->fail())
    {
      printf("ErrorMessage::InitializeErrorLog: ERROR: could not open the output error file for logging\n");
      exit(1);
    }
  }

  initializedErrorLog = true;
  return 0;
}

int ErrorMessage::Initialize()
{
  if (initialized)
    return 0;

  initialized = true;
  
  return 0;
}

int ErrorMessage::LockErrorMutex()
{
  if (threadSafe)
  {
    pthread_mutex_lock( &errorMutex );
  }
  return 0;
}

int ErrorMessage::UnlockErrorMutex()
{
  if (threadSafe)
  {
    pthread_mutex_unlock( &errorMutex );
  }
  return 0;
}

int ErrorMessage::PrintInfoOn()
{
  LockErrorMutex();
  printInfo = true;
  UnlockErrorMutex();
  return 0;
}

int ErrorMessage::PrintInfoOff()
{
  LockErrorMutex();
  printInfo = false;
  UnlockErrorMutex();
  return 0;
}

int ErrorMessage::PrintWarningsOn()
{
  LockErrorMutex();
  printWarnings = true;
  UnlockErrorMutex();
  return 0;
}

int ErrorMessage::PrintWarningsOff()
{
  LockErrorMutex();
  printWarnings = false;
  UnlockErrorMutex();
  return 0;
}

int ErrorMessage::PrintErrorsOn()
{
  LockErrorMutex();
  printErrors = true;
  UnlockErrorMutex();
  return 0;
}

int ErrorMessage::PrintErrorsOff()
{
  LockErrorMutex();
  printErrors = false;
  UnlockErrorMutex();
  return 0;
}


int ErrorMessage::LogInfoOn()
{
  LockErrorMutex();
  logInfo = true;
  UnlockErrorMutex();
  return 0;
}

int ErrorMessage::LogInfoOff()
{
  LockErrorMutex();
  logInfo = false;
  UnlockErrorMutex();
  return 0;
}

int ErrorMessage::LogWarningsOn()
{
  LockErrorMutex();
  logWarnings = true;
  UnlockErrorMutex();
  return 0;
}

int ErrorMessage::LogWarningsOff()
{
  LockErrorMutex();
  logWarnings = false;
  UnlockErrorMutex();
  return 0;
}

int ErrorMessage::LogErrorsOn()
{
  LockErrorMutex();
  logErrors = true;
  UnlockErrorMutex();
  return 0;
}

int ErrorMessage::LogErrorsOff()
{
  LockErrorMutex();
  logErrors = false;
  UnlockErrorMutex();
  return 0;
}


