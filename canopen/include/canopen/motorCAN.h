#ifndef __MOTOR_CANL_H__
#define __MOTOR_CANL_H__



#define TPCANMsg _TPCANMsg
#include "canopen/pcan.h"
#undef TPCANMsg

#define LPSTR  char*
#include "canopen/PCANBasic.h"
#include "resource.h"



#define VERSION_MAJOR		2
#define VERSION_MINOR		0
#define VERSION_PATCH		4
#define VERSION_BUILD		6
#define STR_HELPER(x)		#x
#define STR(x) 				STR_HELPER(x)

////////////////////////////////////////////////////////////////////////////////////////////////////
// DEFINES

#define PROCFILE "/proc/pcan"                        // where to get information
#define MAX_LINE_LEN 255                             // to store a line of text
#define DEVICE_PATH "/dev/pcan"                      // + Minor = real device path
#define LOCAL_STRING_LEN 64                          // length of internal used strings

#define __ioctl(x, y, z) ioctl(x, y, z)
#define __close(fp)      close(fp)
//#define __close(fp) 


#define MINOR_FIRST_PCI 0                            // First minor for PCI
#define MINOR_FIRST_USB 32                           // First minor for USB
#define MINOR_FIRST_PCC 40                           // First minor for PCC
#define API_VERSION  STR(VERSION_MAJOR) "." STR(VERSION_MINOR) "." STR(VERSION_PATCH) "." STR(VERSION_BUILD) // Version
#define MAX_LOG 256                                  // Max length of a log string
#define LOG_FILE "PCANBasic.log"                     // LOG file name
////////////////////////////////////////////////////////////////////////////////////////////////////
// GLOBALS

typedef struct {
	char szDevicePath[LOCAL_STRING_LEN]; // Path of the device
	int nFileNo; // File number
	TPCANBaudrate Btr0Btr1; // Baud rate
	int nFilter; // Filter status
	int nListenOnly; // Listen mode
} PCAN_DESCRIPTOR;

extern PCAN_DESCRIPTOR** ppDescriptors;

extern char szLogLocationPath[LOCAL_STRING_LEN]; // LOG path
extern int nLogPathStatus ; // Path status (-1 not set; 0 set; 1 changed)
extern int nLogFileNo; // LOG file number
extern int nLogEnabled; // LOG enablement
extern int nLogFunction; // LOG configuration


extern TPCANStatus __stdcall CAN_Initialize(TPCANHandle Channel,
		TPCANBaudrate Btr0Btr1, TPCANType HwType, DWORD IOPort, WORD Interrupt);

extern TPCANStatus CAN_Uninitialize(TPCANHandle Channel) ;

extern TPCANStatus CAN_Reset(TPCANHandle Channel) ;

extern TPCANStatus CAN_GetStatus(TPCANHandle Channel);

extern TPCANStatus CAN_Read(TPCANHandle Channel, TPCANMsg* MessageBuffer,
		TPCANTimestamp* TimestampBuffer);


extern TPCANStatus CAN_Write(TPCANHandle Channel, TPCANMsg* MessageBuffer);

extern TPCANStatus CAN_FilterMessages(TPCANHandle Channel, DWORD FromID, DWORD ToID,
		TPCANMode Mode);

extern TPCANStatus CAN_GetValue(TPCANHandle Channel, TPCANParameter Parameter,
		void* Buffer, DWORD BufferLength) ;

extern TPCANStatus CAN_SetValue(TPCANHandle Channel, TPCANParameter Parameter,
		void* Buffer, DWORD BufferLength) ;

extern TPCANStatus CAN_GetErrorText(TPCANStatus Error, WORD Language, LPSTR Buffer);


#endif