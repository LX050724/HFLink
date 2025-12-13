/*********************************************************************
 *            (c) 1995 - 2018 SEGGER Microcontroller GmbH             *
 *                        The Embedded Experts                        *
 *                           www.segger.com                           *
 **********************************************************************
 */
#include <stdint.h>

typedef uint8_t U8;
typedef uint16_t U16;
typedef uint32_t U32;

typedef int8_t I8;
typedef int16_t I16;
typedef int32_t I32;

#define UNKNOWN 0  // Unknown
#define ONCHIP 1   // On-chip Flash Memory
#define EXT8BIT 2  // External Flash Device on 8-bit  Bus
#define EXT16BIT 3 // External Flash Device on 16-bit Bus
#define EXT32BIT 4 // External Flash Device on 32-bit Bus
#define EXTSPI 5   // External Flash Device on SPI

#define MAX_NUM_SECTORS (512) // Max. number of sectors, must not be modified.
#define ALGO_VERSION (0x0101) // Algo version, must not be modified.
#define SECTOR_END                                                                                                     \
    {                                                                                                                  \
        0xFFFFFFFF, 0xFFFFFFFF                                                                                         \
    }

struct SEGGER_OPEN_CMD_INFO; // Forward declaration of OFL lib private struct

struct SECTOR_INFO
{
    U32 SectorSize;      // Sector Size in bytes
    U32 SectorStartAddr; // Start address of the sector area (relative to the "BaseAddr" of the flash)
};

struct FlashDevice
{
    U16 AlgoVer;      // Algo version number
    U8 Name[128];     // Flash device name
    U16 Type;         // Flash device type
    U32 BaseAddr;     // Flash base address
    U32 TotalSize;    // Total flash device size in Bytes (256 KB)
    U32 PageSize;     // Page Size (number of bytes that will be passed to ProgramPage(). MinAlig is 8 byte
    U32 Reserved;     // Reserved, should be 0
    U8 ErasedVal;     // Flash erased value
    U32 TimeoutProg;  // Program page timeout in ms
    U32 TimeoutErase; // Erase sector timeout in ms
    struct SECTOR_INFO SectorInfo[MAX_NUM_SECTORS]; // Flash sector layout definition
};

typedef struct
{
    //
    // Optional functions may be be NULL
    //
    void (*pfFeedWatchdog)(void);                                          // Optional
    int (*pfInit)(U32 Addr, U32 Freq, U32 Func);                           // Mandatory
    int (*pfUnInit)(U32 Func);                                             // Mandatory
    int (*pfEraseSector)(U32 Addr);                                        // Mandatory
    int (*pfProgramPage)(U32 Addr, U32 NumBytes, U8 *pSrcBuff);            // Mandatory
    int (*pfBlankCheck)(U32 Addr, U32 NumBytes, U8 BlankData);             // Optional
    int (*pfEraseChip)(void);                                              // Optional
    U32 (*pfVerify)(U32 Addr, U32 NumBytes, U8 *pSrcBuff);                 // Optional
    U32 (*pfSEGGERCalcCRC)(U32 CRC, U32 Addr, U32 NumBytes, U32 Polynom);  // Optional
    int (*pfSEGGERRead)(U32 Addr, U32 NumBytes, U8 *pDestBuff);            // Optional
    int (*pfSEGGERProgram)(U32 DestAddr, U32 NumBytes, U8 *pSrcBuff);      // Optional
    int (*pfSEGGERErase)(U32 SectorAddr, U32 SectorIndex, U32 NumSectors); // Optional
    void (*pfSEGGERStart)(volatile struct SEGGER_OPEN_CMD_INFO *pInfo);    // Optional
} SEGGER_OFL_API;

//
// Flash module functions
//
extern int Init(U32 Addr, U32 Freq, U32 Func);
extern int UnInit(U32 Func);
extern int BlankCheck(U32 Addr, U32 NumBytes, U8 BlankData);
extern int EraseChip(void);
extern int EraseSector(U32 Addr);
extern int ProgramPage(U32 Addr, U32 NumBytes, U8 *pSrcBuff);
extern U32 Verify(U32 Addr, U32 NumBytes, U8 *pSrcBuff);

//
// SEGGER extensions
//
U32 SEGGER_OPEN_CalcCRC(U32 CRC, U32 Addr, U32 NumBytes, U32 Polynom);  // Optional
int SEGGER_OPEN_Read(U32 Addr, U32 NumBytes, U8 *pDestBuff);            // Optional
int SEGGER_OPEN_Program(U32 DestAddr, U32 NumBytes, U8 *pSrcBuff);      // Optional
int SEGGER_OPEN_Erase(U32 SectorAddr, U32 SectorIndex, U32 NumSectors); // Optional
void SEGGER_OPEN_Start(volatile struct SEGGER_OPEN_CMD_INFO *pInfo);    // Optional

//
// SEGGER OFL lib helper functions that may be called by specific algo part
//
U32 SEGGER_OFL_Lib_CalcCRC(const SEGGER_OFL_API *pAPI, U32 CRC, U32 Addr, U32 NumBytes, U32 Polynom);
void SEGGER_OFL_Lib_StartTurbo(const SEGGER_OFL_API *pAPI, volatile struct SEGGER_OPEN_CMD_INFO *pInfo);
