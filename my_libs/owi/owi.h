/*
 1-Wire code based on Atmel appnote
 AVR318: Dallas 1-Wire master
 */

#ifndef OWI_H_
#define OWI_H_

#include <stdint.h>
#include <inttypes.h>
#include <libopencm3/cm3/cortex.h>

typedef enum {	OWI_RESULT_OK, OWI_RESULT_ERROR } OWI_Result;
typedef enum {	OWI_PINSTATE_LOW, OWI_PINSTATE_HIGH } OWI_PinState;

/* Hardware-independent pin control interface.
 * configure() function is optional if you want to configure
 * the  pins manually.
 */
struct OWI_Interface_Struct;
typedef struct OWI_Interface_Struct OWI_Interface;

struct OWI_Interface_Struct
{
	OWI_Result (*init_driver)(OWI_Interface *interface);
	OWI_Result (*pull_bus_lo)(OWI_Interface *interface);
	OWI_Result (*release_bus)(OWI_Interface *interface);
	OWI_PinState (*get_bus_state)(OWI_Interface *interface);
};

typedef void (*OWI_DelayMicrosecondsFn)(uint16_t us);

/* OWI control structure */
typedef struct
{
	OWI_Interface *interface;
	OWI_DelayMicrosecondsFn delay_microseconds;
	uint8_t owi_device_n;
} OWI;

/****************************************************************************
 ROM commands
 ****************************************************************************/
#define OWI_ROM_READ		0x33    //!< READ ROM command code.
#define OWI_ROM_SKIP		0xcc    //!< SKIP ROM command code.
#define OWI_ROM_MATCH	0x55    //!< MATCH ROM command code.
#define OWI_ROM_SEARCH	0xf0    //!< SEARCH ROM command code.

/****************************************************************************
 FUNCTION commands
 ****************************************************************************/
#define OWI_START_CONVERSION         0x44
#define OWI_READ_SCRATCHPAD          0xbe
#define OWI_WRITE_SCRATCHPAD         0x4e
#define OWI_COPY_SCRATCHPAD          0x48

/*****************************************************************************
 Timing parameters
 *****************************************************************************/

// Bit timing delays in clock cycles us
#define OWI_DELAY_A_STD_MODE    2-2
#define OWI_DELAY_B_STD_MODE    68-2
#define OWI_DELAY_C_STD_MODE    60-2
#define OWI_DELAY_D_STD_MODE    10-2
#define OWI_DELAY_E_STD_MODE    8-2
#define OWI_DELAY_F_STD_MODE    55-2
//#define OWI_DELAY_G_STD_MODE  0
#define OWI_DELAY_H_STD_MODE    480-2
#define OWI_DELAY_I_STD_MODE    70-2
#define OWI_DELAY_J_STD_MODE    410-2

// 1-Wire bit functions

//void DS18B20_Init(uint8_t, GPIO_TypeDef * , uint32_t, uint32_t);
OWI_Result OWI_Init(OWI *owi_int);
OWI_Result OWI_WriteBit1(OWI *owi_int);
OWI_Result OWI_WriteBit0(OWI *owi_int);
OWI_PinState OWI_ReadBit(OWI *owi_int);
OWI_Result OWI_DetectPresence(OWI *owi_int);

// 1-Wire high level functions

OWI_Result OWI_SendByte(OWI *owi_int, uint8_t data);
uint8_t OWI_ReceiveByte(OWI *owi_int);
void OWI_SkipRom(OWI *owi_int);
void OWI_ReadRom(OWI *owi_int, uint8_t * romValue);

// CRC functions

#define     OWI_CRC_OK      0x00    //!< CRC check succeded
#define     OWI_CRC_ERROR   0x01    //!< CRC check failed

uint8_t OWI_ComputeCRC8(uint8_t inData, uint8_t seed);
uint16_t OWI_ComputeCRC16(uint8_t inData, uint16_t seed);
uint8_t OWI_CheckRomCRC(uint8_t * romValue);

#endif /* OWI_H_ */

