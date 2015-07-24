#ifndef __LEPTON_H__
#define __LEPTON_H__

#include "shared.h"

#define LEPTON_CS_LOW() GPIOA->BSRR = 0x01000000
#define LEPTON_CS_HIGH() GPIOA->BSRR = 0x00000100

typedef enum
{
    BUFFER_FREE,
    BUFFER_BUSY_LEPTON,
    BUFFER_RDY_DISP,
    BUFFER_BUSY_DISP
}
BufferStatusCodes;

extern uint16_t Lepton_VoSPI[82];
extern uint16_t LeptonBuffer[2][80*60];
extern BufferStatusCodes BufferStatus[2];

void LeptonSync(void);

#endif
