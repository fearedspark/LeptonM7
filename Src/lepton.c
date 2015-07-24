#include "lepton.h"

uint16_t Lepton_VoSPI[82];

uint16_t LeptonBuffer[2][80*60];

int current_lepton_buffer = -1;

BufferStatusCodes BufferStatus[2] = {BUFFER_FREE, BUFFER_FREE};

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
    int i;
    uint16_t line;
    if(hspi->Instance == SPI2)
    {
        LEPTON_CS_HIGH();
        line = __REV16(Lepton_VoSPI[0]) & 0x0FFF;
        if(line < 60)
        {
            if((current_lepton_buffer < 0) && (line == 0))
            {
                if(BufferStatus[0] == BUFFER_FREE)
                {
                    current_lepton_buffer = 0;
                    BufferStatus[0] = BUFFER_BUSY_LEPTON;
                }
                else if(BufferStatus[1] == BUFFER_FREE)
                {
                    current_lepton_buffer = 1;
                    BufferStatus[1] = BUFFER_BUSY_LEPTON;
                }
            }
            if(current_lepton_buffer >= 0)
            {
                for(i = 0; i < 80; i++)
                {
                    LeptonBuffer[current_lepton_buffer][line*80 + i] = __REV16(Lepton_VoSPI[i + 2]);
                }
                if(line == 59)
                {
                    BufferStatus[current_lepton_buffer] = BUFFER_RDY_DISP;
                    current_lepton_buffer = -1;
                }
            }
        }
        LEPTON_CS_LOW();
        while(HAL_SPI_Receive_DMA(hspi, (uint8_t *) Lepton_VoSPI, 164) != HAL_OK);
    }
}

void LeptonSync(void)
{
    LEPTON_CS_LOW();
    wait_us(10);
    LEPTON_CS_HIGH();
    wait_ms(190);
}
