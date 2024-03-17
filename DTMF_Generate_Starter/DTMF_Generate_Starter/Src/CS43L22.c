#include "CS43L22.h"
#include <stdint.h>

void CS43L22_EXTERNAL_DAC_enable(void);
void CS43L22_EXTERNAL_DAC_I2C_write(uint8_t *iData, uint8_t len);
void CS43L22_EXTERNAL_DAC_I2C_recieve(uint8_t *iData);
void CS43L22_EXTERNAL_DAC_I2S_transmit(uint16_t *buffer,uint16_t buffer_size);

void CS43L22_init(){

	uint8_t iData[2]; // Buffer to read, change and write back register values of the DAC

	// - Set RESET high
	CS43L22_EXTERNAL_DAC_enable();

	// - Load desired register settings

	CS43L22_write( CS43L22_REG_POWER_CTL1, 0x01, 2);

	// - Power Ctl 2
	// PDN_HPB[0:1]  = 10 (HP-B always on)
	// PDN_HPA[0:1]  = 10 (HP-A always on)
	// PDN_SPKB[0:1] = 11 (Speaker B always off)
	// PDN_SPKA[0:1] = 11 (Speaker A always off)

	CS43L22_write( CS43L22_REG_POWER_CTL2, 0xAF, 2);
	CS43L22_write( CS43L22_REG_CLOCKING_CTL,(1 << 7), 2); //Tal vez tenga que poner 0x81 para dividir clock

	iData[0] = CS43L22_REG_INTERFACE_CTL1;

	CS43L22_write(iData[0], iData[1], 1);// Transmit register address to the device ...
	CS43L22_EXTERNAL_DAC_I2C_recieve(&iData[1]); // ... and read 1 byte (the register content).

	iData[1] &= (1 << 5); // Clear all bits except bit 5 which is reserved
	iData[1] &= ~(1 << 7);  // Slave
	iData[1] &= ~(1 << 6);  // Clock polarity: Not inverted
	iData[1] &= ~(1 << 4);  // No DSP mode
	iData[1] |= (1 << 2);  // I2S up to 24 bit (default)
	iData[1] |=  (3 << 0);  // 16-bit audio word length for I2S interface

	CS43L22_write(iData[0], iData[1], 2);

	iData[0] = CS43L22_REG_MISC_CTL ;

	CS43L22_write(iData[0], iData[1], 1); // Transmit register address to the device ...
	CS43L22_EXTERNAL_DAC_I2C_recieve(&iData[1]);  // ... and read 1 byte (the register content).

	iData[1] &= ~(1 << 7);   // Disable passthrough for AIN-A
	iData[1] &= ~(1 << 6);   // Disable passthrough for AIN-B
	iData[1] |=  (1 << 5);   // Mute passthrough on AIN-A
	iData[1] |=  (1 << 4);   // Mute passthrough on AIN-B
	iData[1] &= ~(1 << 3);   // Changed settings take affect immediately

	CS43L22_write(iData[0], iData[1], 2);

	//   - Unmute headphone and speaker
	CS43L22_write( CS43L22_REG_PLAYBACK_CTL2, 0x00, 2);
	//   - Volume PCM-A and Volume PCM-B
	CS43L22_write( CS43L22_REG_PCMA_VOL, 0x00, 2);
	CS43L22_write( CS43L22_REG_PCMB_VOL, 0x00, 2);

	// - Perform initialization as described in the datasheet of the CS43L22, Chapter "4.11 Required Initialization Settings"

	// Write 0x99 to register 0x00.
	CS43L22_write( 0x00, 0x99, 2);
	// Write 0x80 to register 0x47.
	CS43L22_write( 0x47, 0x80, 2);
	// Write '1'b to bit 7 in register 0x32.
	iData[0] = 0x32;
	CS43L22_write(iData[0], iData[1], 1);
	CS43L22_EXTERNAL_DAC_I2C_recieve(&iData[1]);
	iData[1] |= 0x80;
	CS43L22_write(iData[0], iData[1], 2);
	// Write '0'b to bit 7 in register 0x32.
	iData[0] = 0x32;


	CS43L22_write(iData[0], iData[1], 1);
	CS43L22_EXTERNAL_DAC_I2C_recieve(&iData[1]);
	iData[1] &= ~(0x80);
	CS43L22_write(iData[0], iData[1], 2);

	// Write 0x00 to register 0x00.
	CS43L22_write( 0x00, 0x00, 2);
	CS43L22_write( CS43L22_REG_POWER_CTL1 , 0x9E, 2);
}

void CS43L22_write(uint8_t reg, uint8_t Cmd, uint8_t len){

	uint8_t iData[2]; // Buffer to read, change and write back register values of the DAC
	iData[0] =  reg;
	iData[1] = Cmd;

	CS43L22_EXTERNAL_DAC_I2C_write(iData,len);


}

void CS43L22_AudioSend(uint16_t *buffer,uint16_t buffer_size){

	CS43L22_EXTERNAL_DAC_I2S_transmit(buffer,buffer_size);

}

