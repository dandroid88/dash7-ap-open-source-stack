#include <stdint.h>

const uint8_t rfSettings[] = {
	    0x02, 0x06,  // IOCFG0        GDO0 Output Pin Configuration
	    0x00, 0x0B,   // IOCFG2    GDO2 output pin configuration.
	    0x07, 0x04,   // PKTCTRL1  Packet automation control.
	    0x08, 0x05,  // PKTCTRL0      Packet Automation Control
	    0x0b, 0x0C,  // FSCTRL1       Frequency Synthesizer Control
	    0x0d, 0x10,  // FREQ2         Frequency Control Word, High Byte
	    0x0e, 0xB1,  // FREQ1         Frequency Control Word, Middle Byte
	    0x0f, 0x3B,  // FREQ0         Frequency Control Word, Low Byte
	    0x10, 0x2D,  // MDMCFG4       Modem Configuration
	    0x11, 0x3B,  // MDMCFG3       Modem Configuration
	    0x12, 0x1B,  // MDMCFG2       Modem Configuration
	    0x15, 0x62,  // DEVIATN       Modem Deviation Setting
	    0x18, 0x18,  // MCSM0         Main Radio Control State Machine Configuration
	    0x19, 0x1D,  // FOCCFG        Frequency Offset Compensation Configuration
	    0x1a, 0x1C,  // BSCFG         Bit Synchronization Configuration
	    0x1b, 0xC7,  // AGCCTRL2      AGC Control
	    0x1c, 0x00,  // AGCCTRL1      AGC Control
	    0x1d, 0xB0,  // AGCCTRL0      AGC Control
	    0x20, 0xFB,  // WORCTRL       Wake On Radio Control
	    0x21, 0xB6,  // FREND1        Front End RX Configuration
	    0x23, 0xEA,  // FSCAL3        Frequency Synthesizer Calibration
	    0x24, 0x2A,  // FSCAL2        Frequency Synthesizer Calibration
	    0x25, 0x00,  // FSCAL1        Frequency Synthesizer Calibration
	    0x26, 0x1F,  // FSCAL0        Frequency Synthesizer Calibration
	    0x2e, 0x09,  // TEST0         Various Test Settings
	0xFF, 0xFF
};
