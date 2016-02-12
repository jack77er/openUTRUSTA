/** @file spi.h
*
* @brief SPI functions
*
* @author Alvaro Prieto
*/
#ifndef _stm32l0_cc2500_
#define _stm32l0_cc2500_

#include <stdint.h>
//
//void wait_cycles(uint16_t);
//void spi_setup(void);
//void cc_write_reg(uint8_t_t, uint8_t_t);
//void cc_write_burst_reg(uint8_t_t, uint8_t_t*, uint8_t_t);
//uint8_t cc_read_reg(uint8_t_t);
//void cc_read_burst_reg(uint8_t_t, uint8_t_t *, uint8_t_t);
//uint8_t cc_read_status(uint8_t_t);
//void cc_strobe(uint8_t_t);
//void cc_powerup_reset(void);

// Configuration Registers
#define CC2500_IOCFG2       0x00        // GDO2 output pin configuration
#define CC2500_IOCFG1       0x01        // GDO1 output pin configuration
#define CC2500_IOCFG0       0x02        // GDO0 output pin configuration
#define CC2500_FIFOTHR      0x03        // RX FIFO and TX FIFO thresholds
#define CC2500_SYNC1        0x04        // Sync word, high byte
#define CC2500_SYNC0        0x05        // Sync word, low byte
#define CC2500_PKTLEN       0x06        // Packet length
#define CC2500_PKTCTRL1     0x07        // Packet automation control
#define CC2500_PKTCTRL0     0x08        // Packet automation control
#define CC2500_ADDR         0x09        // Device address
#define CC2500_CHANNR       0x0A        // Channel number
#define CC2500_FSCTRL1      0x0B        // Frequency synthesizer control
#define CC2500_FSCTRL0      0x0C        // Frequency synthesizer control
#define CC2500_FREQ2        0x0D        // Frequency control word, high byte
#define CC2500_FREQ1        0x0E        // Frequency control word, middle byte
#define CC2500_FREQ0        0x0F        // Frequency control word, low byte
#define CC2500_MDMCFG4      0x10        // Modem configuration
#define CC2500_MDMCFG3      0x11        // Modem configuration
#define CC2500_MDMCFG2      0x12        // Modem configuration
#define CC2500_MDMCFG1      0x13        // Modem configuration
#define CC2500_MDMCFG0      0x14        // Modem configuration
#define CC2500_DEVIATN      0x15        // Modem deviation setting
#define CC2500_MCSM2        0x16        // Main Radio Cntrl State Machine config
#define CC2500_MCSM1        0x17        // Main Radio Cntrl State Machine config
#define CC2500_MCSM0        0x18        // Main Radio Cntrl State Machine config
#define CC2500_FOCCFG       0x19        // Frequency Offset Compensation config
#define CC2500_BSCFG        0x1A        // Bit Synchronization configuration
#define CC2500_AGCCTRL2     0x1B        // AGC control
#define CC2500_AGCCTRL1     0x1C        // AGC control
#define CC2500_AGCCTRL0     0x1D        // AGC control
#define CC2500_WOREVT1      0x1E        // High byte Event 0 timeout
#define CC2500_WOREVT0      0x1F        // Low byte Event 0 timeout
#define CC2500_WORCTRL      0x20        // Wake On Radio control
#define CC2500_FREND1       0x21        // Front end RX configuration
#define CC2500_FREND0       0x22        // Front end TX configuration
#define CC2500_FSCAL3       0x23        // Frequency synthesizer calibration
#define CC2500_FSCAL2       0x24        // Frequency synthesizer calibration
#define CC2500_FSCAL1       0x25        // Frequency synthesizer calibration
#define CC2500_FSCAL0       0x26        // Frequency synthesizer calibration
#define CC2500_RCCTRL1      0x27        // RC oscillator configuration
#define CC2500_RCCTRL0      0x28        // RC oscillator configuration
#define CC2500_FSTEST       0x29        // Frequency synthesizer cal control
#define CC2500_PTEST        0x2A        // Production test
#define CC2500_AGCTEST      0x2B        // AGC test
#define CC2500_TEST2        0x2C        // Various test settings
#define CC2500_TEST1        0x2D        // Various test settings
#define CC2500_TEST0        0x2E        // Various test settings

// Strobe commands
#define CC2500_SRES         0x30        // Reset chip.
#define CC2500_SFSTXON      0x31        // Enable/calibrate freq synthesizer
#define CC2500_SXOFF        0x32        // Turn off crystal oscillator.
#define CC2500_SCAL         0x33        // Calibrate freq synthesizer & disable
#define CC2500_SRX          0x34        // Enable RX.
#define CC2500_STX          0x35        // Enable TX.
#define CC2500_SIDLE        0x36        // Exit RX / TX
#define CC2500_SAFC         0x37        // AFC adjustment of freq synthesizer
#define CC2500_SWOR         0x38        // Start automatic RX polling sequence
#define CC2500_SPWD         0x39        // Enter pwr down mode when CSn goes hi
#define CC2500_SFRX         0x3A        // Flush the RX FIFO buffer.
#define CC2500_SFTX         0x3B        // Flush the TX FIFO buffer.
#define CC2500_SWORRST      0x3C        // Reset real time clock.
#define CC2500_SNOP         0x3D        // No operation.

// Status registers
#define CC2500_PARTNUM      0x30        // Part number
#define CC2500_VERSION      0x31        // Current version number
#define CC2500_FREQEST      0x32        // Frequency offset estimate
#define CC2500_LQI          0x33        // Demodulator estimate for link quality
#define CC2500_RSSI         0x34        // Received signal strength indication
#define CC2500_MARCSTATE    0x35        // Control state machine state
#define CC2500_WORTIME1     0x36        // High byte of WOR timer
#define CC2500_WORTIME0     0x37        // Low byte of WOR timer
#define CC2500_PKTSTATUS    0x38        // Current GDOx status and packet status
#define CC2500_VCO_VC_DAC   0x39        // Current setting from PLL cal module
#define CC2500_TXBYTES      0x3A        // Underflow and # of bytes in TXFIFO
#define CC2500_RXBYTES      0x3B        // Overflow and # of bytes in RXFIFO
#define CC2500_NUM_RXBYTES  0x7F        // Mask "# of bytes" field in _RXBYTES

// Other memory locations
#define CC2500_PATABLE      0x3E
#define CC2500_TXFIFO       0x3F
#define CC2500_RXFIFO       0x3F

// Masks for appended status bytes
#define CC2500_LQI_RX       0x01        // Position of LQI byte
#define CC2500_CRC_OK       0x80        // Mask "CRC_OK" bit within LQI byte

// Definitions to support burst/single access:
//#define CC2500_WRITE_BURST  0x40
//#define CC2500_READ_SINGLE  0x80
//#define CC2500_READ_BURST   0xC0
#define CC2500_WRITE_SINGLE	0x3F
#define CC2500_WRITE_BURST	0x7F
#define CC2500_READ_SINGLE	0xBF
#define CC2500_READ_BURST	0xFF
//// RF settings for CC2500
//typedef enum {
//	SRES 	= 0x30,
//	SFSTXON = 0x31,
//	SXOFF 	= 0x32,
//	SCAL 	= 0x33,
//	SRX 	= 0x34,
//	STX 	= 0x35,
//	SIDLE 	= 0x36,
//	SWOR	= 0x38,
//	SPWD	= 0x39,
//	SFRX	= 0x3A,
//	SFTX	= 0x3B,
//	SWORRST	= 0x3C,
//	SNOP	= 0x3D,
//
//} cc2500_cmd_t;
//
//typedef enum {
//	0x0
//} cc2500_reg_t;

typedef struct {
    uint8_t iocfg2;     // GDO2Output Pin Configuration
    uint8_t iocfg1;		// GDO1Output Pin Configuration
    uint8_t iocfg0;     // GDO0Output Pin Configuration
    uint8_t pktctrl1;   // Packet Automation Control 1
    uint8_t pktctrl0;   // Packet Automation Control 0
    uint8_t channr;     // Channel Number
    uint8_t fsctrl1;    // Frequency Synthesizer Control
    uint8_t fsctrl0;    // Frequency Synthesizer Control
    uint8_t freq2;      // Frequency Control Word, High Byte
    uint8_t freq1;      // Frequency Control Word, Middle Byte
    uint8_t freq0;      // Frequency Control Word, Low Byte
    uint8_t mdmcfg4;    // Modem Configuration
    uint8_t mdmcfg3;    // Modem Configuration
    uint8_t mdmcfg2;    // Modem Configuration
    uint8_t mdmcfg1;    // Modem Configuration
    uint8_t mdmcfg0;    // Modem Configuration
    uint8_t deviatn;    // Modem Deviation Setting
    uint8_t mcsm2;      // Main Radio Control State Machine Configuration
    uint8_t mcsm1;      // Main Radio Control State Machine Configuration
    uint8_t mcsm0;      // Main Radio Control State Machine Configuration
    uint8_t foccfg;     // Frequency Offset Compensation Configuration
    uint8_t bscfg;      // Bit Synchronization Configuration
    uint8_t agcctrl2;   // AGC Control
    uint8_t agcctrl1;   // AGC Control
    uint8_t agcctrl0;   // AGC Control
    uint8_t frend1;     // Front End RX Configuration
    uint8_t frend0;     // Front End RX Configuration
    uint8_t fscal3;     // Frequency Synthesizer Calibration
    uint8_t fscal2;     // Frequency Synthesizer Calibration
    uint8_t fscal1;     // Frequency Synthesizer Calibration
    uint8_t fscal0;     // Frequency Synthesizer Calibration
    uint8_t sync1;		// Frequency Synchronization
    uint8_t sync0;		// Frequency Synchronization
    uint8_t rcctrl1;	// RC Oscillator Control
    uint8_t rcctrl0;	// RC Oscillator Control
    uint8_t worevt1;	// Wake on Radio Timeout
	uint8_t worevt0;	// Wake on Radio Timeout
	uint8_t	worctrl;	// Wake on Radio Control
	uint8_t fstest;		// Testregister
	uint8_t test2;		// Testregister
	uint8_t test1;		// Testregister
	uint8_t test0;		// Testregister
	uint8_t addr;		// Device Adress Register
	uint8_t pktlen;		// Packet Length
	uint8_t fifothr;	// FIFO Treshold Value
} rf_settings_t;

// Rf settings for CC2500
rf_settings_t rf_settings_default = {
    0x29,  // IOCFG2        GDO2Output Pin Configuration
	0x00,  // IOCFG1        GDO1Output Pin Configuration
    0x06,  // IOCFG0        GDO0Output Pin Configuration
	0x12,  // PKTCTRL1      Packet Automation Control
	0x12,  // PKTCTRL0      Packet Automation Control
    0x10,  // CHANNR        Channel Number
    0x09,  // FSCTRL1       Frequency Synthesizer Control
    0x00,  // FSCTRL0       Frequency Synthesizer Control
    0x5D,  // FREQ2         Frequency Control Word, High Byte
    0x93,  // FREQ1         Frequency Control Word, Middle Byte
    0xB1,  // FREQ0         Frequency Control Word, Low Byte
    0x2D,  // MDMCFG4       Modem Configuration
    0x3B,  // MDMCFG3       Modem Configuration
    0x73,  // MDMCFG2       Modem Configuration
    0xA2,  // MDMCFG1       Modem Configuration
    0xF8,  // MDMCFG0       Modem Configuration
    0x01,  // DEVIATN       Modem Deviation Setting
    0x07,  // MCSM2         Main Radio Control State Machine Configuration
    0x30,  // MCSM1         Main Radio Control State Machine Configuration
    0x18,  // MCSM0         Main Radio Control State Machine Configuration
    0x1D,  // FOCCFG        Frequency Offset Compensation Configuration
    0x1C,  // BSCFG         Bit Synchronization Configuration
    0xC7,  // AGCCTRL2      AGC Control
    0x00,  // AGCCTRL1      AGC Control
    0xB2,  // AGCCTRL0      AGC Control
    0xB6,  // FREND1        Front End RX Configuration
    0x10,  // FREND0        Front End RX Configuration
    0xEA,  // FSCAL3        Frequency Synthesizer Calibration
    0x0A,  // FSCAL2        Frequency Synthesizer Calibration
    0x00,  // FSCAL1        Frequency Synthesizer Calibration
    0x11,  // FSCAL0        Frequency Synthesizer Calibration
	0xD3,  // SYNC1
	0x91,  // SYNC0
	0x41,  // RCCTRL1
	0x00,  // RCCTRL0
	0x87,  // WOREVT1
	0x6B,  // WOREVT0
	0xF8,  // WORCTRL
	0x59,  // FSTEST
	0x88,  // TEST2
	0x31,  // TEST1
	0x0B,  // TEST0
	0x01,  // ADDR
	0xff,  // PKTLEN
	0x07   // FIFOTRH
};


#endif /* _stm32l0_cc2500_ */
