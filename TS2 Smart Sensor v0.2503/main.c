/*
 * ======== Standard MSP430 includes ========
 */
#include <msp430.h>

/*
 * ======== Grace related includes ========
 */
#include <ti/mcu/msp430/Grace.h>


		// SMART SENSOR IDENTIFICATION
#define NEST_ID "123456.000,1111.2222N,33333.4444E,060614,AA0000"
		// The last 6 digits is a unique serial number for each smart sensor.
		// Update the last 6 digits before compiling for each new device.
		// The rest of the data will be updated during next registration
		// What you see here is just an example of what it could look like
#define SERIAL_NUMBER "AA0000"
		// Update it here as well.
		// This one won't be modified when the code is run

		// SMART SENSOR TEMPERATURE CALIBRATION
#define TEMPERATURE_OFFSET 0x0000		// Default setting of 0 does not change the reading
#define TEMPERATURE_RATIO 0x1000		// The temperature reading adjusted by the offset times this number will be divided by 0x1000,
		// These values need to be manually entered for each smart sensor.
		// Eventually there will be an automated procedure for setting them.
/*
***************************************************************************************
//
// TURTLE SENSE
// PHASE TWO -- Smart Turtle Sensor Module
// Programmed By Samuel Wantman CC 4.0 BY-SA
//
//      VERSION HISTORY OF PHASE TWO
// 		V0.01 	2/21/14 First operational version for use with Phase II board version 0.14
//            	3/7/14  Code divided to make Sensor and communicator modules
//			  	5/1/14  First run of 28 sensors produced version 0.25
//		V0.2501	5/29/14 version 0.2501 installed on 3 sensors sent to NC
//		V0.2502 6/25/14 version 0.2502 installed on 9 sensors sent to NC essentially unchanged
//		V0.2503
 *
//
// some routines based on code logic from PHASE ONE --Turtle Dialer Phone Interface
// 											Programmed by Thomas Zimmerman  CC 4.0 BY-SA
****************************************************************************************
// HIGH PRIORITY

// TODO // adjust timings for slow and fast days


// Lower priority
// TODO //  Move all the status reports for READY TO RECEIVE to appropriate spots in the routines
// TODO //  Add paramteters for changing clock speeds, timer intervals, and UART speeds
// TODO //  Average orientation computed instead of single read per record
// TODO //  Make DATA_BUFFER_SIZE a variable so that SPI buffer reads can be optimized
// TODO //  Test completion of run by reaching the limit of max_run_time, set by Comm tower.
// TODO //  Test all unused commands to see if they works correctly
// TODO //  Interrupt routine to wake things up if the MSP430 is put to sleep.
// TODO //  test manual reads and writes
// TODO //  test stream FIFO, (transfers data from data buffer to RS485 without processing)
// TODO //  make SLEEP_INTERVALS setting automatic based on Sample Speed.
// TODO //  Automated procedure for calibrating sensors
// TODO //  make UART and SPI communications interrupt driven (next version?)

// Logic for Phase Two
// -------------------
// Initialize Microproessor (settings using GRACE)
// Establish communication with comm tower
// Then the comm tower or hand held unit takes control:
// 		Upload ID, which is serial number and previous GPS location
// 		Wait for confirmation of GPS location or new location
// 		Wait for initiation paramters from comm tower
// 		If there is confirmation upload any old unsent data
// 		Start sensor reads

// Main logic
// //////////
// The sensor stack is read every second or fraction of a second depending upon initiation parameters.
// The frequency of the stack read can be 1,2,4 or 8 times per second
// The sensor can be set to make readings up to 400 times per second, but the each stack read
// is limited to 100 readings, so for speeds faster than 100 Hz there should be multiple reads each second.
// to optimize power, set things up with a smaller buffer and more frequent reads
// The CPU goes to sleep in between reads, and is awakened by a clock interrupt.
// Once awakened, the CPU downloads all the data off the sensor's FIFO stack.
// The data is analyzed to compute the change in acceleration (jerk)
// From the delta x, y and z of consecutive reads, the square of magnitude is calculated..
// Each reading increments one counter in a histogram by scanning for the highest bit
// Every 20 seconds to 6 minutes, a new histogram starts, so that the memory is full of them after a day (or shorter interval)
//
//
// The comm tower initates a phone call and uploads data from the sensor.
// An upload can occur whenever requested by the comm tower, so it is possible to store half the data on that device.
// The comm tower can ask for the most recent record, or all the records.  They can also be removed one by one by repeatedly
// asking to delete the most recent record (the record counter is decreased each time a record is deleted).
// If major activity is detected frequency of reporting can increase to every 2 hours and histograms every 15-30 secs.
// The comm tower makes decisions about reporting frequency and sends new parameters to make it happen.

// All unused pins should be set to Output Low to minimize current drain.


	PIN HARDWARE USAGE
	MSP430FR5739

PIN CONNCECTION	    PORT	FUNC	HIGH	LOW		USAGE
1		XIN			J.4		---		---		---		Crystal
2		XOUT		J.5		---		---		---		Crystal
3		AVSS		---		---		---		---		Analog Ground
4		AVCC		---		---		---		---		Analog 3.3 V
5		---			1.0		---		---		---		Not connected (output low)
6		---			1.1		---		---		---		Not connected (output low)
7		SensorPWR	1.2		OUT		ON		OFF		Sensor Power (set low to reset)
8		---			3.0		---		---		---		Not connected (output low)
9		---			3.1		---		---		---		Not connected (output low)
10		TX/RX ON	3.2		IN		TX/RX	OFF		Monitor whether RS485 (TX/RX) is On or Off
11		INTERRUPT	3.3		OUT		RQ		---		Communication interrupt request to Comm board
12		---			1.3		---		---		---		Not connected (output low)
13		ADXL-INT1	1.4		IN		RQ		---		Sensor interrupt 1 (internal pull-down)
14		ADXL-INT2	1.5		IN		RQ		---		Sensor interrupt 2 (internal pull-down)
15		TDO			J.0		Opt		Opt		Opt		JTAG connector
16		TDI			J.1		Opt		Opt		Opt		JTAG connector
17		TMS			J.2   	Opt		Opt		Opt		JTAG connector
18		TCK			J.3  	Opt		Opt		Opt		JTAG connector
19		ADXL-MOSI	2.5		SPI A1	DATA	DATA	Sensor SPI
20		ADXL-MISO	2.6 	SPI A1	DATA	DATA	Sensor SPI
21		TEST		TEST  	---		---		---		JTAG connector
22		#RST		#RST  	---		---		---		JTAG connector
23		U-OUT		2.0		UCA0TXD	DATA	DATA	Uart to Comm Head, UCA0TXD
24		U-IN		2.1		UCA0RXD	DATA	DATA	Uart from Comm Head, UCA0RXD
25		---			2.2		---		---		---		Not connected (output low)
26		TAMPER		3.4		OUT		Normal	Tamper	Communication tamper line: Output High)
27		---			3.5		---		---		---		Not connected (output low)
28		---			3.6		---		---		---		Not connected (output low)
29		(LED2)		3.7		---		---		---		Not connected (output low) (prototype had an LED)
30		---			1.6		---		---		---		Not connected (output low)
31		---			1.7		---		---		---		Not connected (output low)
32		VCORE		VCORE	---		---		---		Internal power
33		DVSS		DVSS	---		---		---		Digital Ground
34		DVCC		DVCC	---		---		---		Digital Power
35		(LED1)		2.7		---		---		---		Not connected (output low) (prototype had an LED)
36		ADXL-CS		2.3		OUT		CS		OFF		Sensor SPI chip select (set as output high if sensor is not connected)
37		ADXL-CLK	2.4		SPI A1	CLK		CLK		Sensor SPI
38		AVSS		AVSS	---		---		---		Analog ground
*/

// Interrupt code resides in InterruptVectors_init.c


//////////////////////////////////////////////////////////
// Program definitions
//////////////////////////////////////////////////////////

	//Default data recording parameters:
	// see the  SAMPLE_SPEED documentation for the ADXL sensor below
	// The sample speed is the rate of sensor reads (12.5, 25, 50, 100, 200 or 400 Hz)
	// 0 = 12.5 Hz, 1 = 25 Hz, 2 = 50 Hz, 3 = 100 Hz,  4 = 200 Hz and 5 = 400 Hz
#define READ_SPEED 		1		// Default number of sets of buffer reads per second.
#define SLEEP_INTERVALS 8		// Default is 8.  Must be a power of 2.
								// The timer is set to have an interrupt every 1/8 of a second, so this is number of interrupts to
								// wait before emptying the buffer
								// Each set of sensor readings is 6 bytes.  2 bytes for each sensor read of X, Y and Z acceleration
								// Temperature is only read once for each record.
								// The ADXL FIFO buffer is set to hold a maximum of 510 readings (a maximum of 170 sensor reading sets of 3)
								// If the speed is 100 Hz or less the buffer can be emptied once per second.
								// For 200 reads, this should be set to 2 buffer downloads (SLEEP_INTERVALS = 4)
								// For 400 reads, this should be set to 4 buffer downloads. (SLEEP_INTERVALS = 2)
								// the number of samples per second times SLOWDAYBINSEC can not be more than 64K or bins might overflow
	// Data Buffer definitions
#define DATA_BUFFER_SIZE 440	// The size of the data buffer for receiving data from SPI (set to maximum for 100 readings + 10%)


	// Default timings
#define SECS_IN_DAY 86400		// Number of seconds in a day
#define MAXRUN (SECS_IN_DAY-5)	// The maximum length of a run before an interrupt (a day minus a few seconds)
#define MAXRUN4H ((MAXRUN & 0xFF000000) / 0x01000000) // The highest byte of MAXRUN
#define MAXRUN3  ((MAXRUN & 0x00FF0000) / 0x00010000)
#define MAXRUN2  (MAXRUN & 0x0000FF00) / 0x00000100
#define MAXRUN1L  (MAXRUN & 0x000000FF)			// The lowest byte of MAXRUN
#define HOURS 2					// Number of hours for data collection per call-in period once motion detected.  Can be any integer value.
#define BINSEC 30				// The default is 30  // Number of seconds in each bin record.  Must be factor of 3600
#define BINSEC_HI ((BINSEC & 0xFF00) / 0x0100)	// The high byte of SLOWDAY_BIN_SEC
#define BINSEC_LO (BINSEC & 0x00FF) 		// The low byte of SLOWDAY_BIN_SEC
#define SLOWDAY 24				// Number of hours in a slow day of data collection
#define SLOWDAY_BIN_SEC 360   	// eventually this will be 360 but could be less if records are also stored in comm board.
								// Number of seconds in each bin record on a slow day (every 6 minutes)
#define SLOWBIN_HI ((SLOWDAY_BIN_SEC & 0xFF00) / 0x0100)	// The high byte of SLOWDAY_BIN_SEC
#define SLOWBIN_LO (SLOWDAY_BIN_SEC & 0x00FF)	// The low byte of SLOWDAY_BIN_SEC
#define MAX_BIN 10	     		// number of histogram bins in each record
#define DATA_SIZE 16			// This is MAX_BIN plus other parameters (currently 6 - Temp, X, Y, Z, Count, Max)
#define DATA_BYTES 32			// The number of bytes in one record (DATA_SIZE * 2)
#define MAX_RECORDS 240  		// eventually this will be 240  // The maximum number of Bin records assuming about 11 K and 32 bytes per record
#define MAX_RECORDS_HI ((MAX_RECORDS & 0xFF00) / 0x0100) 	// The high byte of MAX_RECORDS
#define MAX_RECORDS_LO  (MAX_RECORDS & 0x00FF) 	// The low byte of MAX_RECORDS

#define CLOCKSPEED 8			// The number of  million cycles per second for the normal clockspeed

	// Other Smart Sensor Parameters
#define CALIBRATE_TEMP YES		// Should temperature readings be calibrated?

//    #define  MILLISECOND 24000		// The number of cycles in a millisecond

	//  Misc. Program definitions
#define SHUT_OFF_LEVEL 0x01DD	// The ADC reading for .7 volts
#define JOLT_BITS 29			// The number of usable bits in dx^2 + dy^2 + dz^2  each delta is a maximum of 13 bits,
								// The maximum for the square of 13 bits is 26 bits, and the maximum for the sum
								// or three 26 bit numbers is 28 bits. Another bit for a reading of zero
#define JOLT_MASK 0x08000000	// A mask for the staring bit of a jolt (the 28th bit)

	// General purpose definitions
#define TRUE 1
#define FALSE 0
#define YES 1
#define NO 0
#define SEND 1
#define RECEIVE 0
#define CR 13					// Carriage return
#define SPACE 32				// space character

	// Smart Sensor Interrupt codes
	// These codes are returned after a command 0x0F is received
	// thet explain the reason for generating an interrupt
	// All codes less than 0x80 are not errors.
#define NO_ERROR  0x00 				// = No error -- proceeding as instructed
#define NO_ERROR_INT1 0x01 			// = INT1 generated an interrupt
#define NO_ERROR_INT2 0x02			// = INT2 generated an interrupt
#define NO_ERROR_DATA_READY 0x04 	// = operation completed successfully, data ready to report
#define NO_ERROR_COMPLETE 0x08		// = operation completed successfully, no data to report
#define NO_ERROR_START_UP 0x10		// = Normal start-up
#define NO_ERROR_SIGN_OFF 0x20		// = processing beginning, communication over
#define NO_ERROR_READY_TO_RECEIVE 0x40 // = ready to receive data from master

// >= 0x80 = an error condition exists.  The error code is returned:
#define ERROR_SENSOR_TIMEOUT  0x81	// = Sensor Start up failure (timed out)
#define ERROR_NO_COMMAND 0x82		// = Unknown command received
#define ERROR_BAD_DATA 0x84			// = Bad Data received
#define ERROR_TX_TIMEOUT 0x88		// = Data transmission timed out
#define ERROR_OUT_OF_RANGE 0x90		// = Parameters out of range
#define ERROR_START_UP 0xA0 		// = Unknown Start up error


//////////////////////////////////////////////////////////
// MSP430 Microprocessor definitions
//////////////////////////////////////////////////////////

	//	(TSS = Turtle Smart Sensor)

//#define TSS_LED BIT7				//  LEDs are connected to pin 2.7 and 3.7 only in prototype

	// Port 1

#define TSS_ADXL_POWER BIT2
#define TSS_ADXL_INT1 BIT4
#define TSS_ADXL_INT2 BIT5

	// Port 2
#define TSS_UCA0TXD BIT0
#define TSS_UCA0RXD BIT1
#define TSS_ADXL_CS BIT3
#define TSS_ADXL_CLK BIT4
#define TSS_ADXL_MOSI BIT5
#define TSS_ADXL_MISO BIT6

	// Port 3
#define TSS_TXRX_ON BIT2		// Enable RS485 communications
#define TSS_INTERRUPT BIT3
#define TSS_TAMPER BIT4

	// UART and SPI definitions
#define SPI_TX_BUSY (!(UCA1IFG & UCTXIFG))	// Bit location to monitor while transmission is in process
#define SPI_RX_BUSY (!(UCA1IFG & UCRXIFG)) 	// But location to monitor while receiving data is in process
#define SPI_BUSY (UCA1STATW & UCBUSY)	 	// Alternate bit location to monitor all spi activity
#define UART_TX_BUSY (!(UCA0IFG & UCTXIFG))		// Bit location to monitor UART transmission
#define RX_BUFF_EMPTY (!(UCA0IFG & UCRXIFG)) 	// But location to monitor UART rx buffer
#define UART_BUSY (UCA0STATW & UCBUSY)	 	// Alternate bit location to monitor all UART activity

//////////////////////////////////////////////////////////
// ADXL362 Sensor definitions
//////////////////////////////////////////////////////////

        // ADXL362 Instruction set

    #define ADXL_WRITE 0x0A			// Instruction to write to a register
    #define ADXL_READ  0x0B			// Instruction to read from a register
    #define ADXL_FIFO  0x0D			// Instruction to read the FIFO stack

        // ADXL362 Addresses
    #define	XDATA_LO 0x0E			// First data byte address -- Low byte of XData (can be read as 8 bytes: XLO, XHI, YLO, YHI, ZLO, ZHI, TEMPLO, TEMPHI)


    // ADXL PINS used with MSP430
    #define ADXL_MISO	BIT6
    #define ADXL_MOSI	BIT5
    #define ADXL_CLOCK	BIT4
    #define ADXL_SELECT BIT3		// Chip select for ADXL chip is P1.3
    #define ADXL_TAMPER	BIT1		// Tamper interconnect line returns from the sensor with 3.3V -- connected to P1.1
    #define ADXL_INT2	BIT0		// Interrupt input line for ADXL interrupt 2 is P1.0
        // The other pins were set using Grace and hopefully are not needed

    // Other defines used with ADXL362
    #define XYZ_PINS	0xC000		// 00 = x reading, 01 = y reading, 10 = z reading, 11 = temp reading
    #define X_READING	0x0000
    #define Y_READING 	0x4000
    #define Z_READING	0x8000
    #define T_READING 	0xC000
    #define NEG_MASK	0x3000		// These bits always indicate a negative reading.  Both should be set or unset
    #define ADXL_MASK	0x3FFF
    #define MAX16 		0xFFFF   	 // max unsigned int value

    //	//	//	//	//	ADXL362 Routines	//	//	//	//	//	//	//
    /*
        1. Set activity and inactivity thresholds and timers.
        	a. Write to Register 0x20 to Register 0x26.
        	b. To minimize false positive motion triggers, set the TIME_ACT register greater than 1.
        2. Configure activity and inactivity functions.
        	a. Write to Register 0x27.
        3. Configure FIFO.
        	a. Write to Register 0x28 and Register 0x29.
        4. Map interrupts.
        	a. Write to Register 0x2A and Register 0x2B.
        5. Configure general device settings.
        	a. Write to Register 0x2C.
        6. Turn measurement on.
        	a. Write to Register 0x2D.
        Settings for each of the registers vary based on application requirements.

        Table 11. Register Summary

        Reg		Name			Bits	 Bit 7	 Bit 6	  Bit 5		 Bit 4	   Bit 3		   Bit 2		    Bit 1	   Bit 0	Reset	RW
        0x00	DEVID_AD		[7:0]								DEVID_AD[7:0]													0xAD	R
    	0x01	DEVID_MST		[7:0]								DEVID_MST[7:0]													0x1D	R
    	0x02	PARTID			[7:0]								PARTID[7:0]														0xF2	R
    	0x03	REVID			[7:0]								REVID[7:0]														0x01	R
    	0x08	XDATA			[7:0]								XDATA[7:0]														0x00	R
    	0x09	YDATA			[7:0]								YDATA[7:0]														0x00	R
    	0x0A	ZDATA			[7:0]								ZDATA[7:0]														0x00	R
    	0x0B	STATUS			[7:0]  ERR_USER_ REGS|AWAKE  |INACT|ACT		|FIFO_OVER-RUN	|FIFO_WATER-MARK|FIFO_READY	|DATA_READY	0x40	R
    	0x0C	FIFO_ENTRIES_L	[7:0]												FIFO_ENTRIES_L[7:0]								0x00	R
    	0x0D	FIFO_ENTRIES_H	[7:0]									UNUSED								   |FIFO_ENTRIES_H[1:0]	0x00	R
    	0x0E	XDATA_L			[7:0]								XDATA_L[7:0]													0x00	R
    	0x0F	XDATA_H			[7:0]							SX			|						XDATA_H[3:0]					0x00	R
    	0x10	YDATA_L			[7:0]								YDATA_L[7:0]													0x00	R
    	0x11	YDATA_H			[7:0]							SX			|						YDATA_H[3:0]					0x00	R
    	0x12	ZDATA_L			[7:0]								ZDATA_L[7:0]													0x00	R
    	0x13	ZDATA_H			[7:0]							SX			|						ZDATA_H[3:0]					0x00	R
    	0x14	TEMP_L			[7:0]								TEMP_L[7:0]														0x00	R
    	0x15	TEMP_H			[7:0]							SX			|						TEMP_H[3:0]						0x00	R
    	0x20	THRESH_ACT_L	[7:0]								THRESH_ACT_L[7:0]												0x00	RW
        0x21	THRESH_ACT_H	[7:0]				UNUSED									| 			 THRESH_ACT_H[2:0]			0x00	RW
        0x22	TIME_ACT		[7:0]								TIME_ACT[7:0]													0x00	RW
        0x23	THRESH_INACT_L	[7:0]								THRESH_INACT_L[7:0]												0x00	RW
        0x24	THRESH_INACT_H	[7:0]				UNUSED									|  			THRESH_INACT_H[2:0]			0x00	RW
        0x25	TIME_INACT_L	[7:0]								TIME_INACT_L[7:0]												0x00	RW
        0x26	TIME_INACT_H	[7:0]								TIME_INACT_H[7:0]												0x00	RW
        0x27	ACT_INACT_CTL	[7:0]	RES					  | LINKLOOP 	|  INACT_REF 	|INACT_EN 		|ACT_REF 	|ACT_EN		0x00	RW
        0x28	FIFO_CONTROL	[7:0]				UNUSED					|	AH			|FIFO_TEMP		|		FIFO_MODE		0x00	RW
        0x29	FIFO_SAMPLES	[7:0]								FIFO_SAMPLES[7:0]												0x80	RW
        0x2A	INTMAP1			[7:0]	INT_LOW 	  |AWAKE  |INACT|ACT	|FIFO_OVER-RUN 	|FIFO_WATER-MARK|FIFO_READY	|DATA_READY	0x00	RW
        0x2B	INTMAP2			[7:0]	INT_LOW 	  |AWAKE  |INACT|ACT	|FIFO_OVER-RUN	|FIFO_WATER-MARK|FIFO_READY	|DATA_READY	0x00	RW
        0x2C	FILTER_CTL		[7:0]	     RANGE			  |RES  |HALF_BW|	EXT_SAMPLE	|		 		  ODR					0x13	RW
        0x2D	POWER_CTL		[7:0]	RES			  |EXT_CLK|  LOW_NOISE	|		WAKEUP	|	AUTOSLEEP	|	MEASURE				0x00	RW
    */
	// Refer to ADXL362 documentation for complete descriptions:
	//			http://dlnmh9ip6v2uc.cloudfront.net/datasheets/BreakoutBoards/ADXL362.pdf

    /*  ADXL362  Read and Write Register Commands
        The command structure for the read register and write register
        commands is as follows:
        </CS down> <command byte (0x0A or 0x0B)> <address
        byte> <data byte> <additional data bytes for multi-byte> …
        </CS up>
        The read and write register commands support multibyte
        (burst) read/write access.
        Reading from the FIFO buffer is a command structure that does not have an address.
    	</CS down> <command byte (0x0D)> <data byte> <data byte> … </CS up>
        */

    // ADXL bit definitions

       //	FIFO_CONTROL BITS
    #define FIFO_AH				BIT3 	// This bit is the MSB of the FIFO_SAMPLES register, allowing FIFO samples a range of 0 to 511 bytes.
    #define FIFO_TEMP_ON 		BIT2	// Store Temperature Data to FIFO. 1 = temperature data is stored in the FIFO together with x-, y-, and z-axis acceleration data.
    #define FIFO_OFF 			0x00	// BIT0 and BIT1 combinations to Enable FIFO and Mode Selection.
    #define FIFO_OLDEST_SAVED 	BIT0	// Only the oldest samples are saved.  Once the FIFO fills, nothing else is stored until the data is read, then it empties
    #define FIFO_STREAM			BIT1	// The FIFO is filled and then overwritten with new readings
    #define FIFO_TRIGGERED (BIT0 | BIT1)// b01 = Oldest saved mode.  b10 = Stream mode. b11=Triggered mode.

        //	FILTER CONTROL BITS (select one range, one BW, one ODR)
    #define RANGE_2G		0x00	// Default sensitivity setting of +/- 2g
    #define RANGE_4G		BIT6	// +/- 4G
    #define RANGE_8G		BIT7	// +/- 8G   (use only one range)
    #define HALF_BW			0x00	// Default setting of half bandwidth antialiasing filtering
    #define QUARTER_BW		BIT4	// More conservative antialiasing of quarter bandwidth
    #define EXT_SAMPLE 		BIT3	// External Sampling Trigger. 1 = the INT2 pin is used for external conversion timing control.

		// OUTPUT DATA RATES
        // Bits 0, 1 and 2  of FILTER_CTL select the ODR Output Data Rate.
		// Selects ODR and configures internal filters to a bandwidth using the QUARTER_BW or the HALF_BW bit setting.
    #define ODR_12			0x00	// 12.5 Hz ODR
    #define ODR_25	 		BIT0	// 25 Hz ODR
    #define ODR_50 			BIT1	// 50 Hz ODR (reset default)
    #define ODR_100    (BIT1 | BIT0)// 100 Hz ODR
    #define ODR_200	   		BIT2	// 200 Hz ODR
    #define ODR_400    (BIT2 | BIT0)// 400 Hz ODR
	#define SAMPLE_SPEED	ODR_50

        //	POWER CONTROL BITS
    #define EXT_CLK			BIT6 	// External Clock. 1 = the accelerometer runs off the external clock provided on the INT1 pin.
									// AN EXTERNAL CLOCK IS IS NOT SUPPORTED BY THIS SOFTWARE)
    #define INT_CLK 		0x00 	// External Clock. 0 = the accelerometer runs off internal clock (default)
    #define NORMAL_NOISE  	0x00	// Bits 4 and 5 select LOW_NOISE Selects Power vs. Noise Tradeoff: 00=Normal (default and lower power consumption)
    #define LOW_NOISE 	 	BIT4	// 01 = Low noise mode.
    #define ULTRA_LOW_NOISE BIT5	// 10=Ultralow noise mode.
    #define WAKEUP_MODE		BIT3	// Wake-Up Mode.  1 = the part operates in wake-up mode.
    #define AUTOSLEEP 		BIT2	// Autosleep. Activity and inactivity detection must be in linked mode or
        							// loop mode (LINK/LOOP bits in ACT_INACT_CTL register) to enable
        							// autosleep; otherwise, the bit is ignored. 1 = autosleep is enabled, and
        							//the device enters wake-up mode automatically upon detection of inactivity.
    #define ADXL_STANDBY	0x00	// Bit 0 and Bit1 Selects Measurement Mode or Standby. Start up the chip in Standby, and then switch to Measurement
    #define ADXL_ON			BIT1 	// Start recording data in measurement mode


    #define	THRESH_ACT_L	0x00 // low byte of the activity threshold (default is 0x00) The sensor only records data when over the threshold
    #define	THRESH_ACT_H	0x00 // high byte (lowest three bits) of the activity threshold (default is 0x00)
    #define	TIME_ACT		0x00 // When this timer is used, only sustained motion can trigger activity detection (default is 0x00)
    #define	THRESH_INACT_L	0x00 // low byte of inactivity threshold (default is 0x00) The sensor stops recording data when under the threshold
    #define	THRESH_INACT_H	0x00 // high byte (lowest three bits) of the inactivity threshold (default is 0x00)
    #define TIME_INACT_L	0x00 // low byte of inactivity timer (default is 0x00) The sensor goes to sleep after reading this many samples of inactivity
    #define	TIME_INACT_H	0x00 // high byte (lowest three bits) of the inactivity timer (default is 0x00)
    #define	ACT_INACT_CTL	0x00 // activity/inactivity control byte.  (default is 0x00 -- not activated)
    #define	FIFO_CONTROL	(FIFO_AH | FIFO_OLDEST_SAVED) // more than 256 bytes, and oldest save mode.  No temperature readings in the FIFO (once a record is enough)
    #define	FIFO_SAMPLES	244  //  510 FIFO samples  (3 X 170) // Along with AH, this is the number of two byte samples to save in the buffer (subtract 256 if AH is set).
        						 // Should be multiple of 3 unless FIFO_TEMP_ON is added to FIFO_CONTRO, then it should be a multiple of 4.  511 maximum.
    #define	INTMAP1	 		0x00 // no interrupts for now.
    #define	INTMAP2			0x00 // This interrupt is not currently used.  It can send interrupts to the MSP430, or used to sync a clock from the MSP430
    #define	FILTER_CTL 		(RANGE_2G | HALF_BW | SAMPLE_SPEED)	  		// 2g sensitivity, normal anti-aliasing and 100 Hz sample rate
    #define	POWER_CTL_OFF	(INT_CLK | ULTRA_LOW_NOISE | ADXL_STANDBY) // Startup in standby, and then change the ADXL_ON bit when ready
    #define	POWER_CTL_ON	(INT_CLK | ULTRA_LOW_NOISE | ADXL_ON)		 // The byte to send when ready to read data



//////////////////////////////////////////////////////////
// Start of FRAM data storage
//////////////////////////////////////////////////////////

#pragma SET_DATA_SECTION(".fram_vars")

		// DATA STORAGE DEFINITIONS
		// These are the usage for each byte in each record of the data[][] array
		// see SENSOR DATA immediately below
#define TEMPS 0
#define X_POS 1
#define Y_POS 2
#define Z_POS 3
#define READINGS 4
#define HIGHEST_BIN 5
#define BINS 6 			// The first bin stored


		// SENSOR DATA
		// each record is 32 bytes with 10 bins
unsigned int data[MAX_RECORDS][MAX_BIN + 6];
	// all the data in one place
	// data[][0] = data[][TEMPS] = the temperature reading at the completion of each record
	// data[][1] = data[][X_POS] = one (first) reading per record of absolute orientation x, y and z
	// data[][2] = data[][Y_POS]
	// data[][3] = data[][Z_POS]
	// data[][4] = data[][READINGS] = the total number of accumulated readings for each record
	// data[][5] = data[][HIGHEST_BIN] = the highest bin that is not zero.  This corresponds to the highest reading in the record.
	// 									This is also the highest in the record if the highest bin is bigger than MAX_BIN
	// 									if so, than low bins are dropped.
	// 									eg.  if this is 15 and MAX_BIN is 10, then bins below level 5 are dropped.
	// data[][6] = data[][BINS] = first bin count... and continuing to data[][15] = last bin count(highest bin)

		// SENSOR COUNTERS AND FLAGS
unsigned int recUploads = 0;				// The number of records that have been uploaded
unsigned int recCount = 0;					// The count of how many records have been collected since last upload
unsigned long int secCount = 0;				// seconds since last data upload
unsigned int recSecs = 0;					// The number of seconds that the current record has been active
unsigned char active = NO;					// "Yes" if the turtles are starting to move,  "No" if they are not
unsigned char first_start = YES;			// Only "Yes" the first time the sensor is used. Might not be needed
unsigned char command;						// The command in process
unsigned char error_code = NO_ERROR;				// see interrupt reporting routine for error codes

		// TEMPERATURE CALIBRATION SETTINGS
		// During Calibration, the reading at 0 degrees C becomes the temperature offset, and the ratio of 0x0200
		// divided by the reading at room temperature (minus the offset) becomes the temperature ratio.
signed int temperature_offset = TEMPERATURE_OFFSET;		// Default setting does not change the reading
unsigned int temperature_ratio = TEMPERATURE_RATIO;		// The temperature reading adjusted by the offset times this number will be divided by 0x1000,


		// NEST IDENTIFICATION and SENSOR ID
		// This is the data that results from a GPS read from the phone boards
		// The first 32 bytes returned by the AT$GPSACP command is stored just as received
		// This has the UTC time, Latitude and longitude of the reading
		// This is followed by the date of activation and serial number
		// See the top of the program to set the serial number.
unsigned char nest_ID[] = NEST_ID;			// The unique ID of each nest based on time, date, location and sensor ID
		// NEST_ID is defined as "122330.000,4542.8106N,01344.2720E,240613,AA001"
		// it breaks down as follows:
unsigned const char old_ID[] = SERIAL_NUMBER;	 // Store a copy of a SERIAL_NUMBER that cannot be altered
	#define NEST_ID_BYTES 47  	// the number of bytes in the nest_ID above
		// Starts with the UTC time, and GPS location copied directly from the GPS output.
		// The UTC is in the format: hhmmss.sss
	#define UTC 0				// nest_ID[UTC] - The offset for the GPS UTC time code
	#define UTC_BYTES 6			// The last byte is nest_ID[9], but the last 3 digits always seem to be zeros
		// LATITUDE is in the format: ddmm.mmmm N/S  where:
		//	dd - degrees 00..90
		//	mm.mmmm - minutes 00.0000..59.9999
		//  N/S: North / South
	#define LATITUDE 11			// nest_ID[LATITUDE] - The offset for the GPS latitude
	#define LATITUDE_BYTES 10 	// The last byte is nest_ID[20]
		// LOGITUDE is similar: dddmm.mmmm E/W   with the differences:
			// ddd - degrees 000..180
		// E/W: East / West
	#define LONGITUDE  22		// nest_ID[LONGITUDE] - The offset for the GPS longitude
	#define LONGITUDE_BYTES 11 	// The last byte is nest_ID[32]
		// The DATE is the date the sensor was activated in the format: ddmmyy  where:
		// dd - day 01..31
		// mm - month 01..12
		// yy - year 00..99 - 2000 to 2099
	#define DATE 34				// nest_ID[DATE] - The date the sensor was activated
	#define DATE_BYTES 6		// The last byte is nest_ID[39]
	#define DAY 34
	#define DAY_BYTES 2
	#define MONTH 36
	#define MONTH_BYTES 2
	#define YEAR 38
	#define YEAR_BYTES 2
		// The SERIAL NUMBER is a unique id for each Smart Sensor produced
		// The first letter is the hardware version
		// The second letter is the software version
		// The next 4 digits is a unique production number for each device
	#define SERIAL 41		// nest_ID[SERIAL_START] - The offset for the sensor serial number nest_ID[SERIAL_START]
	#define SERIAL_BYTES 6		// The last byte is nest_ID[46]



		// SMART SENSOR PARAMTETERS
		// These 25 bytes can be downloaded from the comm tower to change the behavior of the sensor.
unsigned char parameters[40] = {
	THRESH_ACT_L, THRESH_ACT_H, TIME_ACT, THRESH_INACT_L,
	THRESH_INACT_H, TIME_INACT_L, TIME_INACT_H,
	ACT_INACT_CTL, FIFO_CONTROL, FIFO_SAMPLES,
	INTMAP1, INTMAP2, FILTER_CTL, POWER_CTL_OFF, // parameters[0] to [13] - the first 14 are all the ADXL reset parameters.
	SLEEP_INTERVALS,				// parameters[14] - The number of clock interrupts between each FIFO buffer read.
	READ_SPEED,						// parameters[15] - The number of buffer reads per second (default is 1)
	SLOWBIN_LO, SLOWBIN_HI,			// parameters[16] and [17] - The number of seconds to accumulate readings in each record (set of bins)
	MAXRUN1L, MAXRUN2, MAXRUN3, MAXRUN4H,	// parameters[18] to [21] - The maximum time the sensor can run without reporting back to comm head.
	MAX_RECORDS_LO, MAX_RECORDS_HI,		// parameters[22] and [23] - The maximum number of records to collect
	CALIBRATE_TEMP,						// parameters[24] - Is the temperature calibration active (YES=calibrate)
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 };	// parameters[25] to [39] reserved for future use
#define PARAM_BYTES 40 // the number of bytes in the parameters structure above


unsigned int secMax = SLOWDAY_BIN_SEC;			 // The number of seconds to accumulate readings in each record (set of bins)
unsigned long int max_run_time = SECS_IN_DAY-60; // The maximum time the sensor can run without reporting back to comm head.
unsigned int maxRecords = MAX_RECORDS;			 // The maximum number of records to collect	 (can be changed at some future point)

unsigned const char copyleft[] = "TurtleSense Smart Sensor version 0.01 -- CC 4.0 BY-SA NerdsWithoutBorders.Net";

#pragma SET_DATA_SECTION()
// end of FRAM data section (~8K)



//////////////////////////////////////////////////////////
// SRAM VARIABLES
//////////////////////////////////////////////////////////

		// Non persistant variables -- these will be reset after a powerup
volatile unsigned int data_buffer[DATA_BUFFER_SIZE]; 	// SPI  data buffer (incoming data)

union Buffer
{
	volatile unsigned int bins[JOLT_BITS];		// Temporary storage for ADXL data until a record is completed and compressed.
												// 28 bins will be created, but only MAX_BIN of them will be stored in bins[].
												// All bins lower than the MAX_BIN highest will be discarded.
												// The bins increase exponentially by a factor of the square root of two (about 3db amplitude per bin)
	volatile unsigned char bytes[JOLT_BITS+JOLT_BITS]; //Temporary storage for UART data
	} buffer;

//volatile unsigned int temp_bins[JOLT_BITS] ;
volatile unsigned char starting_up = YES;		// "Yes" after a reset or power-up, "No" otherwise
volatile unsigned char force_shut_down = NO;	// Set to "yes" after a bad battery reading
volatile unsigned int adc_read;					// The data read in the adc interrupt routine
volatile signed int aXold=0;           			// previous acc value, to determine delta acc
volatile signed int aYold=0;
volatile signed int aZold=0;

		// Counters
unsigned int x;
unsigned int y;
unsigned int z;
unsigned int bin=0;         // bin counter



//////////////////////////////////////////////////////////
// START OF PROGRAM CODE
//////////////////////////////////////////////////////////

		//   SQUARE a number using MSP430FR57xx 16x16 signed Multiply
		//   hardware module.
		//   The calculation is automatically initiated after the second operand is
		//   loaded. Results are stored in RESLO and RESHI.

unsigned long int square(signed int operand1)
{
  unsigned long int result;
  MPYS = operand1;		// Load first operand - signed
  OP2 = operand1;		// Load second operand
  __no_operation();
  __no_operation();		// wait a tiny bit
  result = RESHI;
  return(result<<16 | RESLO);
}


		// WAIT a specified number of milliseconds (long int)
void wait(unsigned long int wait_millisecs)
{ 	unsigned int cycles = CLOCKSPEED;
		while (wait_millisecs > 0)
		{	for (cycles = CLOCKSPEED; cycles>0; cycles--) __delay_cycles(982) ;
			wait_millisecs-- ; // each count is a millisecond more or less
		}
}


		// Sleep for a set number of INTERVALS
void interval(unsigned char intervals)
{	while (intervals)
		{	__bis_SR_register(LPM3_bits + GIE); 		// Enter low power mode until the next tick of the clock
			intervals--;
		}
}


		// SLEEP for a specified number of seconds
void sleep(unsigned int secs)
{	while (secs)
		{	interval(8);		// The clock timer is set to wake up 8 times a second.
			secs-- ;
		}
}

void reset_SPI(void)
{	wait(1);
	UCA1CTL1 |= UCSWRST;			// Reset the SPI
	__no_operation();
	UCA1CTL1 &= ~UCSWRST; 			// confirm that SPI is enabled
	wait(1);						// wait a millisecond
}



		// RECEIVE a BYTE over the UART from the Comm board
char receive_byte(void)
{		char byte_rcvd;
		while (RX_BUFF_EMPTY) ;
		byte_rcvd = UCA0RXBUF;  //see what came in
		return (byte_rcvd);
}


		// READ data from the ADXL sensor FIFO stack
void ADXL_FifoRead(void)
		// Reading from the ADXL's FIFO buffer is a command structure that does not have an address.
		// </CS down> <command byte (0x0D)> <data byte> <data byte> … </CS up>
{	unsigned int temp;				// temporary storage for a read
	reset_SPI();				// confirm that SPI is enabled
	while(SPI_BUSY);			// Make sure no activity is happening
	P2OUT &= ~ADXL_SELECT;			// Select the chip by setting STE pin low (P2.3)
	__no_operation();			// let the ADXL settle
	while(SPI_TX_BUSY);			// wait for the TX buffer to empty
	UCA1TXBUF= ADXL_FIFO ;		// Prepare to transmit the data in the FIFO stack
	for( x = 2; x>0 ; x--) {	// Ignore the first two bytes, they look like garbage
		while(SPI_TX_BUSY) ;		// finish up transmit
		UCA1TXBUF= 0xFF;			// Send two dummy bytes to push the FIFO command out of the buffer and the first byte of data off the stack
		temp = UCA1RXBUF ;			// empty the recieved buffer
		}
	for( x = 0; x < DATA_BUFFER_SIZE-1; x++ ) {		// Fill up the buffer with the data from the stack.  The first is junk
		while(SPI_TX_BUSY) ;		// wait for the TX buffer to empty
		UCA1TXBUF= 0xFF;			// Send another dummy byte to push data off the stack
		data_buffer[x] = UCA1RXBUF;	// get the byte from the previous send
		while(SPI_TX_BUSY) ;		// wait for the TX buffer to empty
		UCA1TXBUF= 0xFF;			// Send another dummy byte to push data off the stack
		temp = UCA1RXBUF ;
		data_buffer[x] |= (temp<<8) ;	//  add the second byte * 256
		}
	while(SPI_TX_BUSY) ;	// wait for SPI module to finish up
	temp = UCA1RXBUF;	// get the last byte
	P2OUT |= ADXL_SELECT;			// deselect

}


		// PROCESS the DATA read in from the ADXL FIFO stack
		// and prepare bin histogram
void Process_data(void)
{
	unsigned long int bin_mask;
	char bin_count;
	char not_first_read = NO;
	signed int tempX, tempY, tempZ;    	 	// temp values
	signed int aX, aY, aZ; 					// value of accelerometer for each coordinate
	signed long int dX, dY, dZ;				// difference values for each coordinate
	signed long int mag_squared;	 		// absolute magnitude of vector squared (dX^2 + dY^2 +dZ^2)
			//Skip over bad data and synch up with the good data
	for (x=0; x< DATA_BUFFER_SIZE-3 && !(   ((data_buffer[x] & X_READING) == X_READING) &&
											((data_buffer[x+1] & Y_READING) == Y_READING) &&
											((data_buffer[x+2] & Z_READING) == Z_READING)  );	x++ );
	while (x<DATA_BUFFER_SIZE)
	{			// process good data to find the "jerk" (third derivative of displacement)
		tempX=data_buffer[x];
		tempY=data_buffer[x+1];
		tempZ=data_buffer[x+2];
		if (	((tempX & XYZ_PINS) == X_READING) &&
				((tempY & XYZ_PINS) == Y_READING) &&
				((tempZ & XYZ_PINS) == Z_READING) )		// process only non-zero readings
		{		 // make the data proper 16 bit integers
			aX= (tempX & ADXL_MASK) | ((tempX & NEG_MASK)<<2);  // mask off the top 2 bits and copy the sign bits there
			aY= (tempY & ADXL_MASK) | ((tempY & NEG_MASK)<<2);
			aZ= (tempZ & ADXL_MASK) | ((tempZ & NEG_MASK)<<2);
			if ( not_first_read )		// process the result into the bins
			{		// skip the first read because the delta will be very high
					// analyzing the square of delta ACCELERATION (turns a 12 bit signed data into an unsigned 25 bit number)
				dX= (aX - aXold);				// compute the differences
				dY= (aY - aYold);
				dZ= (aZ - aZold);
				mag_squared = square(dX) + square(dY) + square(dZ);  // sum the square of the differences
					// bin 29 (28 counting from bin 0) is the highest bit possible (but very unlikely)
				bin_mask = JOLT_MASK;   		// Start looking at the highest bit possible
				for (bin_count=JOLT_BITS-1;	(bin_count) && ((bin_mask & mag_squared) == 0); bin_count--)
					//mask off one bit and see if it is zero
					bin_mask >>= 1 ;			// if so move the mask over one bit to the right and look at the next bin
				buffer.bins[bin_count]++ ;    	// increment the appropriate bin
				data[recCount][READINGS]++ ;	// increment the number of readings
			}
			else
			{	not_first_read = TRUE ;
						// Save the first reading off the stack in the orientation log
			// TODO // make this an average reading
				data[recCount][X_POS] = aX;
				data[recCount][Y_POS] = aY;
				data[recCount][Z_POS] = aZ;
			}			// save the reading for next time in aXold, aYold, and aZold
			aXold = aX;
			aYold = aY;
			aZold = aZ;
		}
		x = x+3;		// increment the counter to look at the next readings
	}
}


			// CLEAR THE TEMPORARY BINS
			// deletes all data from the temporary bins after they have been saved as a record
void clear_temp_bins(void)
{	for (bin =  0; bin < JOLT_BITS; bin++) buffer.bins[bin] = 0;
}


			// TRANSFER A STRING
			// sends or receives a character array of known length using the UART
			// 1st parameter is the pointer to the array, the second is the length of the array
			// the third is whether sending or receiving 0 = RECEIVE, 1 = SEND
			// two more bytes are received to indicate the starting offset in the array
			// and how many bytes to transfer (255 maximum)
void transfer_string(unsigned char *string_ptr, unsigned char string_cnt, unsigned char sending)
{	unsigned char temp_count, start_byte, bytes2move;	// counters
	while (RX_BUFF_EMPTY) ;				// wait for a full buffer
	start_byte = UCA0RXBUF;				// the next byte received is the starting byte
	for (temp_count = 0; temp_count < start_byte; temp_count++) string_ptr++;	// point to the starting byte
	while (RX_BUFF_EMPTY) ;				// wait for a full buffer
	bytes2move = UCA0RXBUF;				// the next byte received is how may bytes will move
	if (sending)
	{	while (bytes2move--)			// count through all of them
		{	while (UART_TX_BUSY);		//wait for an empty buffer
			UCA0TXBUF= *string_ptr;		// send a byte
			string_ptr++;				// increment the pointer
		}
		return;
	}
	for (; bytes2move; bytes2move--)	// count through all of them
	{	while (RX_BUFF_EMPTY) ;			// wait for a full buffer
		*string_ptr = UCA0RXBUF; 		// get a byte
		string_ptr++;					// increment the pointer
	}
}


			// SEND A RECORD
			// send a record over the UART to the comm head
void send_a_record(int rec2send)
{	unsigned int temp = 0;						// used to temporarily hold the data we're sending
	unsigned int data_count = 0 ;				// count how many bytes we'll send
	while (data_count <  DATA_SIZE)			// we'll send 16 integers
	{	temp=data[rec2send][data_count]; 	// temporarily store the data to send
		while(UART_TX_BUSY);				// wait for an empty buffer
		UCA0TXBUF= temp>>8 ;				// send the high byte first
		while(UART_TX_BUSY);				// wait for an empty buffer
		UCA0TXBUF = temp & 0x00FF;			// send the low byte
		data_count++;						// increment data count
	}
}


////////////// SMART SENSOR COMMANDS////////////////////////////////////////////////////


////////////// Command 0x00 -- cycles the ADXL sensor OFF
			// This is a hardware reset of the ADXL by turning off all power to the chip
			// The chip starts up with its default values in standby
void ADXL_off(void)
{	unsigned char p2save[6];  // save the settings for Port 2
		// Save the settings for P2
	p2save[0] = P2OUT;	// Port 2 Port Select Register 1
	p2save[1] =	P2SEL1;	// Port 2 Direction Register
	p2save[2] = P2DIR;	// Por;t 2 Resistor Enable Register */
		// Stop sending power to any of the SPI pins by setting the appropriate pins low on P2
		// The ADXL chip can only be reset if there is no voltage on ANY pin.
	P2OUT = 0;		// Port 2 Port Select Register 1
	P2SEL1 = 0; 	// Port 2 Port Select Register 1
	P2DIR = 0xFF;	// Port 2 Direction Register (all outputs)
		// Turn off power to the ADXL and reset it
	P1OUT &= ~TSS_ADXL_POWER;			// Setting P1.2 low turns off the sensor
	wait(20);							//  Wait 20 msec.
		//This is probably much more than necessary as the power goes to zero in about 1 ms.
		//turn the ADXL back on
	P1OUT |= TSS_ADXL_POWER ;			// turn it on by setting the bit high (P1.2)
	wait(10);						// wait 10 ms (sensor spec'd for 5)
			// Restore Port 2 settings
	P2OUT = p2save[0];	// Port 2 Port Select Register 1 */
	P2SEL1 = p2save[1];	// Port 2 Direction Register */
	P2DIR = p2save[2];	// Port 2 Resistor Enable Register */

}


////////////// Command 0x01 -- turns the ADXL sensor ON
			// This takes the sensor out of standby
			// and puts it in run mode.  Checks that it is actually on.
void ADXL_on (void)
{ 	unsigned char test_byte = 0;
	for (x= 0; x <10; x++)	// try ten times
	{		// Prepare the SPI
		reset_SPI();				// confirm that SPI is enabled
		P2OUT &= ~ADXL_SELECT;			// Select the chip by setting STE pin low (P2.3)
			// Write the instruction to the ADXL to turn on
		while(SPI_TX_BUSY);
		UCA1TXBUF = ADXL_WRITE;		// Tell the ADXL to transmit the data in the FIFO stack
		while(SPI_TX_BUSY);
		UCA1TXBUF = 0x2D;			// send the address of the POWER_CTL byte
		while(SPI_TX_BUSY);
		UCA1TXBUF = parameters[13] | ADXL_ON;	// send the instructions for the POWER_CTL byte to turn on
		while(SPI_BUSY);			// wait for it to finish transmitting
			// Reset the SPI
		P2OUT |= ADXL_SELECT;			// deselect
		wait(1);
		P2OUT &= ~ADXL_SELECT;			// Select the chip by setting STE pin low (P2.3)
			// Confirm that the ADXL is on
		while(SPI_TX_BUSY);
		UCA1TXBUF = ADXL_READ;			// Tell the ADXL to transmit the data in a register
		while(SPI_TX_BUSY);
		UCA1TXBUF =  0x2D;				// send the address of the POWER_CTL byte
		while(SPI_TX_BUSY);
		UCA1TXBUF =  0xFF;				// send a dummy byte to receive a byte
		while(SPI_BUSY);
		test_byte = UCA1RXBUF;			// read what came in
		P2OUT |= ADXL_SELECT;			// deselect
		if (test_byte == parameters[13] | ADXL_ON) return;
	}	 // Time out after ten tries and report a problem with the sensor
	error_code = ERROR_SENSOR_TIMEOUT;
}

////////////// Command 0x02 -- RESET the ADXL sensor
			// Software and hardware reset of the ADXL sensor
			// returns an error code of 1 if the sensor does not turn on after 5 tries
char ADXL_reset(void)
{
	unsigned char reset_tries = 5;
	unsigned char ADXL_not_operational = TRUE;
	unsigned char test_byte = 0;

	error_code = NO_ERROR;			// reset any previous error code
			// keep resetting until things look good
	while ( ADXL_not_operational)
	{  // TODO // watchdog timer for timing out.
		while (SPI_BUSY);				// check to see if any other device is using the SPI line.
				// Turn off power to the ADXL and reset it
		ADXL_off();
		reset_SPI();				// confirm that SPI is enabled

					// do a software reset of the ADXL sensor
					// ADXL SOFT RESET REGISTER Address: 0x1F, Name: SOFT_RESET
					//	Writing Code 0x52 (representing the letter, R, in ASCII or
					//	unicode) to this register immediately resets the ADXL362. All
					//	register settings are cleared, and the sensor is placed in standby.
					//	Interrupt pins are configured to a high output impedance mode
					//	and held to a valid state by bus keepers.
					//	This is a write-only register. If read, data in it is always 0x00.
		P2OUT &= ~ADXL_SELECT;			// Select the chip by setting STE pin low (P2.3)
		while(SPI_TX_BUSY);				// wait for the SPI to finish
		UCA1TXBUF = ADXL_WRITE;			// Tell the ADXL to write to its registers
		while(SPI_TX_BUSY);				// wait for the SPI to finish
		UCA1TXBUF= 0x1F;				// Send the address of the soft reset register
		while(SPI_TX_BUSY);				// wait for the SPI to finish
		UCA1TXBUF = 0x52;				// send the reset code
		while(SPI_BUSY);				// wait for the last transmission to end
		P2OUT |= ADXL_SELECT;

				// initialize all the settings in standby mode
		reset_SPI();				// confirm that SPI is enabled
		P2OUT &= ~ADXL_SELECT;			// Select the chip by setting STE pin low (P2.3)
		while(SPI_TX_BUSY);				// wait for the SPI to finish
		UCA1TXBUF = ADXL_WRITE;			// Tell the ADXL to write to its registers
		while(SPI_TX_BUSY);				// wait for the SPI to finish
		UCA1TXBUF= 0x20;				// Send the address of the first register (the ADXL auto increments after that)
		for( x = 0; x < 13; x++ ) {		// Send the first 13 parameters
			while(SPI_TX_BUSY);
			UCA1TXBUF= parameters[x] ;	// Send the setting to the register
			}
		while(SPI_TX_BUSY);
		UCA1TXBUF= parameters[13] & ~ADXL_STANDBY ;	// Send the POWER_CTL setting to the register but make sure it is in standby
		while(SPI_BUSY);				// wait for the SPI to finish
		P2OUT |= ADXL_SELECT;			// deselect the ADXL
		wait(10);						// wait 10 ms for the chip to wake up before taking readings (spec'd for 5)

				// Read back the FIFO register to see if it was set correctly.  If so, we can read and write to the sensor.

		reset_SPI();				// confirm that SPI is enabled
		P2OUT &= ~ADXL_SELECT;			// Select the chip by setting STE pin low (P2.3)
		while(SPI_TX_BUSY);
		UCA1TXBUF = ADXL_READ;			// Tell the ADXL to transmit the data in a register
		while(SPI_TX_BUSY);
		UCA1TXBUF =  0x29;				// send the address of the FIFO_SAMPLES byte
		while(SPI_TX_BUSY);
		UCA1TXBUF =  0xFF;				// send a dummy byte to receive a byte
		while(SPI_BUSY);
		test_byte = UCA1RXBUF;			// read what came in
		P2OUT |= ADXL_SELECT;			// deselect the ADXL
		wait(1);						// wait 1 ms
		if (test_byte == parameters[9]) ADXL_not_operational = FALSE;	// If is what we wrote to the register all is well
		else sleep(1);					// wait a second before trying again
		reset_tries--;
		if (!reset_tries) error_code = ERROR_SENSOR_TIMEOUT;     	//TODO// If we time out, there's a bug and/or we need to report by phone of a malfunction
	}
	return (error_code);
}


////////////// Command 0x03 -- Put the MSP430 TO SLEEP
			// Puts the microprocessor in a very low power mode and wakes it up
			// if it gets an interrupt from the ADXL sensor or the master board.
			// If the ADXL sensor has been turned off, then only an interrupt
			// from the master board on pin 3.3 or a power reset will wake it up
void msp2sleep(void)
{
	__bis_SR_register(LPM4_bits + GIE); 		// Enter low power mode until an interrupt wakes up the microprocessor
}


////////////// Command 0x04 -- CALIBRATE
			// Stores calibration data sent by the comm tower
			// Most of this process is controlled by the set-up board:
			// First a register check is done to upload the sensor settings to the set-up board
			// The settings are saved for later.  New settings are downloaded to stream temperature data
			// Buttons on the set-up board record the low temperature (an ice water bath at 0 degrees C)
			// and room temperature.  The reading at 0 degrees C becomes the temperature offset, and the ratio of 0x0200
			// divided by the reading at room temperature (minus the offset) becomes the temperature ratio.
			// This Calibrate command downloads the temperature ratio and the temperature offset to the
			// smart sensor and stores it for calibrating the temperature readings made by the sensor.
			// To simplify the math, the ratio is a two byte integer, which can be multiplied by the two byte
			// temperature reading after the offset is subtracted.  The result, shifted 12 bits right
			// can then be stored as the temperature reading:  0x0000  is zero degrees C;  0x0080 is 5 degrees C;
			// 0x0100 is ten degrees C;  0x0200 is twenty degrees C; etc...  Negative readings are possible.

void calibrate(void)
{	temperature_offset = receive_byte();	// get the first byte of the offset
	temperature_offset <<=8;				// make it the high byte
	temperature_offset += receive_byte();	// get the second byte
	temperature_ratio = receive_byte();		// get the first byte of the offset
	temperature_ratio <<=8;					// make it the high byte
	temperature_ratio += receive_byte();	// get the second byte
}


////////////// Command 0x05 -- REGISTER Smart Sensor
			// Stores new registration data from comm tower (installation, time and GPS location)
			// Same logic as new_paramters() below.
void register_SS(void)
{	transfer_string(nest_ID, NEST_ID_BYTES, RECEIVE);
}


////////////// Command 0x06 -- Load NEW PARAMTERS
			// Downloads new parameters from comm tower and stores them
			// This includes: ADXL register settings
			//				  Record timings, and Record quantities
			// The format is first send the starting offset into the array, and how many bytes will be transferred.
			// followed by the value of the parameters.  Changing all 25 bytes of the parameters would require sending
			// 27 total.  It would start with a zero, and then 25 and then the parameters in order.
			// Changing just a single parameter byte would start with the offset into the array, then a one,
			// and than the value of the parameter.  Some parameters are more than one byte
			// so care must be taken to make sure all the bytes are sent.  The first parameter is number zero.
			// An error is generated if the offset plus the number of bytes goes out of range.
void new_parameters(void)
{	unsigned char offset;
	transfer_string(parameters, PARAM_BYTES, RECEIVE);

			// create integers out of parameter bytes (they might have changed)
	secMax =  (parameters[17]<<8) | parameters[16] ;	// The number of seconds to accumulate readings in each record (set of bins)
	max_run_time = parameters[21];
	for(offset=20; offset>17; offset--)
	{	max_run_time <<= 8;
		max_run_time |= parameters[offset]; 			// turn 4 unsigned chars into a long int
	}
	maxRecords = (parameters[23]<<8) | parameters[22];	// The maximum number of records to collect
	ADXL_off();							//  turn the sensor off!
	error_code = ADXL_reset();			// reset the ADXL and load the new parameters
}


////////////// Command 0x07 -- CLEAR last RECORD
			// Erases the last record and lowers the count
			// Also used by clear_all_records() to erase all the records
void clear_record(void)
{	if (recCount)		// Check that there is a record to erase!
	{	recCount--;		// Decrement the count of how many records have been collected since last upload
		for (z = 0; z < DATA_SIZE; z++) data[recCount][z] = 0;  // erase the record
		recSecs = 0;	// Reset the number of seconds that the current record has been active
	}
}


////////////// Command 0x08 -- CLEAR ALL the RECORDS
			// erase all data and resets the counts
void clear_all_records(void)
{	recCount = MAX_RECORDS;			// Make sure they are all errased
	while(recCount) clear_record(); // clear them all until they are all empty
	recUploads = 0;					// The number of records that have been uploaded
	secCount = 0;					// The number of seconds since last upload
	clear_temp_bins();				// Erase these too.
}


////////////// Command 0x09 -- SEND LAST RECORD
			// sends the most recent record created (the current recCount)
void send_last_record(void)
{		while (UART_TX_BUSY);		//wait for an empty buffer
		UCA0TXBUF= DATA_BYTES;		// we'll send 32 bytes
		send_a_record(recCount);	// send the data
}


////////////// Command 0x0A -- SEND ALL RECORDS
			// sends all the records in memory
void send_all_records(void)
{
	unsigned int temp_count=0;			// counts how many records have been sent
	while (UART_TX_BUSY);				//wait for an empty buffer
	UCA0TXBUF= recCount >>8;			// we'll send the high byte of the number of records
	while (UART_TX_BUSY);				//wait for an empty buffer
	UCA0TXBUF= recCount & 0x00FF;		// we'll send the low byte of the number of records
	while (UART_TX_BUSY);				//wait for an empty buffer
	UCA0TXBUF= DATA_BYTES;				// we'll send 32 bytes in each record
	while (temp_count < recCount) {		// until we've sent them all
		send_a_record(temp_count);		// send another record
		temp_count++;					// increment the counter
	}
}


////////////// Command 0x0B -- MANUAL OPERATION
			// Manually reads or writes directly with the ADXL sensor without interpretation
			// Parameters for ADXL affect the result
			// can be used to change parameters or read from the X, Y, Z and Temp registers
			// This routine does not read from the ADXL FIFO buffer
			// use command 0x0C for that.
			// To use this, the read or write ADXL command should be sent, followed by the starting address of
			// the ADXL register, and then the number of bytes to read or write using SPI
void manual_operation(void)
{	unsigned char command, address, byte_count, count;
	reset_SPI();					// confirm that SPI is enabled
	command = receive_byte();		// read, write but don't stream
	address = receive_byte();		// the address we will read from or write to
	byte_count= receive_byte();
	if ((command != ADXL_READ) && (command != ADXL_WRITE)) // it can only be a read or write
	{	error_code = ERROR_OUT_OF_RANGE;
		for (count = 0; count <byte_count; count++)
		{	if (command == ADXL_READ)			// reading from the sensor means writing to the comm tower
			{	while(UART_TX_BUSY);		// wait for an empty buffer
				UCA0TXBUF = 0x0000;			// send zeros
			}
			else
			{address = receive_byte();	// receive a byte but don't do anything with it
			}
			count--;					// decrement the counter
		}
		return;
	}
		// if things are in range we can continue
	if (command == ADXL_WRITE)		// If we're writing data we have to receive it first
		// read from the UART and write to the SPI
	{	for (count = 0; count <byte_count; count++)	buffer.bytes[count] = receive_byte();
			// we'll count off the bytes we were told to receive,  get a byte and store it in the buffer
		P2OUT &= ~ADXL_SELECT;		// Select the chip by setting STE pin low (P2.3)
			// Write the instruction to the ADXL to turn on
		while(SPI_BUSY);			// Make sure no activity is happening
		UCA1TXBUF = ADXL_WRITE;		// Tell the ADXL to read or write data
		while(SPI_TX_BUSY)
		UCA1TXBUF = address;		// send the start address of the register
		for (count = 0; count <byte_count; count++)
			{	while(SPI_TX_BUSY);
				UCA1TXBUF = buffer.bytes[count];	// send the data received
			}
		while(SPI_BUSY);			// wait for it to finish transmitting
		P2OUT |= ADXL_SELECT;		// deselect
		return;
	}
		//otherwise we are reading from the SPI and writing to the UART
	reset_SPI();					// confirm that SPI is enabled
	P2OUT &= ~ADXL_SELECT;			// Select the chip by setting STE pin low (P2.3)
	__no_operation();				// let the ADXL settle
	while(SPI_TX_BUSY);				// wait for the TX buffer to empty
	UCA1TXBUF = ADXL_READ;			// Tell the ADXL to transmit the data in a register
	while(SPI_TX_BUSY);
	UCA1TXBUF= address ;			// Send the address of the first read locationn
	for (count = 0; count <byte_count; count++)
	{		// Fill up the buffer with the data from the ADXL
		while(SPI_TX_BUSY) ;		// wait for the TX buffer to empty
		UCA1TXBUF= 0xFF;			// Send another dummy byte to push data off the stack
		buffer.bytes[count] = UCA1RXBUF;	// get the byte from the previous send
		}
	while(SPI_TX_BUSY) ;			// wait for SPI module to finish up
	address = UCA1RXBUF;			// get the last byte but ignore it
	P2OUT |= ADXL_SELECT;			// deselect
	for (count = 0; count <byte_count; count++)
	{	while(UART_TX_BUSY);		// wait for an empty buffer
		UCA0TXBUF = buffer.bytes[count];	// send the data in the buffer
	}
}



////////////// Command 0x0C -- Stream data
			// Sends out sensor data as soon as it is read off the ADXL FIFO stack
			// Parameters for ADXL affect how data is streamed and should be set before calling this routine.
			// This routine does not read the X,Y, Z and Temp registers.
			// The ADXL chip must be reset and turned on before using this routine.
void stream_data(void)
{	unsigned int temp = 0;		// used to temporarily hold the data we're sending
	unsigned char reads;
	reset_SPI();				// confirm that SPI is enabled
	while (P3IN & TSS_TXRX_ON)  // Continue streaming until the comm tower turns off the transceivers
	{	P2OUT &= ~TSS_UCA0TXD;	// start the transmit line low.
		for (reads = parameters[15]; reads > 0 ; reads-- )
		{	interval(parameters[14]); 					// Enter low power mode 3 for a fraction of a second (the loop starts up again every 1/8 second -- restored by timer interupt)
			if (P3IN & TSS_TXRX_ON) ADXL_FifoRead();	// If the tranceivers are on, read the data off the ADXL FIFO stack
			if (P3IN & TSS_TXRX_ON) 					// If the transceivers are on, send back data
			{	// TODO // this will need modification if the buffer size can vary.
				for( x = 0; x < DATA_BUFFER_SIZE-1; x++ ) 	// we'll send the entire buffer
				{	temp=data_buffer[x]; 				// temporarily store the data to send
					while(UART_TX_BUSY);				// wait for an empty buffer
					UCA0TXBUF= temp>>8 ;				// send the high byte first
					while(UART_TX_BUSY);				// wait for an empty buffer
					UCA0TXBUF = temp & 0x00FF;			// send the low byte
				}
			}
		}
	}
}														// this should continue until the transceivers are turned off


////////////// Command 0x0D -- CHECK ID
			// uploads the current nest ID registration to the comm tower
			// The send_string rountine receives two more bytes,
			// the starting byte and how many bytes are requested
			// first byte sent is the number of bytes being sent,
			// then they are sent in order.  The number of bytes being sent will be zero
			// if the starting byte and/or number of bytes requested is out of range.
void check_ID(void)
{	transfer_string(nest_ID, NEST_ID_BYTES, SEND);
}


////////////// Command 0x0E -- SEND PARAMETERS
			// Uploads current parameters and counts  to comm tower
			// The send_string rountine receives two more bytes,
			// the starting byte and how many bytes are requested
			// first byte sent is the number of bytes being sent,
			// then they are sent in order.  The number of bytes being sent will be zero
			// if the starting byte and/or number of bytes requested is out of range.
void send_parameters(void)
{	transfer_string(parameters, PARAM_BYTES, SEND);
}


////////////// Command 0x0F -- SEND a PROGRESS REPORT (error_code)
			// Sends a one byte code that explains the reason for status of the smart sensor
			// The report is two bytes.  The first one is the last command sent, and
			// the second is the last status/error code.
			// All codes less than 0x80 are not errors.
			// 0x00 = No interrupt was generated -- operation in progress
			// 0x01 = INT1 generated an interrupt
			// 0x02 = INT2 generated an interrupt
			// 0x04 = operation completed successfully, data ready to report
			// 0x08 = operation completed successfully, no data to report
			// 0x10 = normal start-up
			// 0x20 = command received, operation underway, communication over
			// >= 0x80 = an error condition exists.  The error code is returned:
			// 0x81	= Sensor Start up failure (timed out)
			// 0x82 = Unknown command received
			// 0x84 = Bad Data received
			// 0x88 = Data transmission timed out
			// 0x90 = Parameters out of range
			// 0xA0 = Unknown Start up error

void send_progress_report(void)
{	UCA0CTL1 |= UCSWRST;			// Reset the UART
	__no_operation();
	UCA0CTL1 &= ~UCSWRST; 			// confirm that UART is enabled
	wait(1);
	while(UART_TX_BUSY);				// wait for an empty buffer
	UCA0TXBUF= command ;				// send the code
	wait(1);
	while(UART_TX_BUSY);				// wait for an empty buffer
	UCA0TXBUF= error_code ;				// send the code
}


////////////// Command 0x10 -- Resume Run and Process data in bins
			// resets the sensor and gets things going
void resume_run_bins(void)
{	volatile unsigned int temp, temp1, temp2;
	volatile unsigned char reads, out_of_time = 0;
	volatile long int temp_temp; // temporary temperature reading

				// Continue until it is time to stop
	while ((recCount < maxRecords) && (!out_of_time))  // continue if there is room for more and still time
	{							// recCount goes from 0 to MAXRECORDS minus 1
		P2OUT &= ~TSS_UCA0TXD;
		for (reads = parameters[15]; reads > 0 ; reads-- )
		{	interval(parameters[14]); 		// Enter low power mode 3 for a fraction of a second (the loop starts up again every 1/8 second -- restored by timer interupt)
			ADXL_FifoRead();		// Read the data off the ADXL FIFO stack
			Process_data();			// process the data and store in buffer.bins array
		}

				// increment counters
		recSecs++;					// Increment the number of seconds in the current record
		secCount++;					// Increment the total number of seconds since last transmission
		out_of_time = (secCount > max_run_time);
		if ((recSecs >= secMax) || out_of_time)	// Check to see if it is time to start a new record or time has run out

				// Process and store the temporary bins in the record.
		{	bin = JOLT_BITS - 1;	// Start with the bin with the highest readings
			while( (bin>0) && (buffer.bins[bin] == 0) ) bin =  bin - 1 ; // Find the highest bin with a non zero count in it
			data[recCount][HIGHEST_BIN]= bin + 1;	// Save it in the highest_bin array (1-25)
			if (bin < MAX_BIN-1) bin = MAX_BIN-1;	// is room for all the bins with non-zero data if not, we'll drop the lowest ones
			z = DATA_SIZE;   						// Counter for the real data storage bins in the record
			while (z > BINS)
			{	// transfer the temporary data into the record
				z =  z - 1;								// the range of the array is MAX_BIN - 1 to zero.
				data[recCount][z] = buffer.bins[bin];		// move the temp bin to the array.
				bin = bin - 1 ;							// fill all of them.
			}
			clear_temp_bins();	// reset the buffer.bins for the next record

				// Get a temperature reading
			reset_SPI();						// confirm that SPI is enabled
			temp_temp = 0; 						// start out with zero
			for (z=8; z>0; z--)					// This is looped 8 times for an average reading
			{	P2OUT &= ~ADXL_SELECT;			// Select the chip by setting STE pin low (P2.3)
				UCA1TXBUF = ADXL_READ;			// Tell the ADXL to transmit the data in a register
				while(SPI_TX_BUSY);				// wait for the TX buffer to empty
				UCA1TXBUF =  0x14;				// send the address of the temperature byte
				while(SPI_TX_BUSY) ;			// wait for the TX buffer to empty
				UCA1TXBUF= 0xFF;				// Send a dummy byte to push the address out of the buffer
				while(SPI_TX_BUSY) ;			// wait for the TX buffer to empty
				UCA1TXBUF= 0xFF;				// Send another dummy byte to push data off the stack
				temp = UCA1RXBUF;				// this byte is junk -- clear the buffer
				while(SPI_RX_BUSY);				// wait for the TX buffer to empty
				UCA1TXBUF= 0xFF;				// Send another dummy byte to push more data off the stack
				temp1 = UCA1RXBUF;				// get the low byte of temperature byte from the previous send
				while (SPI_RX_BUSY)				// get the second byte received
				temp2 = UCA1RXBUF;				// get the low byte of temperature byte from the previous send
				temp_temp += (temp2<<8)|temp1 ;	// add the second byte * 256
				P2OUT |= ADXL_SELECT;			// deselect
				wait(1);
			}
			data[recCount][TEMPS] = temp_temp>>3;	// divide 8 accumulated readings by 8
			if (parameters[24])						// check to see if calibration is on
			{		// use calibration data to adjust the temperature reading
				  MPYS = temp = data[recCount][TEMPS] - temperature_offset ;	// Load first operand - signed reading
				  OP2 = temperature_ratio;										// Load second operand - unsigned ratio
				  __no_operation();
				  __no_operation();		// wait a tiny bit for the multiplication to finish
				  data[recCount][TEMPS] = (temp & 0x8000) |RESHI<<4 | RESLO>>12 ; 	// divide result by 0x01000 and restore sign
			}

				// prepare for a new record
			recSecs = 0 ;			// reset the second counter
			recCount++ ;			// and add a new record

		}
	}
}


////////////// Command 0x11 -- START a new RUN and Process data in BINS
			// resets the sensor and gets things going
void start_run_bins(void)
{	clear_all_records();	// Reset the memory
	if (ADXL_reset()) return; // Reset and initialize the ADXL
	ADXL_on();				// Turn the sensor on
	sleep(1);				// Wait a second
	clear_temp_bins();		// Reset the temporary bins
	ADXL_FifoRead();		// Read the FIFO stack to empty it of initial bad reads (we lose a second of data)
	recCount = 0;			// Reset counts
	secCount = 1;			// Reset the number of seconds we have been running
	data[0][READINGS] = 0;	// Reset the number of readings to zero
	if (error_code & 0x80) return;	// if there is an error, don't go any further
	resume_run_bins();				// start 'er up.
}


/////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////// MAIN ROUTINE ////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////

void main(void)
{	unsigned char last_error=0;
	Grace_init();                   // Activate Grace-generated configuration without interrupts sent to the comm tower
	UCA0CTL1 |= UCSWRST;			// Reset the UART
	wait(1);
	UCA0CTL1 &= ~UCSWRST; 			// confirm that UART is enabled
	ADXL_off();						// reset the ADXL sensor so it doesn't get in a high power funk
	error_code = NO_ERROR_START_UP;	// report the current condition
	P3OUT |= TSS_INTERRUPT;			// Tell the comm head to wake up because we are ready for communication
	while (!(P3IN & TSS_TXRX_ON)) ;	// Wait for the RS485 chip to turn on
	P3OUT &= ~TSS_INTERRUPT;		// reset the interrupt once it is acknowledged
	while (RX_BUFF_EMPTY) ;
	command = UCA0RXBUF;  			//see what came in
	if (command != 0x0F) error_code = ERROR_START_UP;			// The first communication should be an interrupt request
	send_progress_report();
	while (TRUE)
	{	P3OUT |= TSS_INTERRUPT;			// Tell the comm head that we are ready for communication
		while (!(P3IN & TSS_TXRX_ON)) ;	// Wait for the RS485 chip to turn on (it might be on already)
		P3OUT &= ~TSS_INTERRUPT;		// reset the interrupt once it is acknowledged
		while (RX_BUFF_EMPTY) ;
		command = UCA0RXBUF;  			// get a command
		last_error = error_code;
		error_code = NO_ERROR;			// nothing wrong so far
		switch (command)				// process the command
		{		// commands 00-02:  Off, On and Reset the ADXL sensor
			case 0x00 :
				ADXL_off();					// Hardware power cycle -- puts ADXL in default low power standby
				send_progress_report();
				break;
			case 0x01 :
				ADXL_on();					// turns the sensor on
				send_progress_report();
				break;
			case 0x02 :
				error_code = ADXL_reset(); // does a hardware and software reset of the ADXL and puts it in standby
				send_progress_report();
				break;
			case 0x03 :
				msp2sleep();				// puts the microprocessor in a very low power mode
				send_progress_report();
				break;
				// commands 04-06: Receive the settings for the smart sensor
			case 0x04 :
				error_code = NO_ERROR_READY_TO_RECEIVE;
				send_progress_report(); 	// let the comm board know that we're expecting data
				calibrate();				// Receive temperature calibration data from Comm board
				break;
			case 0x05 :
				error_code = NO_ERROR_READY_TO_RECEIVE;
				send_progress_report(); 	// let the comm board know that we're expecting data
				register_SS();				// Receive GPS data and ID from Comm board for a new ID
				break;
			case 0x06:
				error_code = NO_ERROR_READY_TO_RECEIVE;
				send_progress_report(); 	// let the comm board know that we're expecting data
				new_parameters();			// Receive new parameters from Comm board
				break;
				// commands 07-08: delete bin records from the smart sensor
			case 0x07 :
				clear_record(); 			// Deletes the last bin record
				send_progress_report();
				break;
			case 0x08 :
				clear_all_records();		// Deletes all the bin records
				send_progress_report();
				break;
				// commands 09-0C: Send sensor data
			case 0x09 :
				error_code = NO_ERROR_DATA_READY;
				send_progress_report(); 	// let the comm board know that we're ready to send data
				send_last_record();			// sends out the last bin record
				break;
			case 0x0A :
				error_code = NO_ERROR_DATA_READY;
				send_progress_report(); 	// let the comm board know that we're ready to send data
				send_all_records();			// sends out all the bin records
				break;
			case 0x0B:
				error_code = NO_ERROR_READY_TO_RECEIVE;
				send_progress_report(); 	// let the comm board know that we're expecting data
				manual_operation();			// sends and receives data directly to and from the ADXL sensor
				break;
			case 0x0C:
				error_code = NO_ERROR_DATA_READY;
				send_progress_report(); 	// let the comm board know that we're ready to send data
				stream_data();				// streams data from the sensor's FIFO stack
				break;
				// commands 0D-0F: Send out current status of the smart sensor
			case 0x0D :
				error_code = NO_ERROR_DATA_READY;
				send_progress_report(); 	// let the comm board know that we're ready to send data
				check_ID();					// sends the sensor ID to the comm board
				break;
			case 0x0E :
				error_code = NO_ERROR_DATA_READY;
				send_progress_report(); 	// let the comm board know that we're ready to send data
				send_parameters();			// sends requested parameters to the comm board
				break;
			case 0x0F :
				error_code = last_error;	 // just sends the reason for the last interrupt (also error reporting)
				send_progress_report();
				break;
				// commands 10+: programmed run scripts
			case 0x10 :
				error_code = NO_ERROR_SIGN_OFF;
				send_progress_report(); 	// let the comm board know that it will be a while...
				resume_run_bins();			// continues a run after an interruption
				break;
			case 0x11 :
				error_code = NO_ERROR_SIGN_OFF;
				send_progress_report(); 	// let the comm board know that it will be a while...
				start_run_bins();			// starts a new run after resetting everything
				break;
			default :
				error_code = ERROR_NO_COMMAND; // the wrong byte was sent or there was a communication error
				send_progress_report();
		}
		if (!error_code) error_code = NO_ERROR_COMPLETE ;	// operation completed successfully, no data waiting
	}
}	/// We'll never get here.


//////////////// End of program//////////////////////////////





