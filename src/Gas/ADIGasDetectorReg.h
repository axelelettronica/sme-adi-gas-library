/*
 * LPS25HReg.h
 *
 * Created: 26/04/2015 18:45:26
 *  Author: speirano
 */


#ifndef ADI_GAS_DETECTOR_REG_H_
#define ADI_GAS_DETECTOR_REG_H_




/*
 * [7] PD: power down control.
 * Default value: 0
 * (0: power-down mode; 1: active mode)
 *
 * [6:4] ODR2, ODR1, ODR0: output data rate selection.
 * Default value: 00
 *
 * [3] DIFF_EN: Interrupt circuit enable.
 * Default value: 0
 * (0: interrupt generation disabled; 1: interrupt circuit enabled)
 *
 * [2] BDU: block data update.
 * Default value: 0
 * (0: continuous update; 1: output registers not updated until MSB and LSB reading)
 BDU bit is used to inhibit the output registers update between the reading of upper and
 lower register parts. In default mode (BDU = ?0?), the lower and upper register parts are
 updated continuously. If it is not sure to read faster than output data rate, it is recommended
 to set BDU bit to ?1?. In this way, after the reading of the lower (upper) register part, the
 content of that output registers is not updated until the upper (lower) part
 *
 * [1] RESET_AZ: Reset AutoZero function. Reset REF_P reg, set pressure to default value in RPDS
 * register (@0x39/A)
 * (1: Reset. 0: disable)
 *
 * [0] SIM: SPI Serial Interface Mode selection.
 * Default value: 0
 * (0: 4-wire interface; 1: 3-wire interface)
 */
#define CTRL_REG1   0x20
#define POWER_UP    0x80
#define BDU_SET     0x04
#define ODR0_SET    0x10  // 1 read each second



#define CTRL_REG2   0x21
#define CTRL_REG3   0x22
#define REG_DEFAULT 0x00

/*
 * This register is updated every ODR cycle, regardless of BDU value in CTRL_REG1.
 *
 * P_DA is set to 1 whenever a new pressure sample is available.
 * P_DA is cleared when PRESS_OUT_H (2Ah) register is read.
 *
 * T_DA is set to 1 whenever a new temperature sample is available.
 * T_DA is cleared when TEMP_OUT_H (2Ch) register is read.
 *
 * P_OR bit is set to '1' whenever new pressure data is available and P_DA was set in
 * the previous ODR cycle and not cleared.
 * P_OR is cleared when PRESS_OUT_H (2Ah) register is read.
 *
 * T_OR is set to ?1? whenever new temperature data is available and T_DA was set in
 * the previous ODR cycle and not cleared.
 * T_OR is cleared when TEMP_OUT_H (2Ch) register is read.
 *
 * [7:6] Reserved
 *
 * [5] P_OR: Pressure data overrun. Default value: 0
 * (0: no overrun has occurred;
 * 1: new data for pressure has overwritten the previous one)
 *
 * [4] T_OR: Temperature data overrun. Default value: 0
 * (0: no overrun has occurred;
 * 1: a new data for temperature has overwritten the previous one)
 *
 * [3:2] Reserved
 *
 * [1] P_DA: Pressure data available. Default value: 0
 * (0: new data for pressure is not yet available;
 * 1: new data for pressure is available)
 *
 * [0] T_DA: Temperature data available. Default value: 0
 * (0: new data for temperature is not yet available;
 * 1: new data for temperature is available)
 */
#define STATUS_REG          0x27
#define TEMPERATURE_READY    0x1
#define REOSTATE_READY       0x2
#define CO2_READY            0x3


//#define ADI_GAS_DETECTOR_ADDRESS 0xA0 //0x5C
#define ADI_GAS_DETECTOR_ADDRESS 0x50

// DEV ID Register - Read-Only
#define WHO_AM_I              0x01
#define WHO_AM_I_RETURN       0xB1 // Contains the device ID, BDh

// Sw Version Register - Read-Only
#define VERSION_REG           0x02   // Get FW Version

// Temperature Register - Read-Only
#define TEMP_H_REG            0x03   // Get Temperature
// Temperature Register - Read-Only
#define TEMP_L_REG            0x04  // Get Temperature

// RHEOSTATE Register - Read/Write
#define REOST_H_REG           0x0A    // Rheostate Register
// RHEOSTATE Register - Read/Write
#define REOST_L_REG           0x0C   // Rheostate Register
// RHEOSTATE Register - Read/Write
#define REOST_W_H_REG         0x0B    // Rheostate Register
// RHEOSTATE Register - Read/Write
#define REOST_W_L_REG         0x0D   // Rheostate Register

// Gas Value has 4 bytes size:  <XH-XL-H-L>
// GAS Detection Register - Read-Only
#define CO2_H_REG             0x20          // Rheostate Register
// GAS Detection Register - Read-Only
#define CO2_L_REG             0x21         // Rheostate Register
// GAS Detection Register - Read-Only
#define CO2_X_H_REG           0x22          // Rheostate Register
// GAS Detection Register - Read-Only
#define CO2_X_L_REG           0x23         // Rheostate Register

#define I2C_RHEOSTATE_50TP_STORE   0x30     // Store RDAC in last available position
#define I2C_R_RHEOSTATE_50TP_R_H   0x31     // Rheostate TP index val
#define I2C_R_RHEOSTATE_50_TP_R_L  0x32     // Rheostate TP index val		
#define I2C_R_RHEOSTATE_50TP_R_POS 0x33     // Get last used position		

        // Unsupported commands      
#define I2C_SLEEP                   0x3A    // Shutdown
#define I2C_WAKEUP                  0x3B    // Wakeup 
#define I2C_RHEOSTATE_RESET         0x3C    // Reset

#endif /* ADI_GAS_DETECTOR_REG_H_ */
