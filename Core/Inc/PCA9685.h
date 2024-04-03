/*
 * PCA9685.h
 *
 *  Created on: Mar 22, 2024
 *      Author: dcm
 */

#ifndef INC_PCA9685_H_
#define INC_PCA9685_H_

#include <stdbool.h>
#include <stdint.h>
#include "main.h"


extern I2C_HandleTypeDef hi2c1;


#ifndef PCA9685_ENABLE_SOFTWARE_I2C
#include "stm32f4xx_hal_i2c.h"
#if BUFFER_LENGTH
#define PCA9685_I2C_BUFFER_LENGTH   BUFFER_LENGTH
#elif I2C_BUFFER_LENGTH
#define PCA9685_I2C_BUFFER_LENGTH   I2C_BUFFER_LENGTH
#else
#pragma message( "i2c buffer length not defined - using default value of 32, which may not be correct for your microcontroller. Check stm32f4xx_hal_i2c.h (or similar) for your hardware and manually define BUFFER_LENGTH or I2C_BUFFER_LENGTH to remove this warning.")
#define PCA9685_I2C_BUFFER_LENGTH   32
#endif // /if BUFFER_LENGTH
#else
#include "stm32f4xx.h"
#define PCA9685_USE_SOFTWARE_I2C
#endif // /ifndef PCA9685_ENABLE_SOFTWARE_I2C



// Default proxy addresser i2c addresses
#define PCA9685_I2C_DEF_ALLCALL_PROXYADR    	(byte)0xE0      // Default AllCall i2c proxy address
#define PCA9685_I2C_DEF_SUB1_PROXYADR       	(byte)0xE2      // Default Sub1 i2c proxy address
#define PCA9685_I2C_DEF_SUB2_PROXYADR       	(byte)0xE4      // Default Sub2 i2c proxy address
#define PCA9685_I2C_DEF_SUB3_PROXYADR       	(byte)0xE8      // Default Sub3 i2c proxy address


// Output driver control mode (see datasheet Table 12 and Fig 13, 14, and 15 concerning correct
// usage of OUTDRV).
typedef enum  {
    PCA9685_OutputDriverMode_OpenDrain,         // Module outputs in an open-drain (aka direct connection) style structure with 400mA @5v total sink current, useful for LEDs and low-power Servos
    PCA9685_OutputDriverMode_TotemPole,         // Module outputs in a totem-pole (aka push-pull) style structure with 400mA @5v total sink current and 160mA total source current, useful for external drivers (default)

    PCA9685_OutputDriverMode_Count,             // Internal use only
    PCA9685_OutputDriverMode_Undefined 		= -1     // Internal use only
}PCA9685_OutputDriverMode;

// Output-enabled/active-low-OE-pin=LOW driver output mode (see datasheet Table 12 and
// Fig 13, 14, and 15 concerning correct usage of INVRT).
typedef enum  {
    PCA9685_OutputEnabledMode_Normal,           // When OE is enabled/LOW, channels output a normal signal, useful for N-type external drivers (default)
    PCA9685_OutputEnabledMode_Inverted,         // When OE is enabled/LOW, channels output an inverted signal, useful for P-type external drivers or direct connection

    PCA9685_OutputEnabledMode_Count,            // Internal use only
    PCA9685_OutputEnabledMode_Undefined 	= -1    // Internal use only
}PCA9685_OutputEnabledMode;


// Output-not-enabled/active-low-OE-pin=HIGH driver output mode (see datasheet Section
// 7.4 concerning correct usage of OUTNE).
typedef enum  {
    PCA9685_OutputDisabledMode_Low,             // When OE is disabled/HIGH, channels output a LOW signal (default)
    PCA9685_OutputDisabledMode_High,            // When OE is disabled/HIGH, channels output a HIGH signal (only available in totem-pole mode)
    PCA9685_OutputDisabledMode_Floating,        // When OE is disabled/HIGH, channel outputs go into a floating (aka high-impedance/high-Z) state, which may be further refined via external pull-up/pull-down resistors

    PCA9685_OutputDisabledMode_Count,           // Internal use only
    PCA9685_OutputDisabledMode_Undefined	 = -1   // Internal use only
}PCA9685_OutputDisabledMode;



// Channel update strategy used when multiple channels are being updated in batch.
typedef enum  {
    PCA9685_ChannelUpdateMode_AfterStop,        // Channel updates commit after full-transmission STOP signal (default)
    PCA9685_ChannelUpdateMode_AfterAck,         // Channel updates commit after individual channel update ACK signal

    PCA9685_ChannelUpdateMode_Count,            // Internal use only
    PCA9685_ChannelUpdateMode_Undefined		 = -1    // Internal use only
}PCA9685_ChannelUpdateMode;



// Software-based phase balancing scheme.
typedef enum  {
    PCA9685_PhaseBalancer_None,                 // Disables software-based phase balancing, relying on installed hardware to handle current sinkage (default)
    PCA9685_PhaseBalancer_Linear,               // Uses linear software-based phase balancing, with each channel being a preset 16 steps (out of the 4096/12-bit value range) away from previous channel (may cause LED flickering/skipped-cycles on PWM changes)

    PCA9685_PhaseBalancer_Count,                // Internal use only
    PCA9685_PhaseBalancer_Undefined			 = -1        // Internal use only
}PCA9685_PhaseBalancer;



#ifndef PCA9685_USE_SOFTWARE_I2C

void PCA9685_1(byte i2cAddress, I2C_HandleTypeDef i2cWire , uint32_t i2cSpeed );

// Convenience constructor for custom Wire instance. See main constructor.
void PCA9685_2(I2C_HandleTypeDef i2cWire, uint32_t i2cSpeed, byte i2cAddress);

#else

    PCA9685_3(byte i2cAddress);

#endif

    // Resets modules. Typically called in setup(), before any init()'s. Calling will
    // perform a software reset on all PCA9685 devices on the Wire instance, ensuring
    // that all PCA9685 devices on that line are properly reset.
    void resetDevices();

    // Initializes module. Typically called in setup().
    // See individual enums for more info.
    //function prototype
     void init_1(PCA9685_OutputDriverMode driverMode,
    		 	 PCA9685_OutputEnabledMode enabledMode,
                 PCA9685_OutputDisabledMode disabledMode,
                 PCA9685_ChannelUpdateMode updateMode,
                 PCA9685_PhaseBalancer phaseBalancer);




    // Convenience initializer for custom phase balancer. See main init method.
    void init_2(PCA9685_PhaseBalancer phaseBalancer,
    			PCA9685_OutputDriverMode driverMode,
				PCA9685_OutputEnabledMode enabledMode,
				PCA9685_OutputDisabledMode disabledMode,
				PCA9685_ChannelUpdateMode updateMode);



    // Initializes module as a proxy addresser. Typically called in setup(). Used when
    // instance talks through to AllCall/Sub1-Sub3 instances as a proxy object. Using
    // this method will disable any method that performs a read or conflicts with certain
    // states. Proxy addresser i2c addresses must be >= 0xE0, with defaults provided via
    // PCA9685_I2C_DEF_[ALLCALL|SUB[1-3]]_PROXYADR defines.
    void initAsProxyAddresser();

    // Mode accessors
    byte getI2CAddress();
    uint32_t getI2CSpeed();
    PCA9685_OutputDriverMode getOutputDriverMode();
    PCA9685_OutputEnabledMode getOutputEnabledMode();
    PCA9685_OutputDisabledMode getOutputDisabledMode();
    PCA9685_ChannelUpdateMode getChannelUpdateMode();
    PCA9685_PhaseBalancer getPhaseBalancer();

    // Min: 24Hz, Max: 1526Hz, Default: 200Hz. As Hz increases channel resolution
    // diminishes, as raw pre-scaler value, computed per datasheet, starts to require
    // much larger frequency increases for single-digit increases of the raw pre-scaler
    // value that ultimately controls the PWM frequency produced.
    void setPWMFrequency(float pwmFrequency);
    // Sets standard servo frequency of 50Hz.
    void setPWMFreqServo();

    // Turns channel either full on or full off
    void setChannelOn(int channel);
    void setChannelOff(int channel);

    // PWM amounts 0 - 4096, 0 full off, 4096 full on
    void setChannelPWM(int channel, uint16_t pwmAmount);
    void setChannelsPWM(int begChannel, int numChannels, const uint16_t *pwmAmounts);

    // Sets all channels, but won't distribute phases
    void setAllChannelsPWM(uint16_t pwmAmount);

    // Returns PWM amounts 0 - 4096, 0 full off, 4096 full on
    uint16_t getChannelPWM(int channel);

    // Enables multiple talk-through paths via i2c bus (lsb/bit0 must stay 0). To use,
    // create a new proxy instance using initAsProxyAddresser() with proper proxy i2c
    // address >= 0xE0, and pass that instance's i2c address into desired method below.

    void enableAllCallAddress(byte i2cAddressAllCall);



    void enableSub1Address(byte i2cAddressSub1 );
    void enableSub2Address(byte i2cAddressSub2);
    void enableSub3Address(byte i2cAddressSub3 );
    void disableAllCallAddress();
    void disableSub1Address();
    void disableSub2Address();
    void disableSub3Address();

    // Allows external clock line to be utilized (power reset required to disable)
    void enableExtClockLine();

    byte getLastI2CError();

	#ifdef PCA9685_ENABLE_DEBUG_OUTPUT
    	int getWireInterfaceNumber();
    	void printModuleInfo();
    	void checkForErrors();
	#endif

   extern byte _i2cAddress;                                       // Module's i2c address (default: B000000)
   #ifndef PCA9685_USE_SOFTWARE_I2C
    	extern I2C_HandleTypeDef _i2cWire;                                      // Wire class instance (unowned) (default: Wire)
        extern uint32_t _i2cSpeed;                                     // Module's i2c clock speed (default: 400000)
   #endif
        extern PCA9685_OutputDriverMode _driverMode;                   // Output driver mode
        extern PCA9685_OutputEnabledMode _enabledMode;                 // OE enabled output mode
       extern PCA9685_OutputDisabledMode _disabledMode;               // OE disabled output mode
       extern PCA9685_ChannelUpdateMode _updateMode;                  // Channel update mode
       extern PCA9685_PhaseBalancer _phaseBalancer;                   // Phase balancer scheme
       extern bool _isProxyAddresser;                                 // Proxy addresser flag (disables certain functionality)
       extern byte _lastI2CError;                                     // Last module i2c error

       byte getMode2Value();
       void getPhaseCycle(int channel, uint16_t pwmAmount, uint16_t *phaseBegin, uint16_t *phaseEnd);

       void writeChannelBegin(int channel);
       void writeChannelPWM(uint16_t phaseBegin, uint16_t phaseEnd);
       void writeChannelEnd();

       void writeRegister(byte regAddress, byte value);
       byte readRegister(byte regAddress);

   #ifdef PCA9685_USE_SOFTWARE_I2C
       uint8_t _readBytes;
   #endif
       void i2cWire_begin();
       HAL_StatusTypeDef i2cWire_begin_end_write_Transmission(uint8_t, uint8_t);
       //uint8_t i2cWire_endTransmission(void);
       uint8_t i2cWire_requestFrom(uint8_t, uint8_t);
       //size_t i2cWire_write(uint8_t data);
       uint8_t i2cWire_read(void);



       // Uses a linear interpolation method to quickly compute PWM output value. Uses
           // default values of 2.5% and 12.5% of phase length for -90/+90 (or -1x/+1x).
           void PCA9685_ServoEval_1(uint16_t minPWMAmount, uint16_t maxPWMAmount);

           // Uses a cubic spline to interpolate due to an offsetted zero point that isn't
           // exactly between -90/+90 (or -1x/+1x). This takes more time to compute, but gives a
           // smoother PWM output value along the entire range.
           void PCA9685_ServoEval_2(uint16_t minPWMAmount, uint16_t midPWMAmount, uint16_t maxPWMAmount);

           // Returns the PWM value to use given the angle offset (-90 to +90)
               uint16_t pwmForAngle(float angle);

               // Returns the PWM value to use given the speed multiplier (-1 to +1)
               uint16_t pwmForSpeed(float speed);

               extern float *_coeff;      // a,b,c,d coefficient values
               extern bool _isCSpline;    // Cubic spline tracking, for _coeff length



#endif // PCA9685_STM32F4_H
