/*
 * PCA9685.c
 *
 *  Created on: Mar 24, 2024
 *      Author: cdemir
 */


#include "PCA9685.h"
#include <assert.h>
#include "stm32f4xx_hal.h"
#include "math.h"
#include "main.h"


extern I2C_HandleTypeDef hi2c1;

#define PCA9685_I2C_BASE_MODULE_ADDRESS (byte)0x40
#define PCA9685_I2C_BASE_MODULE_ADRMASK (byte)0x3F
#define PCA9685_I2C_BASE_PROXY_ADDRESS  (byte)0xE0
#define PCA9685_I2C_BASE_PROXY_ADRMASK  (byte)0xFE

// Register addresses from data sheet
#define PCA9685_MODE1_REG               (byte)0x00
#define PCA9685_MODE2_REG               (byte)0x01
#define PCA9685_SUBADR1_REG             (byte)0x02
#define PCA9685_SUBADR2_REG             (byte)0x03
#define PCA9685_SUBADR3_REG             (byte)0x04
#define PCA9685_ALLCALL_REG             (byte)0x05
#define PCA9685_LED0_REG                (byte)0x06          // Start of LEDx regs, 4B per reg, 2B on phase, 2B off phase, little-endian
#define PCA9685_PRESCALE_REG            (byte)0xFE
#define PCA9685_ALLLED_REG              (byte)0xFA

// Mode1 register values
#define PCA9685_MODE1_RESTART           (byte)0x80
#define PCA9685_MODE1_EXTCLK            (byte)0x40
#define PCA9685_MODE1_AUTOINC           (byte)0x20
#define PCA9685_MODE1_SLEEP             (byte)0x10
#define PCA9685_MODE1_SUBADR1           (byte)0x08
#define PCA9685_MODE1_SUBADR2           (byte)0x04
#define PCA9685_MODE1_SUBADR3           (byte)0x02
#define PCA9685_MODE1_ALLCALL           (byte)0x01

// Mode2 register values
#define PCA9685_MODE2_OUTDRV_TPOLE      (byte)0x04
#define PCA9685_MODE2_INVRT             (byte)0x10
#define PCA9685_MODE2_OUTNE_TPHIGH      (byte)0x01
#define PCA9685_MODE2_OUTNE_HIGHZ       (byte)0x02
#define PCA9685_MODE2_OCH_ONACK         (byte)0x08

#define PCA9685_SW_RESET                (byte)0x06          // Sent to address 0x00 to reset all devices on Wire line
#define PCA9685_PWM_FULL                (uint16_t)0x1000    // Special value for full on/full off LEDx modes
#define PCA9685_PWM_MASK                (uint16_t)0x0FFF    // Mask for 12-bit/4096 possible phase positions

#define PCA9685_CHANNEL_COUNT           16
#define PCA9685_MIN_CHANNEL             0
#define PCA9685_MAX_CHANNEL             (PCA9685_CHANNEL_COUNT - 1)
#define PCA9685_ALLLED_CHANNEL          -1                  // Special value for ALLLED registers

#ifdef PCA9685_USE_SOFTWARE_I2C
bool __attribute__((noinline)) i2c_init(void);
bool __attribute__((noinline)) i2c_start(uint8_t addr);
void __attribute__((noinline)) PCA9685_i2c_stop(void) asm("ass_i2c_stop");
bool __attribute__((noinline)) PCA9685_i2c_write(uint8_t value) asm("ass_i2c_write");
uint8_t __attribute__((noinline)) i2c_read(bool last);
#endif

#ifndef PCA9685_USE_SOFTWARE_I2C


void PCA9685_1(byte i2cAddress, I2C_HandleTypeDef* i2cWire , uint32_t i2cSpeed )
{



	 	_i2cAddress = i2cAddress;
	    _i2cWire = i2cWire;
	    _i2cSpeed = i2cSpeed;
	    _driverMode = PCA9685_OutputDriverMode_Undefined;
	    _enabledMode = PCA9685_OutputEnabledMode_Undefined;
	    _disabledMode = PCA9685_OutputDisabledMode_Undefined;
	    _updateMode = PCA9685_ChannelUpdateMode_Undefined;
	    _phaseBalancer = PCA9685_PhaseBalancer_Undefined;
	    _isProxyAddresser = false;
	    _lastI2CError = 0;
}



void PCA9685_2(I2C_HandleTypeDef* i2cWire, uint32_t i2cSpeed, byte i2cAddress)
{


		_i2cAddress = i2cAddress;
	    _i2cWire = i2cWire;
	    _i2cSpeed = i2cSpeed;
	    _driverMode = PCA9685_OutputDriverMode_Undefined;
	    _enabledMode = PCA9685_OutputEnabledMode_Undefined;
	    _disabledMode = PCA9685_OutputDisabledMode_Undefined;
	    _updateMode = PCA9685_ChannelUpdateMode_Undefined;
	    _phaseBalancer = PCA9685_PhaseBalancer_Undefined;
	    _isProxyAddresser = false;
	    _lastI2CError = 0;
}


#else

PCA9685_3(byte i2cAddress)
{
			_i2cAddress = i2cAddress;
		    _i2cWire = i2cWire;
		    _i2cSpeed = i2cSpeed;
		    _driverMode = PCA9685_OutputDriverMode_Undefined;
		    _enabledMode = PCA9685_OutputEnabledMode_Undefined;
		    _disabledMode = PCA9685_OutputDisabledMode_Undefined;
		    _updateMode = PCA9685_ChannelUpdateMode_Undefined;
		    _phaseBalancer = PCA9685_PhaseBalancer_Undefined;
		    _isProxyAddresser = false;
		    _lastI2CError = 0;
}

#endif // /ifndef PCA9685_USE_SOFTWARE_I2C

void resetDevices()
{
#ifdef PCA9685_ENABLE_DEBUG_OUTPUT
	printf("resetDevices\n\r");
#endif

MX_I2C1_Init();

#ifdef PCA9685_ENABLE_DEBUG_OUTPUT
void checkForErrors ();
#endif
uint8_t address = 0x00;
byte A=PCA9685_SW_RESET;

HAL_I2C_Master_Transmit(&hi2c1,(uint16_t)(address<<1),&A,1, HAL_MAX_DELAY);
delay(1000);


}

void init_1(PCA9685_OutputDriverMode driverMode,
                   PCA9685_OutputEnabledMode enabledMode,
                   PCA9685_OutputDisabledMode disabledMode,
                   PCA9685_ChannelUpdateMode updateMode,
                   PCA9685_PhaseBalancer phaseBalancer) {
    if (_isProxyAddresser) return;

    _i2cAddress = PCA9685_I2C_BASE_MODULE_ADDRESS | (_i2cAddress & PCA9685_I2C_BASE_MODULE_ADRMASK);

    _driverMode = driverMode;
    _enabledMode = enabledMode;
    _disabledMode = disabledMode;
    _updateMode = updateMode;
    _phaseBalancer = phaseBalancer;

    assert(!(_driverMode == PCA9685_OutputDriverMode_OpenDrain && _disabledMode == PCA9685_OutputDisabledMode_High && "Unsupported combination"));

    byte mode2Val = getMode2Value();

#ifdef PCA9685_ENABLE_DEBUG_OUTPUT
    printf("init_1 mode2Val: 0x");
    printf("%X ",mode2Val);
    printf(", i2cAddress: 0x");
    printf("%X",_i2cAddress);
    printf(", i2cWire#: ");
    printf("%d",getWireInterfaceNumber());
    printf(", i2cSpeed: ");
    printf("%d",round(getI2CSpeed() / 1000.0f));
    printf("kHz");
    printf(", driverMode: ");
    switch(_driverMode) {
        case PCA9685_OutputDriverMode_OpenDrain: printf("OpenDrain"); break;
        case PCA9685_OutputDriverMode_TotemPole: printf("TotemPole"); break;
        case PCA9685_OutputDriverMode_Count:
        case PCA9685_OutputDriverMode_Undefined:
            printf("%d",_driverMode); break;
    }
    printf(", enabledMode: ");
    switch (_enabledMode) {
        case PCA9685_OutputEnabledMode_Normal: printf("Normal"); break;
        case PCA9685_OutputEnabledMode_Inverted: printf("Inverted"); break;
        case PCA9685_OutputEnabledMode_Count:
        case PCA9685_OutputEnabledMode_Undefined:
            printf("%d",_enabledMode); break;
    }
    printf(", disabledMode: ");
    switch (_disabledMode) {
        case PCA9685_OutputDisabledMode_Low: printf("Low"); break;
        case PCA9685_OutputDisabledMode_High: printf("High"); break;
        case PCA9685_OutputDisabledMode_Floating: printf("Floating"); break;
        case PCA9685_OutputDisabledMode_Count:
        case PCA9685_OutputDisabledMode_Undefined:
            printf("%d",_disabledMode); break;
    }
    printf(", updateMode: ");
    switch (_updateMode) {
        case PCA9685_ChannelUpdateMode_AfterStop: printf("AfterStop"); break;
        case PCA9685_ChannelUpdateMode_AfterAck: printf("AfterAck"); break;
        case PCA9685_ChannelUpdateMode_Count:
        case PCA9685_ChannelUpdateMode_Undefined:
            printf("%d",_updateMode); break;
    }
    printf(", phaseBalancer: ");
    switch(_phaseBalancer) {
        case PCA9685_PhaseBalancer_None: printf("None"); break;
        case PCA9685_PhaseBalancer_Linear: printf("Linear"); break;
        case PCA9685_PhaseBalancer_Count:
        case PCA9685_PhaseBalancer_Undefined:
            printf("%d",_phaseBalancer); break;
    }
    printf("\n\r");
#endif

    MX_I2C1_Init();

#ifdef PCA9685_ENABLE_DEBUG_OUTPUT
    checkForErrors();
#endif

    writeRegister(PCA9685_MODE1_REG, PCA9685_MODE1_RESTART | PCA9685_MODE1_AUTOINC);
    writeRegister(PCA9685_MODE2_REG, mode2Val);
}

void init_2(PCA9685_PhaseBalancer phaseBalancer,
                   PCA9685_OutputDriverMode driverMode,
                   PCA9685_OutputEnabledMode enabledMode,
                   PCA9685_OutputDisabledMode disabledMode,
                   PCA9685_ChannelUpdateMode updateMode) {

    init_2(driverMode, enabledMode, disabledMode, updateMode, phaseBalancer);
}


void initAsProxyAddresser() {
    if (_driverMode != PCA9685_OutputDriverMode_Undefined) return;

    _i2cAddress = PCA9685_I2C_BASE_PROXY_ADDRESS | (_i2cAddress & PCA9685_I2C_BASE_PROXY_ADRMASK);
    _isProxyAddresser = true;

#ifdef PCA9685_ENABLE_DEBUG_OUTPUT
    printf("PCA9685::initAsProxyAddresser i2cAddress: 0x");
    printf("%X",_i2cAddress);
    printf(", i2cWire#: ");
    printf("%d",getWireInterfaceNumber());
    printf(", i2cSpeed: ");
    printf("%d",roundf(getI2CSpeed() / 1000.0f));
    printf("kHz");
    printf("\n");
#endif

    MX_I2C1_Init();

#ifdef PCA9685_ENABLE_DEBUG_OUTPUT
    checkForErrors();
#endif
}




byte getI2CAddress()
{
	return _i2cAddress;
}

uint32_t getI2CSpeed() {
#ifndef PCA9685_USE_SOFTWARE_I2C
    return _i2cSpeed;
#else
#if I2C_FASTMODE || F_CPU >= 16000000
    return 400000;
#else
    return 100000;
#endif
#endif // ifndef PCA9685_USE_SOFTWARE_I2C
}


PCA9685_OutputDriverMode getOutputDriverMode() {
    return _driverMode;
}

PCA9685_OutputEnabledMode getOutputEnabledMode() {
    return _enabledMode;
}

PCA9685_OutputDisabledMode getOutputDisabledMode() {
    return _disabledMode;
}

PCA9685_ChannelUpdateMode getChannelUpdateMode() {
    return _updateMode;
}

PCA9685_PhaseBalancer getPhaseBalancer() {
    return _phaseBalancer;
}


byte getMode2Value() {
    byte mode2Val = (byte)0x00;

    if (_driverMode == PCA9685_OutputDriverMode_TotemPole) {
        mode2Val |= PCA9685_MODE2_OUTDRV_TPOLE;
    }

    if (_enabledMode == PCA9685_OutputEnabledMode_Inverted) {
        mode2Val |= PCA9685_MODE2_INVRT;
    }

    if (_disabledMode == PCA9685_OutputDisabledMode_High) {
        mode2Val |= PCA9685_MODE2_OUTNE_TPHIGH;
    }
    else if (_disabledMode == PCA9685_OutputDisabledMode_Floating) {
        mode2Val |= PCA9685_MODE2_OUTNE_HIGHZ;
    }

    if (_updateMode == PCA9685_ChannelUpdateMode_AfterAck) {
        mode2Val |= PCA9685_MODE2_OCH_ONACK;
    }

    return mode2Val;
}

void setPWMFrequency(float pwmFrequency) {
    if (pwmFrequency < 0 || _isProxyAddresser) return;

    // This equation comes from section 7.3.5 of the datasheet, but the rounding has been
    // removed because it isn't needed. Lowest freq is 23.84, highest is 1525.88.
    int preScalerVal = (25000000 / (4096 * pwmFrequency)) - 1;
    if (preScalerVal > 255) preScalerVal = 255;
    if (preScalerVal < 3) preScalerVal = 3;

#ifdef PCA9685_ENABLE_DEBUG_OUTPUT
    printf("setPWMFrequency pwmFrequency: ");
    printf("%f",pwmFrequency);
    printf(", preScalerVal: 0x%X",preScalerVal);
    printf("%X\n",preScalerVal);
#endif

    // The PRE_SCALE register can only be set when the SLEEP bit of MODE1 register is set to logic 1.
    byte mode1Reg = readRegister(PCA9685_MODE1_REG);
    writeRegister(PCA9685_MODE1_REG, (mode1Reg = (mode1Reg & ~PCA9685_MODE1_RESTART) | PCA9685_MODE1_SLEEP));
    writeRegister(PCA9685_PRESCALE_REG, (byte)preScalerVal);

    // It takes 500us max for the oscillator to be up and running once SLEEP bit has been set to logic 0.
    writeRegister(PCA9685_MODE1_REG, (mode1Reg = (mode1Reg & ~PCA9685_MODE1_SLEEP) | PCA9685_MODE1_RESTART));
    HAL_Delay(500);
}

void setPWMFreqServo() {
    setPWMFrequency(50);
}

void setChannelOn(int channel) {
    if (channel < 0 || channel > 15) return;

#ifdef PCA9685_ENABLE_DEBUG_OUTPUT
    printf("PCA9685::setChannelOn");
#endif

    writeChannelBegin(channel);
    writeChannelPWM(PCA9685_PWM_FULL, 0);  // time_on = FULL; time_off = 0;
    writeChannelEnd();
}

void setChannelOff(int channel) {
    if (channel < 0 || channel > 15) return;

#ifdef PCA9685_ENABLE_DEBUG_OUTPUT
    printf("PCA9685::setChannelOff");
#endif

    writeChannelBegin(channel);
    writeChannelPWM(0, PCA9685_PWM_FULL);  // time_on = 0; time_off = FULL;
    writeChannelEnd();
}

void setChannelPWM(int channel, uint16_t pwmAmount) {
    if (channel < 0 || channel > 15) return;

#ifdef PCA9685_ENABLE_DEBUG_OUTPUT
    printf("setChannelPWM\n");
#endif

    writeChannelBegin(channel);

    uint16_t phaseBegin, phaseEnd;
    getPhaseCycle(channel, pwmAmount, &phaseBegin, &phaseEnd);

    writeChannelPWM(phaseBegin, phaseEnd);

    writeChannelEnd();
}


void PsetChannelsPWM(int begChannel, int numChannels, const uint16_t *pwmAmounts) {
    if (begChannel < 0 || begChannel > 15 || numChannels < 0) return;
    if (begChannel + numChannels > 16) numChannels -= (begChannel + numChannels) - 16;

#ifdef PCA9685_ENABLE_DEBUG_OUTPUT
    printf("PCA9685::setChannelsPWM numChannels: ");
    printf("%d\n",numChannels);
#endif

    // From avr/libraries/Wire.h and avr/libraries/utility/twi.h, BUFFER_LENGTH controls
    // how many channels can be written at once. Therefore, we loop around until all
    // channels have been written out into their registers. I2C_BUFFER_LENGTH is used in
    // other architectures, so we rely on PCA9685_I2C_BUFFER_LENGTH logic to sort it out.

    while (numChannels > 0) {
        writeChannelBegin(begChannel);

#ifndef PCA9685_USE_SOFTWARE_I2C
        int maxChannels = MIN(numChannels, (PCA9685_I2C_BUFFER_LENGTH - 1) / 4);
#else // TODO: Software I2C doesn't have buffer length restrictions? -NR
        int maxChannels = numChannels;
#endif
        while (maxChannels-- > 0) {
            uint16_t phaseBegin, phaseEnd;
            getPhaseCycle(begChannel++, *pwmAmounts++, &phaseBegin, &phaseEnd);

            writeChannelPWM(phaseBegin, phaseEnd);
            --numChannels;
        }

        writeChannelEnd();
        if (_lastI2CError) return;
    }
}

void setAllChannelsPWM(uint16_t pwmAmount) {
#ifdef PCA9685_ENABLE_DEBUG_OUTPUT
    printf("setAllChannelsPWM");
#endif

    writeChannelBegin(PCA9685_ALLLED_CHANNEL);

    uint16_t phaseBegin, phaseEnd;
    getPhaseCycle(PCA9685_ALLLED_CHANNEL, pwmAmount, &phaseBegin, &phaseEnd);

    writeChannelPWM(phaseBegin, phaseEnd);

    writeChannelEnd();
}

uint16_t getChannelPWM(int channel) {
    if (channel < 0 || channel > 15 || _isProxyAddresser) return 0;

    byte regAddress = PCA9685_LED0_REG + (channel << 2);

#ifdef PCA9685_ENABLE_DEBUG_OUTPUT
    printf("getChannelPWM channel: ");
    printf(channel);
    printf(":,%X", regAddress);
    printf("%X",regAddress);
#endif




    if(HAL_I2C_Master_Transmit(&hi2c1,(uint16_t)(_i2cAddress<<1),&regAddress,1, HAL_MAX_DELAY)!=HAL_OK)
   {
#ifdef PCA9685_ENABLE_DEBUG_OUTPUT
        checkForErrors();
#endif
        return 0;
    }

    int bytesRead = i2cWire_requestFrom((uint8_t)_i2cAddress, 4);
    if (bytesRead != 4) {
        while (bytesRead-- > 0)
            i2cWire_read();
#ifdef PCA9685_USE_SOFTWARE_I2C
        PCA9685_i2c_stop(); // Manually have to send stop bit in software i2c mode
#endif
        _lastI2CError = 4;
#ifdef PCA9685_ENABLE_DEBUG_OUTPUT
        checkForErrors();
#endif
        return 0;
  }
#ifndef PCA9685_SWAP_PWM_BEG_END_REGS
    uint16_t phaseBegin = (uint16_t)i2cWire_read();
    phaseBegin |= (uint16_t)i2cWire_read() << 8;
    uint16_t phaseEnd = (uint16_t)i2cWire_read();
    phaseEnd |= (uint16_t)i2cWire_read() << 8;
#else
    uint16_t phaseEnd = (uint16_t)i2cWire_read();
    phaseEnd |= (uint16_t)i2cWire_read() << 8;
    uint16_t phaseBegin = (uint16_t)i2cWire_read();
    phaseBegin |= (uint16_t)i2cWire_read() << 8;
#endif

#ifdef PCA9685_USE_SOFTWARE_I2C
    PCA9685_i2c_stop(); // Manually have to send stop bit in software i2c mode
#endif

#ifdef PCA9685_ENABLE_DEBUG_OUTPUT
    printf("  getChannelPWM phaseBegin: ");
    printf("%d",phaseBegin);
    printf(", phaseEnd: ");
    printf("%d",phaseEnd);
#endif

    // See datasheet section 7.3.3
    uint16_t retVal;
    if (phaseEnd >= PCA9685_PWM_FULL)
        // Full OFF
        // Figure 11 Example 4: full OFF takes precedence over full ON
        // See also remark after Table 7
        retVal = 0;
    else if (phaseBegin >= PCA9685_PWM_FULL)
        // Full ON
        // Figure 9 Example 3
        retVal = PCA9685_PWM_FULL;
    else if (phaseBegin <= phaseEnd)
        // start and finish in same cycle
        // Section 7.3.3 example 1
        retVal = phaseEnd - phaseBegin;
    else
        // span cycles
        // Section 7.3.3 example 2
        retVal = (phaseEnd + PCA9685_PWM_FULL) - phaseBegin;

#ifdef PCA9685_ENABLE_DEBUG_OUTPUT
    printf("getChannelPWM retVal: ");
    printf("\n%d",retVal);
#endif

    return retVal;
}

void PenableAllCallAddress(byte i2cAddressAllCall) {
    if (_isProxyAddresser) return;

    byte i2cAddress = PCA9685_I2C_BASE_PROXY_ADDRESS | (i2cAddressAllCall & PCA9685_I2C_BASE_PROXY_ADRMASK);

#ifdef PCA9685_ENABLE_DEBUG_OUTPUT
    printf("enableAllCallAddress i2cAddressAllCall: 0x");
    printf("%X\n",i2cAddress);
#endif

    writeRegister(PCA9685_ALLCALL_REG, i2cAddress);

    byte mode1Reg = readRegister(PCA9685_MODE1_REG);
    writeRegister(PCA9685_MODE1_REG, (mode1Reg |= PCA9685_MODE1_ALLCALL));
}

void enableSub1Address(byte i2cAddressSub1) {
    if (_isProxyAddresser) return;

    byte i2cAddress = PCA9685_I2C_BASE_PROXY_ADDRESS | (i2cAddressSub1 & PCA9685_I2C_BASE_PROXY_ADRMASK);

#ifdef PCA9685_ENABLE_DEBUG_OUTPUT
    printf("enableSub1Address i2cAddressSub1: 0x");
    printf("%X\n",i2cAddress);
#endif

    writeRegister(PCA9685_SUBADR1_REG, i2cAddress);

    byte mode1Reg = readRegister(PCA9685_MODE1_REG);
    writeRegister(PCA9685_MODE1_REG, (mode1Reg |= PCA9685_MODE1_SUBADR1));
}


void enableSub2Address(byte i2cAddressSub2) {
    if (_isProxyAddresser) return;

    byte i2cAddress = PCA9685_I2C_BASE_PROXY_ADDRESS | (i2cAddressSub2 & PCA9685_I2C_BASE_PROXY_ADRMASK);

#ifdef PCA9685_ENABLE_DEBUG_OUTPUT
    printf("enableSub2Address i2cAddressSub2: 0x");
    printf("%X\n",i2cAddress);
#endif

    writeRegister(PCA9685_SUBADR2_REG, i2cAddress);

    byte mode1Reg = readRegister(PCA9685_MODE1_REG);
    writeRegister(PCA9685_MODE1_REG, (mode1Reg |= PCA9685_MODE1_SUBADR2));
}

void enableSub3Address(byte i2cAddressSub3) {
    if (_isProxyAddresser) return;

    byte i2cAddress = PCA9685_I2C_BASE_PROXY_ADDRESS | (i2cAddressSub3 & PCA9685_I2C_BASE_PROXY_ADRMASK);

#ifdef PCA9685_ENABLE_DEBUG_OUTPUT
    printf("enableSub3Address i2cAddressSub3: 0x");
    printf("%X\n",i2cAddress);
#endif

    writeRegister(PCA9685_SUBADR3_REG, i2cAddress);

    byte mode1Reg = readRegister(PCA9685_MODE1_REG);
    writeRegister(PCA9685_MODE1_REG, (mode1Reg |= PCA9685_MODE1_SUBADR3));
}

void PdisableAllCallAddress() {
    if (_isProxyAddresser) return;

#ifdef PCA9685_ENABLE_DEBUG_OUTPUT
    printf("disableAllCallAddress\n");
#endif

    byte mode1Reg = readRegister(PCA9685_MODE1_REG);
    writeRegister(PCA9685_MODE1_REG, (mode1Reg &= ~PCA9685_MODE1_ALLCALL));
}

void disableSub1Address() {
    if (_isProxyAddresser) return;

#ifdef PCA9685_ENABLE_DEBUG_OUTPUT
    printf("disableSub1Address\n");
#endif

    byte mode1Reg = readRegister(PCA9685_MODE1_REG);
    writeRegister(PCA9685_MODE1_REG, (mode1Reg &= ~PCA9685_MODE1_SUBADR1));
}

void PCdisableSub2Address() {
    if (_isProxyAddresser) return;

#ifdef PCA9685_ENABLE_DEBUG_OUTPUT
    printf("disableSub2Address\n");
#endif

    byte mode1Reg = readRegister(PCA9685_MODE1_REG);
    writeRegister(PCA9685_MODE1_REG, (mode1Reg &= ~PCA9685_MODE1_SUBADR2));
}


void PdisableSub3Address() {
    if (_isProxyAddresser) return;

#ifdef PCA9685_ENABLE_DEBUG_OUTPUT
    printf("disableSub3Address\n");
#endif

    byte mode1Reg = readRegister(PCA9685_MODE1_REG);
    writeRegister(PCA9685_MODE1_REG, (mode1Reg &= ~PCA9685_MODE1_SUBADR3));
}

void enableExtClockLine() {
#ifdef PCA9685_ENABLE_DEBUG_OUTPUT
    printf("enableExtClockLine\n");
#endif

    // The PRE_SCALE register can only be set when the SLEEP bit of MODE1 register is set to logic 1.
    byte mode1Reg = readRegister(PCA9685_MODE1_REG);
    writeRegister(PCA9685_MODE1_REG, (mode1Reg = (mode1Reg & ~PCA9685_MODE1_RESTART) | PCA9685_MODE1_SLEEP));
    writeRegister(PCA9685_MODE1_REG, (mode1Reg |= PCA9685_MODE1_EXTCLK));

    // It takes 500us max for the oscillator to be up and running once SLEEP bit has been set to logic 0.
    writeRegister(PCA9685_MODE1_REG, (mode1Reg = (mode1Reg & ~PCA9685_MODE1_SLEEP) | PCA9685_MODE1_RESTART));
    delay(50000);//this is 500 mseconds
}


byte getLastI2CError() {
    return _lastI2CError;
}


void getPhaseCycle(int channel, uint16_t pwmAmount, uint16_t *phaseBegin, uint16_t *phaseEnd) {
    if (channel == PCA9685_ALLLED_CHANNEL) {
        *phaseBegin = 0; // ALLLED should not receive a phase shifted begin value
    } else {
        // Get phase delay begin
        switch(_phaseBalancer) {
            case PCA9685_PhaseBalancer_None:
            case PCA9685_PhaseBalancer_Count:
            case PCA9685_PhaseBalancer_Undefined:
                *phaseBegin = 0;
                break;

            case PCA9685_PhaseBalancer_Linear:
                // Distribute high phase area over more of the duty cycle range to balance load
                *phaseBegin = (channel * ((4096 / 16) / 16)) & PCA9685_PWM_MASK;
                break;
        }
    }

    // See datasheet section 7.3.3
    if (pwmAmount == 0) {
        // Full OFF -> time_end[bit12] = 1
        *phaseEnd = PCA9685_PWM_FULL;
    }
    else if (pwmAmount >= PCA9685_PWM_FULL) {
        // Full ON -> time_beg[bit12] = 1, time_end[bit12] = <ignored>
        *phaseBegin |= PCA9685_PWM_FULL;
        *phaseEnd = 0;
    }
    else {
        *phaseEnd = (*phaseBegin + pwmAmount) & PCA9685_PWM_MASK;
    }
}


void writeChannelBegin(int channel) {
    byte regAddress;

    if (channel != PCA9685_ALLLED_CHANNEL)
        regAddress = PCA9685_LED0_REG + (channel * 0x04);
    else
        regAddress = PCA9685_ALLLED_REG;

#ifdef PCA9685_ENABLE_DEBUG_OUTPUT
    printf(" writeChannelBegin channel: ");
    printf("%d",channel);
    printf(", regAddress: 0x");
    printf("%X\n",regAddress);
#endif

    HAL_I2C_Master_Transmit(&hi2c1,(uint16_t)(_i2cAddress<<1),&regAddress,1, HAL_MAX_DELAY);
}

void writeChannelPWM(uint16_t phaseBegin, uint16_t phaseEnd) {
#ifdef PCA9685_ENABLE_DEBUG_OUTPUT
    printf("  PCA9685::writeChannelPWM phaseBegin: ");
    printf("%d",phaseBegin);
    printf(", phaseEnd: ");
    printf("%d\n",phaseEnd);
#endif

#ifndef PCA9685_SWAP_PWM_BEG_END_REGS

    uint8_t phaseBegin_l, phaseBegin_h, phaseEnd_l,phaseEnd_h;

    phaseBegin_l = phaseBegin &0xFF;
    phaseBegin_h = phaseBegin &0xFF;
    phaseEnd_l = phaseEnd &0xFF;
    phaseEnd_h = phaseEnd &0xFF;


    HAL_I2C_Master_Transmit(&hi2c1,(uint16_t)(_i2cAddress<<1),&phaseBegin_l,1, HAL_MAX_DELAY);
    HAL_I2C_Master_Transmit(&hi2c1,(uint16_t)(_i2cAddress<<1),&phaseBegin_h,1, HAL_MAX_DELAY);
    HAL_I2C_Master_Transmit(&hi2c1,(uint16_t)(_i2cAddress<<1),&phaseEnd_l,1, HAL_MAX_DELAY);
    HAL_I2C_Master_Transmit(&hi2c1,(uint16_t)(_i2cAddress<<1),&phaseEnd_h,1, HAL_MAX_DELAY);

#else
       HAL_I2C_Master_Transmit(&hi2c1,(uint16_t)(_i2cAddress<<1),&phaseEnd_l,1, HAL_MAX_DELAY);
       HAL_I2C_Master_Transmit(&hi2c1,(uint16_t)(_i2cAddress<<1),&phaseEnd_h,1, HAL_MAX_DELAY);
       HAL_I2C_Master_Transmit(&hi2c1,(uint16_t)(_i2cAddress<<1),&phaseBegin_l,1, HAL_MAX_DELAY);
       HAL_I2C_Master_Transmit(&hi2c1,(uint16_t)(_i2cAddress<<1),&phaseBegin_h,1, HAL_MAX_DELAY);

#endif
}

void writeChannelEnd() {
    i2cWire_endTransmission();

#ifdef PCA9685_ENABLE_DEBUG_OUTPUT
    checkForErrors();
#endif
}


void PwriteRegister(byte regAddress, byte value) {
#ifdef PCA9685_ENABLE_DEBUG_OUTPUT
    printf("  writeRegister regAddress: 0x");
    printf("%X",regAddress);
    printf(", value: 0x");
    printf("%X",value);
#endif

    HAL_I2C_Master_Transmit(&hi2c1,(uint16_t)(_i2cAddress<<1),&regAddress,1, HAL_MAX_DELAY);
    HAL_I2C_Master_Transmit(&hi2c1,(uint16_t)(_i2cAddress<<1),&value,1, HAL_MAX_DELAY);



#ifdef PCA9685_ENABLE_DEBUG_OUTPUT
    checkForErrors();
#endif
}

byte readRegister(byte regAddress) {
#ifdef PCA9685_ENABLE_DEBUG_OUTPUT
    printf(" readRegister regAddress: 0x");
    printf("%X",regAddress);
#endif

    if(HAL_I2C_Master_Transmit(&hi2c1,(uint16_t)(_i2cAddress<<1),&regAddress,1, HAL_MAX_DELAY)!=HAL_OK)
		{
#ifdef PCA9685_ENABLE_DEBUG_OUTPUT
        checkForErrors();
#endif
        return 0;
    }

    int bytesRead = i2cWire_requestFrom((uint8_t)_i2cAddress, 1);
    if (bytesRead != 1) {
        while (bytesRead-- > 0)
            i2cWire_read();
#ifdef PCA9685_USE_SOFTWARE_I2C
        PCA9685_i2c_stop(); // Manually have to send stop bit in software i2c mode
#endif
        _lastI2CError = 4;
#ifdef PCA9685_ENABLE_DEBUG_OUTPUT
        checkForErrors();
#endif
        return 0;
    }

    byte retVal = i2cWire_read();

#ifdef PCA9685_USE_SOFTWARE_I2C
    PCA9685_i2c_stop(); // Manually have to send stop bit in software i2c mode
#endif

#ifdef PCA9685_ENABLE_DEBUG_OUTPUT
    printf("   readRegister retVal: 0x");
    printf("%X",retVal);
#endif

    return retVal;
}


void i2cWire_begin() {
    _lastI2CError = 0;
#ifndef PCA9685_USE_SOFTWARE_I2C
    hi2c1.Init.ClockSpeed = getI2CSpeed();
#endif
}


void i2cWire_beginTransmission(uint8_t addr) {
    _lastI2CError = 0;
#ifndef PCA9685_USE_SOFTWARE_I2C
    _i2cWire->beginTransmission(addr);
#else
    i2c_start(addr);
#endif
}

uint8_t i2cWire_endTransmission(void) {
#ifndef PCA9685_USE_SOFTWARE_I2C
    return (_lastI2CError = _i2cWire->endTransmission());
#else
    PCA9685_i2c_stop(); // Manually have to send stop bit in software i2c mode
    return (_lastI2CError = 0);
#endif
}

uint8_t i2cWire_requestFrom(uint8_t addr, uint8_t len) {
#ifndef PCA9685_USE_SOFTWARE_I2C
    return _i2cWire->requestFrom(addr, (size_t)len);
#else
    i2c_start(addr | 0x01);
    return (_readBytes = len);
#endif
}

uint8_t i2cWire_write(uint8_t data) {
#ifndef PCA9685_USE_SOFTWARE_I2C
    return _i2cWire->write(data);
#else
    return (size_t)PCA9685_i2c_write(data);
#endif
}


uint8_t i2cWire_read(void) {
#ifndef PCA9685_USE_SOFTWARE_I2C
    return (uint8_t)(_i2cWire->read() & 0xFF);
#else
    if (_readBytes > 1) {
        _readBytes -= 1;
        return (uint8_t)(i2c_read(false) & 0xFF);
    }
    else {
        _readBytes = 0;
        return (uint8_t)(i2c_read(true) & 0xFF);
    }
#endif
}

#ifdef PCA9685_ENABLE_DEBUG_OUTPUT

int getWireInterfaceNumber() {
#ifndef PCA9685_USE_SOFTWARE_I2C
    if (_i2cWire == &Wire) return 0;
#if WIRE_INTERFACES_COUNT > 1
    if (_i2cWire == &Wire1) return 1;
#endif
#if WIRE_INTERFACES_COUNT > 2
    if (_i2cWire == &Wire2) return 2;
#endif
#if WIRE_INTERFACES_COUNT > 3
    if (_i2cWire == &Wire3) return 3;
#endif
#if WIRE_INTERFACES_COUNT > 4
    if (_i2cWire == &Wire4) return 4;
#endif
#if WIRE_INTERFACES_COUNT > 5
    if (_i2cWire == &Wire5) return 5;
#endif
#endif // /ifndef PCA9685_USE_SOFTWARE_I2C
    return -1;
}


static const char *textForWireInterfaceNumber(int wireNum) {
#ifndef PCA9685_USE_SOFTWARE_I2C
    switch (wireNum) {
        case 0: return "Wire";
        case 1: return "Wire1";
        case 2: return "Wire2";
        case 3: return "Wire3";
        case 4: return "Wire4";
        case 5: return "Wire5";
        default: return "<CustomWire>";
    }
#else
    return "SoftwareI2C";
#endif // /ifndef PCA9685_USE_SOFTWARE_I2C
}


void printModuleInfo() {
    printf("\n");
    printf(" ~~~ PCA9685 Module Info ~~~\n");

    printf("\n");
    printf("i2c Address: ");
    printf("0x");
    printf("%X\n",_i2cAddress);
    printf("i2c Instance: ");
    printf("%d",getWireInterfaceNumber());
    printf(": ");
    printf("%d\n"textForWireInterfaceNumber(getWireInterfaceNumber()));
    printf("i2c Speed: ");
    printf("%d",roundf(getI2CSpeed() / 1000.0f));
    printf("kHz\n");

    printf("\n");
    printf("Phase Balancer: ");
    printf("%d",_phaseBalancer);
    printf(": ");
    switch (_phaseBalancer) {
        case PCA9685_PhaseBalancer_None:
            printf("PCA9685_PhaseBalancer_None\n"); break;
        case PCA9685_PhaseBalancer_Linear:
            printf("PCA9685_PhaseBalancer_Linear\n"); break;
        case PCA9685_PhaseBalancer_Count:
        case PCA9685_PhaseBalancer_Undefined:
            printf("\n"); break;
    }

    if (!_isProxyAddresser) {
    	printf("\n");
    	printf("Proxy Addresser: false\n");

    	printf("\n");
    	printf("Mode1 Register:\n");
    	const byte mode1Reg = readRegister(PCA9685_MODE1_REG);
    	printf("0x%X, Bitset:", mode1Reg); // Print in hexadecimal format
    	// Bitwise checking and printing bitset information
    	if (mode1Reg & PCA9685_MODE1_RESTART)
    	    printf(" PCA9685_MODE1_RESTART");
    	if (mode1Reg & PCA9685_MODE1_EXTCLK)
    	    printf(" PCA9685_MODE1_EXTCLK");
    	if (mode1Reg & PCA9685_MODE1_AUTOINC)
    	    printf(" PCA9685_MODE1_AUTOINC");
    	if (mode1Reg & PCA9685_MODE1_SLEEP)
    	    printf(" PCA9685_MODE1_SLEEP");
    	if (mode1Reg & PCA9685_MODE1_SUBADR1)
    	    printf(" PCA9685_MODE1_SUBADR1");
    	if (mode1Reg & PCA9685_MODE1_SUBADR2)
    	    printf(" PCA9685_MODE1_SUBADR2");
    	if (mode1Reg & PCA9685_MODE1_SUBADR3)
    	    printf(" PCA9685_MODE1_SUBADR3");
    	if (mode1Reg & PCA9685_MODE1_ALLCALL)
    	    printf(" PCA9685_MODE1_ALLCALL");
    	printf("\n");

    	printf("\n");
    	printf("Mode2 Register:\n");
    	const byte mode2Reg = readRegister(PCA9685_MODE2_REG);
    	printf("0x%X, Bitset:", mode2Reg); // Print in hexadecimal format
    	if (mode2Reg & PCA9685_MODE2_OUTDRV_TPOLE)
    	    printf(" PCA9685_MODE2_OUTDRV_TPOLE");
    	if (mode2Reg & PCA9685_MODE2_INVRT)
    	    printf(" PCA9685_MODE2_INVRT");
    	if (mode2Reg & PCA9685_MODE2_OUTNE_TPHIGH)
    	    printf(" PCA9685_MODE2_OUTNE_TPHIGH");
    	if (mode2Reg & PCA9685_MODE2_OUTNE_HIGHZ)
    	    printf(" PCA9685_MODE2_OUTNE_HIGHZ");
    	if (mode2Reg & PCA9685_MODE2_OCH_ONACK)
    	    printf(" PCA9685_MODE2_OCH_ONACK");
    	printf("\n\n");

    	printf("SubAddress1 Register:\n");
    	const byte subAdr1Reg = readRegister(PCA9685_SUBADR1_REG);
    	printf("0x%X\n\n", subAdr1Reg);

    	printf("SubAddress2 Register:\n");
    	const byte subAdr2Reg = readRegister(PCA9685_SUBADR2_REG);
    	printf("0x%X\n\n", subAdr2Reg);

    	printf("SubAddress3 Register:\n");
    	const byte subAdr3Reg = readRegister(PCA9685_SUBADR3_REG);
    	printf("0x%X\n\n", subAdr3Reg);

    	printf("AllCall Register:\n");
    	const byte allCallReg = readRegister(PCA9685_ALLCALL_REG);
    	printf("0x%X\n\n", allCallReg);
    } else {
    	printf("\nProxy Addresser: true\n");
    }
}

static const char *textForI2CError(byte errorCode) {
    switch (errorCode) {
    case 0:
        return "Success";
    case 1:
        return "Data too long to fit in transmit buffer";
    case 2:
        return "Received NACK on transmit of address";
    case 3:
        return "Received NACK on transmit of data";
    default:
        return "Other error";
    }
}

void checkForErrors() {
    if (_lastI2CError) {
    	printf("  checkErrors lastI2CError: %d: %s\n", _lastI2CError, textForI2CError(getLastI2CError()));
    }
}
#endif // /ifdef PCA9685_ENABLE_DEBUG_OUTPUT


_coeff=NULL;
_isCSpline=false;

PCA9685_ServoEval_1(uint16_t minPWMAmount, uint16_t maxPWMAmount)

{
    minPWMAmount = MIN(minPWMAmount, PCA9685_PWM_FULL);


    if (maxPWMAmount < minPWMAmount) {
        maxPWMAmount = minPWMAmount;
    } else if (maxPWMAmount > PCA9685_PWM_FULL) {
        maxPWMAmount = PCA9685_PWM_FULL;
    }

    _coeff = (float*)malloc(2 * sizeof(float));
    _isCSpline = false;

    _coeff[0] = minPWMAmount;
    _coeff[1] = (maxPWMAmount - minPWMAmount) / 180.0f;
}


_coeff=NULL;
_isCSpline=false;

void PCA9685_ServoEval_2(uint16_t minPWMAmount, uint16_t midPWMAmount, uint16_t maxPWMAmount)


{
    minPWMAmount = MIN(minPWMAmount, PCA9685_PWM_FULL);



    if (midPWMAmount < minPWMAmount) {
               midPWMAmount = minPWMAmount;
           } else if (midPWMAmount > PCA9685_PWM_FULL) {
               midPWMAmount = PCA9685_PWM_FULL;
           }


    if (maxPWMAmount < minPWMAmount) {
             maxPWMAmount = minPWMAmount;
         } else if (maxPWMAmount > PCA9685_PWM_FULL) {
             maxPWMAmount = PCA9685_PWM_FULL;
         }

    if (maxPWMAmount - midPWMAmount != midPWMAmount - minPWMAmount) {
    	_coeff = (float*)malloc(8 * sizeof(float));
        _isCSpline = true;

        // Cubic spline code adapted from: https://shiftedbits.org/2011/01/30/cubic-spline-interpolation/
        /* "THE BEER-WARE LICENSE" (Revision 42): Devin Lane wrote this [part]. As long as you retain
        * this notice you can do whatever you want with this stuff. If we meet some day, and you
        * think this stuff is worth it, you can buy me a beer in return. */
       // TODO: Looks like I owe Devin Lane a beer. -NR

        float x[3] = { 0, 90, 180 };
        float y[3] = { (float)minPWMAmount, (float)midPWMAmount, (float)maxPWMAmount };
        float c[3], b[2], d[2], h[2], l[1], u[2], a[1], z[2]; // n = 3

        h[0] = x[1] - x[0];
        u[0] = z[0] = 0;
        c[2] = 0;

        for (int i = 1; i < 2; ++i) {
            h[i] = x[i + 1] - x[i];
            l[i - 1] = (2 * (x[i + 1] - x[i - 1])) - h[i - 1] * u[i - 1];
            u[i] = h[i] / l[i - 1];
            a[i - 1] = (3 / h[i]) * (y[i + 1] - y[i]) - (3 / h[i - 1]) * (y[i] - y[i - 1]);
            z[i] = (a[i - 1] - h[i - 1] * z[i - 1]) / l[i - 1];
        }

        for (int i = 1; i >= 0; --i) {
            c[i] = z[i] - u[i] * c[i + 1];
            b[i] = (y[i + 1] - y[i]) / h[i] - (h[i] * (c[i + 1] + 2 * c[i])) / 3;
            d[i] = (c[i + 1] - c[i]) / (3 * h[i]);

            _coeff[4 * i + 0] = y[i]; // a
            _coeff[4 * i + 1] = b[i]; // b
            _coeff[4 * i + 2] = c[i]; // c
            _coeff[4 * i + 3] = d[i]; // d
        }
    }
    else {
    	 _coeff = (float*)malloc(2 * sizeof(float));
        _isCSpline = false;

        _coeff[0] = minPWMAmount;
        _coeff[1] = (maxPWMAmount - minPWMAmount) / 180.0f;
    }
}

uint16_t pwmForAngle(float angle) {
    float retVal;
    angle = constrain(angle + 90, 0, 180);

    if (!_isCSpline) {
        retVal = _coeff[0] + (_coeff[1] * angle);
    }
    else {
        if (angle <= 90) {
            retVal = _coeff[0] + (_coeff[1] * angle) + (_coeff[2] * angle * angle) + (_coeff[3] * angle * angle * angle);
        }
        else {
            angle -= 90;
            retVal = _coeff[4] + (_coeff[5] * angle) + (_coeff[6] * angle * angle) + (_coeff[7] * angle * angle * angle);
        }
    }

    return (uint16_t)MIN((uint16_t)round(retVal), PCA9685_PWM_FULL);
};

uint16_t pwmForSpeed(float speed) {
    return pwmForAngle(speed * 90.0f);
}
