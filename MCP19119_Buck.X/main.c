/*
 * Example code for the Microchip MCP19119 Digitally Enhanced Power Analog Buck 
 * Controller
 * 
 */

#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled and can be enabled by SWDTEN bit of the WDTCON register)
#pragma config PWRTE = ON       // Power-up Timer Enable bit (PWRT enabled)
#pragma config MCLRE = ON       // MCLR Pin Function Select bit (MCLR pin is MCLR function and weak internal pull-up is enabled)
#pragma config CP = OFF         // Code Protection bit (Program memory is not code protected)
#pragma config WRT = OFF        // Flash Program Memory Self Write Enable bits (Write protection off)

#include <xc.h>

#define I2C_ADDR 0x01

#define CH_AN0  0b10000
#define CH_AN1  0b10001
#define CH_AN2  0b10010
#define CH_AN3  0b10011

void Init_TMR2(void)
{
    // Device is clocked by factory calibrated precision 8 MHz internal 
    // oscillator.
    // Timer2 is used to set up the switching frequency.
    // Configurable between 100kHz and 1.6MHz
    T2CONbits.TMR2ON = 0;   // Ensure Timer is off
    T2CONbits.T2CKPS = 0;   // Prescaler 1:1
    PR2 = 0x0F;             // 8MHz / Prescaler 1 / 500kHz = 16;
    PWMRL = 0x0F;           // Maximum duty cycle 40%
    PWMPHL = 0x00;          // No phase shift
}

void Init_GPIO(void)
{
    
}

void Init_I2C(void)
{
    SSPADD = I2C_ADDR;          // Set slave address
    SSPCON1bits.SSPM = 0b0110;  // I2C slave mode, 7-bit address
    SSPCON3bits.SBCDE = 1;      // Enable slave bus collision interrupts
    SSPCON3bits.DHEN = 1;       // Data hold enable
    PIR1bits.SSPIF = 0;         // Clear interrupt flag
    SSPCON1bits.SSPEN = 1;      // Enable 
}

void Init_ADC(void)
{
    // 10-bit ADC. Full Scale is 5V. 
    ANSELAbits.ANSA0 = 1;       // AN0 Analog
    ANSELAbits.ANSA1 = 1;       // AN1 Analog
    ADCON1bits.ADCS = 0b101;    // ADC Conversion Clock = FOSC/16 (TAD 2.0uS)
    ADCON0bits.ADON = 1;        // Enable ADC
}

uint16_t Read_ADC(uint8_t channel)
{
    ADCON0bits.CHS = (channel & 0b11111);   // Select Channel
    ADCON0bits.GO_nDONE = 1;                // Start Conversion
    while (ADCON0bits.GO_nDONE);            // Wait for conversion to complete
    return(ADRESL + (ADRESH << 8));
}

void Load_Calibration(void)
{
    // Load Calibration Data from Flash Memory. Flash memory is not directly 
    // mapped and must be indirectly addressed via Program Memory (PMxxxx) 
    // Special Function Registers
    PMADRH = 0x20;           // Address high byte
    PMCON1bits.CALSEL = 1;   // Read calibration data from test memory
    // Calibration Word 1 0x8020:
    // Internal Oscillator (OSCCAL) & Offset for Output Voltage Remote Sense 
    // Differential Amplifier (DOVCAL)
    PMADRL = 0x80; PMCON1bits.RD = 1; NOP(); NOP(); DOVCAL = PMDATH; OSCCAL = PMDATL;
    // Calibration Word 2 0x8081:
    // Internal BandGap (BGRCAL) & Over-temperature Shutdown Threshold (TTACAL)
    PMADRL = 0x81; PMCON1bits.RD = 1; NOP(); NOP(); TTACAL = PMDATH; BGRCAL = PMDATL;
    // Calibration Word 3 0x8082:
    // Offset Error Amplifier (ZROCAL) & Offset Buffer Amplifier Output Voltage 
    // Regulation Reference Set Point (VROCAL)
    PMADRL = 0x82; PMCON1bits.RD = 1; NOP(); NOP(); VROCAL = PMDATH; ZROCAL = PMDATL;
    // Calibration Word 4 + 5: Internal Temperature Sensor
    
    // Calibration Word 6 + 7: System Input Voltage

    // Calibration Word 8: Offset voltage of unity gain buffer (mV)
    
    // Calibration Word 9 0x8088: Gain of differential amplifier
    //PMADRL = 0x88; PMCON1bits.RD = 1; NOP(); NOP(); x = PMDAT;
    // Calibration Word 10 0x8089: Offset voltage of differential amplifier
    //PMADRL = 0x89; PMCON1bits.RD = 1; NOP(); NOP(); y = PMDATL;
            
    // Calibration Word 11 0x808A: ADC Gain
    
    // Calibration Word 12 0x808B: ADC Offset   
    
}

void Load_Config(void)
{
    // Refer to Chapter 6 Configuring the MCP19118/19 
    
    // 6.1 Input Under-voltage Lockout 
    // Triggers VINIF Interrupt Flag
    // UVLO<5:0> = 26.5 * ln(UVLOSET_POINT/4)
    VINLVLbits.UVLO = 0x00;
    VINLVLbits.UVLOEN = 0;      // Disable UVLO
    
    // 6.2 Output Overcurrent Control Register
    // Triggers OCIF Interrupt Flag
    OCCONbits.OCLEB = 0b11;     // 780nS blanking
    OCCONbits.OOC = 0;          // 160mV drop
    OCCONbits.OCEN = 1;         // Output Overcurrent DAC is enabled
    
    // 6.3 Current Sense AC Gain
    // The current measured across the inductor is a square wave that is 
    // averaged by the capacitor (CS) connected between +ISEN and -ISEN. This 
    // very small voltage plus the ripple can be amplified by the current sense 
    // AC gain circuitry. 
    CSGSCONbits.CSGS = 0b1100;  // 17.5dB
    
    // 6.4 Current Sense DC Gain
    CSDGCONbits.CSDGEN = 0;     // DC gain current sense signal only read by ADC
    CSDGCONbits.CSDG = 0x00;    // 19.5dB
    
    // 6.5 Voltage for Zero Current
    VZCCON = 0x80;              // 0mV Offset
    
    // Compensation
    // Compensation Zero Frequency Setting bits:
    CMPZCONbits.CMPZF = 0b1100; // 18700 Hz    
    // Compensation Gain Setting bits:
    CMPZCONbits.CMPZG = 0b1100; // 7.23 dB

    // 6.7 Slope Compensation
    SLPCRCONbits.SLPG = 0x0;    
    SLPCRCONbits.SLPS = 0x4;    // 0.053 Vpp
    
    // 6.8 Master Error Signal Gain
    // The SLVGNCON register is configured in the multi-phase SLAVE device.
    //SLVGNCONbits.SLVGN = 0;     // -3.3dB
    
    // 6.9 MOSFET Drive Programmable Dead Time
    DEADCONbits.HDLY = 0xA;     // High Side, 71nS delay
    DEADCONbits.LDLY = 0xC;     // Low Side, 64nS delay
    
    // 6.10 Output Voltage Configuration
    // Two registers control the error amplifier reference voltage. The 
    // reference is coarsely set in 15 mV steps and then finely adjusted in 
    // 0.82 mV steps above the coarse setting. Higher output voltages can be 
    // achieved by using a voltage divider connected between the output and 
    // the +VSEN pin.
    // Set to 0.54V
    // Coarse Control = (VOUT/0.0158) ? 1
    OVCCON = 33;            // Coarse Control 0.5372V 
    // Fine Control = (VOUT ? VOUT_COARSE) / 0.0008
    OVFCONbits.OVF = 4;     // Fine Control
    OVFCONbits.VOUTON = 1;
    
    // 6.11 Output Under-voltage
    // Triggers UVIF Interrupt Flag 
    // OUV<7:0> = (VOUT_UV_Detect_Level) / 0.015
    OUVCON = 0x00;
    
    // 6.12 Output Overvoltage
    // Triggers OVIF Interrupt Flag
    // OOV<7:0> = (VOUT_OV_Detect_Level) / 0.015
    OOVCON = 0x00;
    
    // 6.14 Analog Peripheral Enable Control Register 
    PE1bits.DECON = 0;      // Diode Emulation Mode bit
    PE1bits.DVRSTR = 0;     // High-Side Drive Strength Configuration bit
    PE1bits.HDLYBY = 0;     // High-Side Dead Time Bypass Disabled
    PE1bits.LDLYBY = 0;     // Low-Side Dead Time Bypass Disabled
    PE1bits.PDEN = 0;       // -VSEN Weak Pull-Down Enable bit
    PE1bits.PUEN = 0;       // +VSEN Weak Pull-Up Enable bit
    PE1bits.UVTEE = 0;      // Output Under-voltage Accelerator Enable bit
    PE1bits.OVTEE = 0;      // Output Over-voltage Accelerator Enable bit
            
    // 6.15 Analog Block Enable Control Register
    ABECONbits.OVDCEN = 0;  // Output over-voltage DAC control bit
    ABECONbits.UVDCEN = 0;  // Output under-voltage DAC control bit
    ABECONbits.MEASEN = 0;  // Relative efficiency measurement control bit
    ABECONbits.SLCPBY = 0;  // Slope compensation enabled (not bypassed)    
    ABECONbits.CRTMEN = 1;  // Current measurement circuitry enabled
    ABECONbits.TMPSEN = 0;  // Internal temperature sensor 
    ABECONbits.RECIREN = 0; // Relative efficiency circuitry 
    ABECONbits.PATHEN = 1;  // Signal chain circuitry enabled
}

void Enable_Output(void)
{
    T2CONbits.TMR2ON = 1;       // Enable Timer (PWM)
    ATSTCONbits.DRVDIS = 0;     // Enable High-side and Low-side MOSFET drivers
    PE1bits.PUEN = 0;           // Disable +VSEN Weak Pull-Up Enable bit
}

void main(void)
{
    
    Init_TMR2();
    Init_GPIO();
    Init_ADC();
    Init_I2C();
    Load_Calibration();
    Load_Config();
    Enable_Output();

    while(1) {
        
       OVCCON = (uint8_t)Read_ADC(CH_AN0) / 4;
       
    };
}
