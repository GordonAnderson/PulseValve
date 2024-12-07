#ifndef PULSEVALVE_H
#define PULSEVALVE_H

#define FILTER        0.1
#define SIGNATURE     0xAA55A5A5

// Processor output controls
#define HVsetPin   11
#define PVpulsePin 9
#define PVpwrPin   7
#define PV12VPin   5
#define HVrelayPin 22
#define FaultPin   13
#define OnPin      12
// Processor inputs
#define TrigPin    2
#define ADC3p3V    A0
#define ADC5V      A1
#define ADC12V     A2
#define ADCpwr     A5
#define ADCHVmon   A4

// IO macros
#define RelayClosed    digitalWrite(HVrelayPin,   HIGH)
#define RelayOpen      digitalWrite(HVrelayPin,   LOW)
#define ONon           digitalWrite(OnPin,        LOW)
#define ONoff          digitalWrite(OnPin,        HIGH)
#define FaultOn        digitalWrite(FaultPin,     LOW)
#define FaultOff       digitalWrite(FaultPin,     HIGH)
#define PVon           digitalWrite(PVpulsePin,   HIGH)
#define PVoff          digitalWrite(PVpulsePin,   LOW)
#define PVpwrOn        digitalWrite(PVpwrPin,     LOW)
#define PVpwrOff       digitalWrite(PVpwrPin,     HIGH)
#define PV12Von        digitalWrite(PV12VPin,     LOW)
#define PV12Voff       digitalWrite(PV12VPin,     HIGH)

enum Modes
{
  Mode_disable,
  Mode_burst,
  Mode_follow,
  Mode_manual
};

typedef struct
{
  int16_t       Size;                   // This data structures size in bytes
  char          Name[20];               // Holds the module name
  int8_t        Rev;                    // Holds the board revision number
  // Pulse valve controller parameters
  Modes         mode;
  // Burst mode
  int           pulseWidth;             // Pulse witch in uS
  int           burstNum;               // Number if pulses per burst
  int           burstDelay;             // Delay in mS between pulses in a burst
  int           period;                 // Period in mS for repeated burts
  int           cycles;                 // Number os burst cycles
  float         HVsetpoint;             // Pulse HV value in volts
  // Misc control parameters
  bool          activeLow;              // If true trigger input is active low
  int           reduceDelay;            // Delay in mS from 24 volt PV turn on to reduce voltage to 12V
  // Calibration structures
  DACchan       HVdac;                  // PMW analog out to control HV
  ADCchan       HVadc;                  // ADC analog input to monitor HV
  // Data structure signature
  unsigned int  Signature;              // Must be 0xAA55A5A5 for valid data
} PulseValve;

void setMode(void);
void getMode(void);
void genBurst(void);
void genPulse(void);
void setOpen(void);
void setClosed(void);
void getState(void);

void SaveSettings(void);
void RestoreSettings(void);
void FormatFLASH(void);
void bootloader(void);

#endif

