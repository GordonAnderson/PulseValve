
//
// Pulse Valve Controller
//
// Modes:
//  Disabled
//  Programable burst mode, high speed mode
//    - external trigger
//    - Define pulse width
//    - Define pulse voltage
//    - Time between pulse
//    - Number of pulses
//    - nuber of burst
//  Follow mode
//    - PV follows input trigger
//    - 24V on time untill drop to 12V
//    - Invert option
//  Manual
//    ON or OFF
//    Pulse

#include "Arduino.h"
//#include "Hardware.h"
#include <arduino-timer.h>
#include "SAMD51_InterruptTimer.h"

#include <RingBuffer.h>
#include <commandProcessor.h>
#include <charAllocate.h>
#include <debug.h>
#include <Errors.h>
#include <Devices.h>
#include <Adafruit_SPIFlash.h>
#include <FlashStorage.h>
#include <FlashAsEEPROM.h>

#include "PulseValve.h"

commandProcessor cp;
debug dbg(&cp);

const char *Version = "Pulse Valve Driver, version 1.0 October 17, 2024";
auto timer = timer_create_default();
float HVrb = 0;

// Reserve a portion of flash memory to store configuration
// Note: the area of flash memory reserved is cleared every time
// the sketch is uploaded on the board.
FlashStorage(flash_pv, PulseValve);

PulseValve pv;

PulseValve Rev_1_pv = 
{
    sizeof(PulseValve),"PulseValve",1,
    Mode_follow,
    200,5,5,100,10,350,
    false,2,
    {HVsetPin,532.8,-148118.4},
    {ADCHVmon,99.91,0},
    SIGNATURE
};

const int   BPWrange[2] = {100,500};
const int   BNrange[2]  = {1,20};
const int   BDrange[2]  = {5,100};
const int   BPrange[2]  = {10,10000};
const int   BCrange[2]  = {1,10000};
const float BPVrange[2] = {280,400};
const int   RedRange[2] = {1,1000};

Command pvCmds[] =
{
  // Main application commands
  {"GVER", CMDstr,-1,(void *)Version,NULL,                      "Firmware version"},
  {"?NAME",CMDstr,-1,(void *)pv.Name,NULL,                      "Name of this device"},
  {"SAVE",CMDfunction,   0, (void *)SaveSettings,NULL,          "Save settings"},
  {"RESTORE",CMDfunction,0, (void *)RestoreSettings,NULL,       "Restore settings"},
  {"FORMAT",CMDfunction, 0, (void *)FormatFLASH,NULL,           "Format FLASH"},
  {"BLOAD",CMDfunction,  0, (void *)bootloader,NULL,            "Jump to bootloader"},
 // Pulse valve commands
  {"SMODE",CMDfunction,1,(void *)setMode,NULL,                  "Set mode, DISABLE|BURST|FOLLOW|MANUAL"},
  {"GMODE",CMDfunction,0,(void *)getMode,NULL,                  "Report mode, DISABLE|BURST|FOLLOW|MANUAL"},
  {"GSTATE",CMDfunction,0,(void *)getState,NULL,                "Report valve state, OPEN|CLOSED"},
  {"?RED",CMDint,-1,(void *)&pv.reduceDelay,(void *)&RedRange,  "Pulse voltage reduction delay, in mS"},
  // Burst mode commands
  {"?BPW",CMDint,-1,(void *)&pv.pulseWidth,(void *)&BPWrange,   "Pulse width in uS"},
  {"?BPV",CMDfloat,-1,(void *)&pv.HVsetpoint,(void *)&BPVrange, "Pulse voltage in volts"},
  {"GBPVA",CMDfloat,0,(void *)&HVrb,NULL,                       "Pulse voltage readback in volts"},
  {"?BN",CMDint,-1,(void *)&pv.burstNum,(void *)&BNrange,       "Number of pulses in burst"},
  {"?BD",CMDint,-1,(void *)&pv.burstDelay,(void *)&BDrange,     "Pulse to pulse delay in a burst, in mS"},
  {"?BP",CMDint,-1,(void *)&pv.period,(void *)&BPrange,         "Pulse burst period, in mS"},
  {"?BC",CMDint,-1,(void *)&pv.cycles,(void *)&BCrange,         "Pulse burst number of cycles"},
  // Manual pulse valve command
  {"OPEN",CMDfunction,0,(void *)setOpen,NULL,                   "Open the pulse valve"},
  {"CLOSE",CMDfunction,0,(void *)setClosed,NULL,                "Close the pulse valve"},
  {"BURST",CMDfunction,0,(void *)genBurst,NULL,                 "Generate a pulse burst"},
  {"PULSE",CMDfunction,-1,(void *)genPulse,NULL,                "Generate a pulse, uS"},
  {NULL}
};

static CommandList pvList = {pvCmds, NULL};

void Debug(void)
{
}

volatile int pulse;
volatile int cycle;

void generatePulse(int width)
{
  PVon;
  delayMicroseconds(width);
  PVoff;
}

void nextPulse(void)
{
   if((pulse == 0) && (pv.burstNum > 1)) TC.startTimer(pv.burstDelay * 1000, nextPulse);
   generatePulse(pv.pulseWidth);
   pulse++;
   if(pulse >= pv.burstNum)
   {
    cycle++;
    TC.stopTimer();
    if(cycle < pv.cycles)
    {
      pulse = 0;
      TC.startTimer((pv.period - (pv.burstNum - 1) * pv.burstDelay) * 1000 - pv.pulseWidth, nextPulse);
    }
    else ONoff;
   }
}

void generateBurst(void)
{
  ONon;
  if(pv.burstNum > 1) TC.startTimer(pv.burstDelay * 1000, nextPulse);
  pulse = 0;
  cycle = 0;
  nextPulse();
}

void lowerDrive(void) 
{
  PV12Von;
  PVpwrOff;
  TC.stopTimer();
}

void triggerISR(void)
{
  delayMicroseconds(1);
  int state = digitalRead(TrigPin);
  switch(pv.mode)
  {
    case Mode_disable:
      PVoff;
      break;
    case Mode_burst:
      if(!pv.activeLow)
      {
        if(state == HIGH) generateBurst();
      }
      else
      {
        if(state == LOW) generateBurst();
      }
      break;
    case Mode_follow:
      if(!pv.activeLow)
      {
        if(state == HIGH) PVon;
        else {PVoff; PVpwrOn;}
      }
      else
      {
        if(state == LOW) PVon;
        else {PVoff; PVpwrOn;}
      }
      // Start delay to lower drive voltage to 12V
      if(((!pv.activeLow) && (state == HIGH)) || ((pv.activeLow) && (state == LOW)))
      {
        if(pv.reduceDelay > 0) TC.startTimer(pv.reduceDelay * 1000, lowerDrive);
      }
      break;
    case Mode_manual:
      break;
    default:
      break;
  }
}

bool update(void *)
{
  // Read the ADC and update the HV readback value
  int i = 0;
  for(int j=0;j<16;j++) i += analogRead(pv.HVadc.Chan);
  float fval = Counts2Value(i,&pv.HVadc);
  if(HVrb == -1) HVrb = fval;
  else HVrb = FILTER * fval + (1 - FILTER) * HVrb;
  // Write the PWM analog out for the HV setpoint
  analogWrite(pv.HVdac.Chan,Value2Counts(pv.HVsetpoint,&pv.HVdac)/256);
  return true;
}

void setup() 
{
  // Init all digital outputs
  pinMode(HVsetPin,   OUTPUT); digitalWrite(HVsetPin,   LOW);
  pinMode(PVpulsePin, OUTPUT); PVoff;
  pinMode(PVpwrPin,   OUTPUT); PVpwrOff;
  pinMode(PV12VPin,   OUTPUT); PV12Voff;
  pinMode(HVrelayPin, OUTPUT); RelayOpen;

  pinMode(FaultPin, OUTPUT); FaultOff;
  pinMode(OnPin, OUTPUT); ONoff;

  // Set resolutions
  analogReadResolution(12);
  analogWriteResolution(8);
  analogWrite(HVsetPin, 0);

  // Read the flash config contents and test the signature
  pv = flash_pv.read();
  if(pv.Signature != SIGNATURE) pv = Rev_1_pv;

  // Setup the communications
  Serial.begin(0);
  cp.registerStream(&Serial);
  cp.selectStream(&Serial);
  cp.registerCommands(&pvList);
  cp.registerCommands(dbg.debugCommands());
  dbg.registerDebugFunction(Debug);

  timer.every(100, update);

  attachInterrupt(digitalPinToInterrupt(TrigPin), triggerISR, CHANGE);
}

void loop() 
{
  static Modes currentMode = (Modes)-1;

  cp.processStreams();
  cp.processCommands();

  timer.tick();

  if(currentMode != pv.mode) 
  {
    currentMode = pv.mode;
    switch(pv.mode)
    {
      case Mode_disable:
        RelayOpen;
        PVpwrOff;
        PV12Voff;
        break;
      case Mode_burst:
        RelayClosed;
        PV12Von;
        PVpwrOff;
        break;
      case Mode_follow:
        RelayOpen;
        PVpwrOn;
        PV12Von;
        break;
      case Mode_manual:
        RelayOpen;
        PVpwrOn;
        PV12Von;
        break;
      default:
        break;
    }
  }
}

void setMode(void)
{
  char *res;
  if(!cp.checkExpectedArgs(1)) return;
  if(!cp.getValue(&res, (char *)"DISABLE,BURST,FOLLOW,MANUAL")) {cp.sendNAK(ERR_BADARG); return;}
  String token = res;
  if(token == "DIABLE") pv.mode = Mode_disable;
  else if(token == "BURST") pv.mode = Mode_burst;
  else if(token == "FOLLOW") pv.mode = Mode_follow;
  else if(token == "MANUAL") pv.mode = Mode_manual;
  cp.sendACK();
}

void getMode(void)
{
  if(!cp.checkExpectedArgs(0)) return;
  cp.sendACK(false);
  if(pv.mode == Mode_disable)     cp.println("DISABLE");
  else if(pv.mode == Mode_burst)  cp.println("BURST");
  else if(pv.mode == Mode_follow) cp.println("FOLLOW");
  else if(pv.mode == Mode_manual) cp.println("MANUAL");
}

void genBurst(void)
{
  if(!cp.checkExpectedArgs(0)) return;
  generateBurst();
  cp.sendACK();
}

void genPulse(void)
{
  int uS;
  if(!cp.checkExpectedArgs(1)) return;
  if(!cp.getValue(&uS, 1,1000000)) {cp.sendNAK(ERR_BADARG); return;}
  PVon;
  ONon;
  delayMicroseconds(uS);
  ONoff;
  PVoff;
  cp.sendACK();
}

void setOpen(void)
{
  PVon;
  ONon;
  cp.sendACK();
}

void setClosed(void)
{
  ONoff;
  PVoff;
  cp.sendACK();
}

void getState(void)
{
  cp.sendACK(false);
  if(digitalRead(PVpulsePin)==HIGH) cp.println("OPEN");
  else cp.println("CLOSED");
}

void SaveSettings(void)
{
  pv.Signature = SIGNATURE;
  flash_pv.write(pv);
  cp.sendACK();
}

void RestoreSettings(void)
{
  static PulseValve pvd;
  
  // Read the flash config contents and test the signature
  pvd = flash_pv.read();
  if(pvd.Signature == SIGNATURE) pv = pvd;
  else
  {
    cp.sendNAK();
    return;
  }
  cp.sendACK(); 
}

void FormatFLASH(void)
{
  flash_pv.write(Rev_1_pv);  
  cp.sendACK();
}

void bootloader(void)
{
  __disable_irq();
 	//THESE MUST MATCH THE BOOTLOADER
	#define DOUBLE_TAP_MAGIC 			      0xf01669efUL
	#define BOOT_DOUBLE_TAP_ADDRESS     (HSRAM_ADDR + HSRAM_SIZE - 4)

	unsigned long *a = (unsigned long *)BOOT_DOUBLE_TAP_ADDRESS;
	*a = DOUBLE_TAP_MAGIC;
	
	// Reset the device
	NVIC_SystemReset() ;

	while (true);
}
