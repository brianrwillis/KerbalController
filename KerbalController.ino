/******************************************************************************************************
*   _  _____ ___ ___   _   _        ___ ___  _  _ _____ ___  ___  _    _    ___ ___                  
*  | |/ / __| _ \ _ ) /_\ | |      / __/ _ \| \| |_   _| _ \/ _ \| |  | |  | __| _ \                 
*  | ' <| _||   / _ \/ _ \| |__   | (_| (_) | .` | | | |   / (_) | |__| |__| _||   /                 
*  |_|\_\___|_|_\___/_/ \_\____|__ \___\___/|_|\_| |_| |_|_\\___/|____|____|___|_|_\   ___ _  _  ___ 
*  \ \    / /_ _| |  | |  |_ _/ __|  / __|/ _ \| | | | | |_   _|_ _/ _ \| \| / __|    |_ _| \| |/ __|
*   \ \/\/ / | || |__| |__ | |\__ \  \__ \ (_) | |_| |_| | | |  | | (_) | .` \__ \_    | || .` | (__ 
*    \_/\_/ |___|____|____|___|___/  |___/\___/|____\___/  |_| |___\___/|_|\_|___( )  |___|_|\_|\___|
*                                                                                |/               
******************************************************************************************************/
#include "LiquidCrystal.h"
#include "avr/io.h"
#include "avr/interrupt.h"

#define MEN 0         //Master enable from key switch
#define SWIN 1        //Input from switches to MCU
#define SR_DOUT 11    //Data output from MCU to shift registers
#define SR_CLK 13     //Clock for shift registers
#define SR_RST_L 22   //Reset
#define SR_RCLK 23    //Latch

#define DB0 10        //Debug bits
#define DB1 12

//Task periods (us)
#define ELLIPSES_LOCK_PER 1000000   //Speed of ellipses while locked
#define ELLIPSES_INIT_PER 500000    //Speed of ellipses while initializing
#define LCD_CNTL_PER 100000         //Period of LCD updates
#define JOY_CNTL_PER 500            //Period of joystick scanning
#define REG_CNTL_PER 1000           //Period of switch scanning and LED setting
#define RESET_REG_PER 40            //Duration LEDs are on - controls brightness of LEDs

//Task priorities - lower priority tasks will be preempt by higher priority tasks 
//Lower number is higher priority
//4-bit implementation: 0-15, 16-31, etc.
//System Tick Timer has priority 0 and attachInterrupt has priority 128: keep custom task priorities lower
#define LCD_CNTL_PRIO 192
#define JOY_CNTL_PRIO 176
#define REG_RESET_PRIO 160     //Must be lower priority than register controller task, but high enough to preempt other tasks
#define REG_CNTL_PRIO 144

#define LAMPHOLD 500          //Time in ms that LEDs are held on after all are set during lamp test
#define LAMPSEP 50            //Separation in ms of LED flashes during lamp test

#define LEDON_PER 750         //Time in ms that shielded switch LEDs flash on and off
#define LEDOFF_PER 250        

//Debounce periods of switches in ms
#define DBOUNCE_SW 1          //Switches
#define DBOUNCE_SQ 25         //Square buttons
#define DBOUNCE_SM 1          //Small circular buttons
#define DBOUNCE_LG 1          //Large circular buttons

//Detect if register output is switch or LED
#define IS_SWITCH(x) (RegOutputs[x] == 1)
#define IS_LED(x) (RegOutputs[x] == 0)

//Detect is switch is active low (Small circular buttons)
#define IS_ACTIVE_HIGH(x) (SwitchType[x] != 3)
#define IS_ACTIVE_LOW(x) (SwitchType[x] == 3)

//Detect if switch is a toggle switch or button
#define IS_TOGGLE(x) (SwitchType[x] == 1)

enum PROGSTATE {LOCKED, INIT, RUNNING};                                 //Current state of the controller
enum ORIENT {MAN, STBL, PROG, RETG, NML, ANML, RDI, RDO, TRGP, TRGR};   //Ship orientations
enum SHIELDSTATE {CLOSED, OPEN};                                        //Enablers for staging and aborting ship

//Array representing locations of switches and LEDs on shift register outputs.
// 0 = LED, 1 = Switch
const uint8_t RegOutputs[104] = {1, 0, 1, 0, 1, 0, 1, 0, 0, 0, 1, 1, 0, 0, 1, 1, 
                                 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 1, 0, 1, 0, 
                                 1, 0, 1, 0, 1, 0, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1,
                                 1, 1, 1, 1, 1, 1, 1, 0, 1, 0, 1, 1, 1, 1, 1, 1, 
                                 1, 1, 1, 1, 1, 0, 1, 1, 1, 0, 1, 0, 1, 0, 1, 0, 
                                 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0,
                                 0, 0, 0, 0, 0, 0, 0, 0};

//Array representing LED states
//0 = LED off, 1 = LED on
volatile uint8_t LEDOn[46] = {0};

//Arrays representing switch states
//0 = Switch off, 1 = Switch on 
volatile uint8_t SwitchOn[58] = {0};
volatile uint8_t SwitchOnPrev[58] = {0};

//Array representing locations of each style of switch
//1 = Switch, 2 = Square button, 3 = Small button , 4 = Large button
const uint8_t SwitchType[58] = {1, 1, 1, 1, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 1, 1, 
                                1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 3, 3, 
                                3, 3, 1, 4, 1, 2, 2, 2, 2, 2, 2, 2, 2, 2, 4, 1, 
                                3, 1, 1, 1, 1, 1, 1, 1, 1, 1};

//Array representing characters that each switch sends to game
const char SwitchChar[58] = {'0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F', 
                             'G', 'H', 'I', 'J', 'K', 'L', 'M', 'N', 'O', 'P', 'Q', 'R', 'S', 'T', 'U', 'V', 
                             'W', 'X', 'Y', 'Z', 'a', 'b', 'c', 'd', 'e', 'f', 'g', 'h', 'i', 'j', 'k', 'l', 
                             'm', 'n', 'o', 'p', 'q', 'r', 's', 't', 'u', 'v'};

volatile uint8_t EllipsesLoc = 6;       //LCD column to display Ellipses onto
volatile uint8_t EllipsisCnt = 0;       //Number of Ellipses currently displayed onto LCD
volatile uint8_t LampTestRunning = 0;
volatile uint8_t LEDOnCnt = 0;          //Current number of on LEDs for lamp test
volatile uint8_t StageFlashState = 0;   //Variables to control on-off times for protected switches' LEDs
volatile uint8_t AbortFlashState = 0;
volatile uint16_t cnt = 1000;  

volatile PROGSTATE ProgState = LOCKED;
volatile ORIENT Orient = STBL;
volatile SHIELDSTATE StageShield = CLOSED;
volatile SHIELDSTATE AbortShield = CLOSED;

const uint8_t LCD_RST = 2, LCD_EN = 3, D4 = 4, D5 = 5, D6 = 6, D7 = 7;  //Set LCD pins
LiquidCrystal LCD(LCD_RST, LCD_EN, D4, D5, D6, D7);                     //Create LCD object

IntervalTimer EllipsesTimer;
IntervalTimer LCDTimer;
IntervalTimer JoyTimer;
IntervalTimer RegTimer;
IntervalTimer LampTimer;
IntervalTimer RegResetTimer;

elapsedMillis SinceLEDChange;       //Counter for time until next LED to flash in lamp test
elapsedMillis SincePress[58];       //Counters for time since switches' last electrical contacts
elapsedMillis SinceLEDFlash[2];     //Counters for time since protected switches' LEDs were flipped

void setup(){
  pinMode(MEN, INPUT);
  pinMode(SWIN, INPUT);
  pinMode(SR_DOUT, OUTPUT);
  pinMode(SR_CLK, OUTPUT);
  pinMode(SR_RST_L, OUTPUT);
  pinMode(SR_RCLK, OUTPUT);
  
  pinMode(DB0, OUTPUT);
  pinMode(DB1, OUTPUT);
  
  Serial.begin(9600);
  
  LCD.begin(16, 2);                 //Initialize LCD with cols, rows
  Joystick.useManualSend(true);     //Initialize joysticks with manual polling
  
  ResetReg();                       //Reset shift register contents
  
  attachInterrupt(digitalPinToInterrupt(0), LockBoard, FALLING);  //Change program state to LOCKED when user locks board
}

void loop(){
  switch(ProgState){
    case(LOCKED):      
      EllipsesTimer.end();          //Reset timer in case already running
      EllipsesLoc = 6;
      LCD.setCursor(0, 1);
      LCD.print("Willis Solutions");
      LCD.setCursor(0, 0);
      LCD.print("Locked      ");
      LCD.setCursor(EllipsesLoc, 0);
      EllipsesTimer.begin(EllipsesDisp, ELLIPSES_LOCK_PER);   //Create ellipses timer task
      
      while(digitalRead(MEN) == 1){}    //Blocking code: wait until board is locked (Board cannot start unlocked)
      while(digitalRead(MEN) == 0){}    //Blocking code: wait until board is unlocked
      EllipsisCnt = 0;
      ProgState = INIT;
      break;
      
    case(INIT):
      EllipsisCnt = 0;
      EllipsesLoc = 12;
      LCD.setCursor(0, 0);
      LCD.print("Initializing");
      EllipsesTimer.begin(EllipsesDisp, ELLIPSES_INIT_PER);   //Call EllipsesDisp() quicker in initialization state
      
      for(int i = 0; i <= 45; i++){                 //Turn off all LEDs before lamp test
        LEDOn[i] = 0;
      }
      SinceLEDChange = 0;
      LampTimer.begin(LampTest, REG_CNTL_PER);      //Create lamp test task
      LampTimer.priority(REG_CNTL_PRIO);
      LampTestRunning = 1;
      while(LampTestRunning){}                      //Blocking code: wait until lamp test finishes
      LampTimer.end();                              //Stop lamp test
      for(int i = 0; i <= 45; i++){                 //Turn off all LEDs before lamp test
        LEDOn[i] = 0;
      }
      
      EllipsesTimer.end();                          //Stop displaying ellipses
      EllipsisCnt = 0;
      LCD.clear();
      
      LCDTimer.begin(LCDDisp, LCD_CNTL_PER);        //Create LCD controller task
      LCDTimer.priority(LCD_CNTL_PRIO);
      JoyTimer.begin(JoyCntl, JOY_CNTL_PER);        //Create joystick controller task
      JoyTimer.priority(JOY_CNTL_PRIO);
      RegTimer.begin(RegCntl, REG_CNTL_PER);        //Create switch/LED controller task
      RegTimer.priority(REG_CNTL_PRIO);
      
      if(ProgState != LOCKED){                      //If key was turned in initialization state, go back to locked state
        ProgState = RUNNING;
      } else{}
      break;
      
    case(RUNNING):                                  //All tasks are interrupts
      break;
  }
}

/*****************************************************************
* RegCntl():
* ISR - Controls shift registers by shifting a digital high signal 
* through 104 serially connected outputs.
*****************************************************************/
void RegCntl(){  
  digitalWrite(DB0, 1);  
  int led_num = 45;           //Number of LEDs to cycle through 
  int sw_cnt = 0;             //Current switch being analyzed
  
  //Control switches
  digitalWrite(SR_DOUT, 1);   //High signal to shift through registers
  if(IS_SWITCH(0)){           //Output is connected to switch - latch in high signal
    BitShift();
    sw_cnt++;
  } else{}
  digitalWrite(SR_DOUT, 0);   //Only desire one high signal in registers to distinguish switches
  
  for(int i = 1; i < 103; i++){                         //Repeat for remaining register outputs
    SwitchOn[sw_cnt-1] = (digitalRead(SWIN)) ? 1 : 0;   //Update status of switches
    if(IS_SWITCH(i)){
      BitShift();
      sw_cnt++;
    } else{
      PulseCLK();
    }
  }
  
  SendGameData();   //Send switch data to game
  SendLEDData();    //Light LEDs corresponding to game and switch data
  
  //Control LEDs
  for(int i = 103; i >= 0; i--){              
    if(IS_LED(i) && LEDOn[led_num]){              //Outut is connected to LED to be turned on - write high signal to output
      digitalWrite(SR_DOUT, 1);
      PulseCLK();
      led_num--;
    } else if(IS_LED(i) && !LEDOn[led_num]){      //LED to be turned off - write low signal
      digitalWrite(SR_DOUT, 0);
      PulseCLK();
      led_num--;
    } else{
      digitalWrite(SR_DOUT, 0);                   //Output is connected to switch - write low signal
      PulseCLK();      
    }
  }
  PulseRCLK();                                    //Latch in signals sent to outputs
  RegResetTimer.begin(ResetReg, RESET_REG_PER);   //Create register reset task to dim LEDs
  RegResetTimer.priority(REG_RESET_PRIO);
  digitalWrite(DB0, 0);  
}

/*****************************************************************
* BitShift():
* Pulses shift register clock and latch to shift data through 
* register outputs.
*****************************************************************/
void BitShift(){
  PulseCLK();
  PulseRCLK();
}

/*****************************************************************
* PulseCLK():
* Pulses shift register clock.
*****************************************************************/
void PulseCLK(){
  digitalWrite(SR_CLK, 1);
  digitalWrite(SR_CLK, 0);
}

/*****************************************************************
* PulseRCLK():
* Pulses shift register latch.
*****************************************************************/
void PulseRCLK(){
  digitalWrite(SR_RCLK, 1);
  digitalWrite(SR_RCLK, 0);
}

/*****************************************************************
* ResetReg():
* Pulses shift register reset and latch to clear its contents.
*****************************************************************/
void ResetReg(){
  digitalWrite(SR_RST_L, 0);
  digitalWrite(SR_RST_L, 1);
  PulseRCLK();
  RegResetTimer.end();
}

/*****************************************************************
* LampTest():
* ISR - Flashes LEDs in sequence as an initialization display.
*****************************************************************/
void LampTest(){
  int led_num = 45;           //Number of LEDs to cycle through 
  
  //Set LED statuses
  if((SinceLEDChange >= LAMPSEP) && (LEDOnCnt <= 45)){        //Wait for next LED to be turned on
    SinceLEDChange = 0;
    LEDOn[LEDOnCnt] = 1;
    LEDOnCnt++;
  } else if((LEDOnCnt > 45) && (SinceLEDChange >= LAMPHOLD)){ //When all LEDs have been turned on, hold LEDs on and end lamp test
    LEDOnCnt = 0;
    LampTestRunning = 0;
  } else{}
  
  //Control LEDs
  RegResetTimer.end();
  digitalWrite(SR_DOUT, 1);  
  for(int i = 103; i >= 0; i--){            
    if(IS_LED(i) && LEDOn[led_num]){
      digitalWrite(SR_DOUT, 1);
      PulseCLK();
      led_num--;
    } else if(IS_LED(i) && !LEDOn[led_num]){
      digitalWrite(SR_DOUT, 0);
      PulseCLK();
      led_num--;
    } else{
      digitalWrite(SR_DOUT, 0);
      PulseCLK();      
    }
  }
  PulseRCLK();  
  RegResetTimer.begin(ResetReg, RESET_REG_PER);
  RegResetTimer.priority(REG_RESET_PRIO);
}

/*****************************************************************
* JoyCntl():
* ISR - Controls joysticks by periodically reading their values
* and sending them to the game.
*****************************************************************/
void JoyCntl(){
  Joystick.X(analogRead(0));
  Joystick.Y(analogRead(1));
  Joystick.Z(analogRead(2));
  Joystick.Zrotate(analogRead(3));
  Joystick.sliderLeft(analogRead(4));
  
  Joystick.send_now();    //Send joystick values to game
}

/*****************************************************************
* LCDDisp():
* ISR - Displays messages to the LCD.
*****************************************************************/
void LCDDisp(){  
  digitalWrite(DB1, 1);
  LCD.setCursor(0,0);
  LCD.print("APO: ");
  LCD.print(cnt+500);
  LCD.print("k   ");
  
  LCD.setCursor(0,1);
  LCD.print("PER: ");
  LCD.print(cnt);
  LCD.print("k   ");

  if(cnt <= 0) cnt = 1001;
  cnt--;
  digitalWrite(DB1, 0);
}

/*****************************************************************
* EllipsesDisp():
* ISR - Displays Ellipses to the LCD as a form of loading text. 
* Displays one dot every function call up to a maximum of three, 
* then erases and repeats.
*****************************************************************/
void EllipsesDisp(){
  if(EllipsisCnt >= 3){
    LCD.setCursor(EllipsesLoc, 0);
    
    LCD.print("   ");
    LCD.setCursor(EllipsesLoc, 0);
    EllipsisCnt = 0;
  } else{    
    LCD.print("."); 
    EllipsisCnt++;
  }  
}

/*****************************************************************
* LockBoard():
* ISR - when user locks board, set program state to LOCKED and
* stop all tasks.
*****************************************************************/
void LockBoard(){
  //Stop all tasks
  LCDTimer.end();
  JoyTimer.end();
  RegTimer.end();
  EllipsesTimer.end();
  LampTimer.end();
  ResetReg();     

  //Reset all volatile global variables
  EllipsesLoc = 6;
  EllipsisCnt = 0;
  LampTestRunning = 0;
  LEDOnCnt = 0;
  StageFlashState = 0;
  AbortFlashState = 0;
  
  for(int i = 0; i <= 45; i++){   //Turn off all LEDs
    LEDOn[i] = 0;
  }
  if(ProgState == RUNNING){       //Clear LCD
    LCD.clear();
  } else{}
  ProgState = LOCKED;
}

/*****************************************************************
* SendGameData():
* Sends keyboard characters to game corresponding to switch data.
*****************************************************************/
void SendGameData() {
  int dbounce = 0;
  
  for(int i = 0; i < 58; i++){
    switch(SwitchType[i]){        //Each type of switch has debounce period
      case(1):
        dbounce = DBOUNCE_SW;
        break;
      case(2):
        dbounce = DBOUNCE_SQ;
        break;
      case(3):
        dbounce = DBOUNCE_SM;
        break;
      case(4):
        dbounce = DBOUNCE_LG;
        break;
      default:
        break;
    }
    if(SincePress[i] >= dbounce){                                       //Wait for specific switch's debounce period to complete
      if((IS_ACTIVE_HIGH(i) && SwitchOn[i] && !SwitchOnPrev[i])         //Differentiate between low and active high switches
        || (IS_ACTIVE_LOW(i) && !SwitchOn[i] && !SwitchOnPrev[i])){     //and only send characters once per switch activation
        Keyboard.print(SwitchChar[i]);
//        Joystick.button(1, 1);    //For Windows joystick calibration
        SwitchOnPrev[i] = 1; 
        switch(i){            
          case(4):
            Orient = MAN;         //Set orientation of ship
            break; 
          case(5):
            Orient = STBL;
            break;  
          case(6):
            Orient = PROG;
            break;  
          case(7):
            Orient = RETG;
            break;  
          case(8):
            Orient = NML;
            break;  
          case(9):
            Orient = ANML;
            break;  
          case(10):
            Orient = RDI;
            break;  
          case(11):
            Orient = RDO;
            break; 
          case(12):
            Orient = TRGP;
            break;  
          case(13):
            Orient = TRGR;
            break;
          case(36):
            StageShield = OPEN;   //Set states of shielded switches
            LEDOn[21] = 1;        //Begin LEDs' flashing sequences on, not off
            SinceLEDFlash[0] = 0;
            break;
          case(47):
            AbortShield = OPEN;
            LEDOn[22] = 1;
            SinceLEDFlash[1] = 0;
            break;
          default:
            break; 
        }
      } else if((IS_ACTIVE_HIGH(i) && !SwitchOn[i] && SwitchOnPrev[i])    //User deactivated switch/lifted key
               || (IS_ACTIVE_LOW(i) && SwitchOn[i] && SwitchOnPrev[i])){
//        Joystick.button(1, 0);      //For Windows joystick calibration
        SwitchOnPrev[i] = 0; 
        if(IS_TOGGLE(i)){           //For switches, send character on toggle
          Keyboard.print(SwitchChar[i]);
          if(i == 36){
            StageShield = CLOSED;   //Set states of shielded switches
            LEDOn[21] = 0;          //Immediately turn off LEDs
          } else if(i == 47){
            AbortShield = CLOSED;   
            LEDOn[22] = 0;         
          } else{}
        } else{}
      } else{}
        SincePress[i] = 0;          //Reset switch debounce timer
    } else{}
  }
}

/*****************************************************************
* SendLEDData():
* Activates LEDs corresponding to switch and game data.
*****************************************************************/
void SendLEDData(){  
  //Turn on LEDs when their respective switches are on
  for(int i = 0; i < 4; i++) {
    ToggleLED(i, i);
  }  
  for(int i = 14; i < 20; i++) {
    ToggleLED(i, i);
  }
  ToggleLED(34, 20);  
  for (int i = 49; i < 58; i++) {
    ToggleLED(i, i - 26);
  }
  
  //Control ship orientation LEDs - only one LED on at a time 
  for(int i = 4; i < 14; i++){
    LEDOn[i] = 0;
  }
  switch(Orient){
    case(MAN):
      LEDOn[4] = 1;
      break;
    case(STBL):
      LEDOn[5] = 1;
      break;
    case(PROG):
      LEDOn[6] = 1;
      break;
    case(RETG):
      LEDOn[7] = 1;
      break;
    case(NML):
      LEDOn[8] = 1;
      break;
    case(ANML):
      LEDOn[9] = 1;
      break;
    case(RDI):
      LEDOn[10] = 1;
      break;
    case(RDO):
      LEDOn[11] = 1;
      break;
    case(TRGP):
      LEDOn[12] = 1;
      break;
    case(TRGR):
      LEDOn[13] = 1;
      break;
    default:
      break;
  }  

  //Control shielded switches' LEDs - timers control duty cycles
  //Stage
  if((SinceLEDFlash[0] >= LEDOFF_PER) && (StageShield == OPEN) && (StageFlashState == 1)){    
    LEDOn[21] = 1;
    StageFlashState = 0;
    SinceLEDFlash[0] = 0;
  } else{}
  if((SinceLEDFlash[0] >= LEDON_PER) && (StageShield == OPEN) && (StageFlashState == 0)){
    LEDOn[21] = 0;
    StageFlashState = 1;
    SinceLEDFlash[0] = 0;
  } else{}  
  //Abort
  if((SinceLEDFlash[1] >= LEDOFF_PER) && (AbortShield == OPEN) && (AbortFlashState == 1)){
    LEDOn[22] = 1;
    AbortFlashState = 0;
    SinceLEDFlash[1] = 0;
  } else{} 
  if((SinceLEDFlash[1] >= LEDON_PER) && (AbortShield == OPEN) && (AbortFlashState == 0)){
    LEDOn[22] = 0;
    AbortFlashState = 1;
    SinceLEDFlash[1] = 0;
  } else{}    

}
  
/*****************************************************************
* ToggleLED():
* Turn on LED if corresponding switch is on.
*****************************************************************/
void ToggleLED(int x, int y) {
  if((!IS_ACTIVE_LOW(x) && SwitchOn[x])       //Active low and active high switches
    || (IS_ACTIVE_LOW(x) && !SwitchOn[x])){
    LEDOn[y] = 1;
  } else{
    LEDOn[y] = 0;
  }
}





