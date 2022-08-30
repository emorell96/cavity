#include <SPI.h>

const int reset1 = A0;
const int clr1   = A1;
const int ldac1  = A2;
const int sync1  = A3;

const int reset2 = A4;
const int clr2   = A5;
const int ldac2  = A6;
const int sync2  = A7;

int reset = reset1;
int clr   = clr1;
int ldac  = ldac1;
int sync  = sync1;

const bool plotPIDValues = true;
 
#define AD5791_NOP 0 // No operation (NOP).
#define AD5791_REG_DAC 1 // DAC register.
#define AD5791_REG_CTRL 2 // Control register.
#define AD5791_REG_CLR_CODE 3 // Clearcode register.
#define AD5791_CMD_WR_SOFT_CTRL 4 // Software control register(Write only).
 
typedef enum {
ID_AD5760,ID_AD5780,ID_AD5781,ID_AD5790,ID_AD5791,} AD5791_type;
 
struct ad5791_chip_info {
unsigned int resolution;
};
static const struct ad5791_chip_info ad5791_chip_info[] = {
[ID_AD5760] = {.resolution = 16,},
[ID_AD5780] = {.resolution = 18,},
[ID_AD5781] = {.resolution = 18,},
[ID_AD5790] = {.resolution = 20,},
[ID_AD5791] = {.resolution = 20,}
};
 
AD5791_type act_device;
 
/* Maximum resolution */
#define MAX_RESOLUTION 20
/* Register Map */
#define AD5791_NOP 0 // No operation (NOP).
#define AD5791_REG_DAC 1 // DAC register.
#define AD5791_REG_CTRL 2 // Control register.
#define AD5791_REG_CLR_CODE 3 // Clearcode register.
#define AD5791_CMD_WR_SOFT_CTRL 4 // Software control register(Write only).
/* Input Shift Register bit definition. */
#define AD5791_READ (1ul << 23)
#define AD5791_WRITE (0ul << 23)
#define AD5791_ADDR_REG(x) (((unsigned long)(x) & 0x7) << 20)
/* Control Register bit Definition */
#define AD5791_CTRL_LINCOMP(x) (((x) & 0xF) << 6) // Linearity error compensation.
#define AD5791_CTRL_SDODIS (1 << 5) // SDO pin enable/disable control.
#define AD5791_CTRL_BIN2SC (1 << 4) // DAC register coding selection.
#define AD5791_CTRL_DACTRI (1 << 3) // DAC tristate control.
#define AD5791_CTRL_OPGND (1 << 2) // Output ground clamp control.
#define AD5791_CTRL_RBUF (1 << 1) // Output amplifier configuration control.
/* Software Control Register bit definition */
#define AD5791_SOFT_CTRL_RESET (1 << 2) // RESET function.
#define AD5791_SOFT_CTRL_CLR (1 << 1) // CLR function.
#define AD5791_SOFT_CTRL_LDAC (1 << 0) // LDAC function.
/* DAC OUTPUT STATES */
#define AD5791_OUT_NORMAL 0x0
#define AD5791_OUT_CLAMPED_6K 0x1
#define AD5791_OUT_TRISTATE 0x2
 
long AD5791_SetRegisterValue(unsigned char registerAddress, unsigned long registerValue) {
  unsigned char writeCommand[3] = {0, 0, 0};
  unsigned long spiWord = 0;
  char status = 0;
  spiWord = AD5791_WRITE | AD5791_ADDR_REG(registerAddress) | (registerValue & 0xFFFFF);
  writeCommand[0] = (spiWord >> 16) & 0x0000FF;
  writeCommand[1] = (spiWord >> 8 ) & 0x0000FF;
  writeCommand[2] = (spiWord >> 0 ) & 0x0000FF;
   
  digitalWrite(sync,LOW);
  status = SPI.transfer(writeCommand[0]);
  status = SPI.transfer(writeCommand[1]);
  status = SPI.transfer(writeCommand[2]);
  digitalWrite(sync,HIGH);
 
  return 0;
}
 
long AD5791_GetRegisterValue(unsigned char registerAddress) {
  unsigned char registerWord[3] = {0, 0, 0};
  unsigned long dataRead = 0x0;
  char status = 0;
  registerWord[0] = (AD5791_READ | AD5791_ADDR_REG(registerAddress)) >> 16;
 
  digitalWrite(sync,LOW);
 
  status = SPI.transfer(registerWord[0]);
  status = SPI.transfer(registerWord[1]);
  status = SPI.transfer(registerWord[2]);
  digitalWrite(sync,HIGH);
 
 
  registerWord[0] = 0x00;
  registerWord[1] = 0x00;
  registerWord[2] = 0x00;
    digitalWrite(sync,LOW);
  registerWord[0] = SPI.transfer(0x00);
  registerWord[1] = SPI.transfer(0x00);
  registerWord[2] = SPI.transfer(0x00);
    digitalWrite(sync,HIGH);
  dataRead = ((long)registerWord[0] << 16) |
             ((long)registerWord[1] << 8) |
             ((long)registerWord[2] << 0);
  return dataRead;
}

//-----------------------------------------------------------------------
#include <string.h>
#include <math.h>
#include <IntervalTimer.h>
IntervalTimer myTimer;
#define Tsample 25 //sample time for timer in microseconds
#define pi 3.14159

//Low pass butterworth filter order=2 alpha1=0.0003 ==> 15Hz
class  FilterBuLp2
{
  public:
    FilterBuLp2()
    {
      v[0]=0.0;
      v[1]=0.0;
      v[2]=0.0;
    }
  private:
    float v[3];
  public:
    float main(float x) //class II 
    {
      v[0] = v[1];
      v[1] = v[2];
      v[2] = (8.870817736483793681e-7 * x)
         + (-0.99733782013962946067 * v[0])
         + (1.99733427181253486715 * v[1]);
      return 
         (v[0] + v[2]) + 2 * v[1];
    }
};

//Low pass filter, order=2 alpha1=0.0003 ==> 15Hz

const float lpf2_freq1 = 100;
const float lpf2_freq2 = 500;

class  LPF2
{
  public:
    LPF2(float corner_freq)
    {
      v[0]=0.0;
      v[1]=0.0;      
      LPF2_al = 0.03 / 213 * corner_freq;
    }
  private:
    float LPF2_al;
  private:
    float v[2];
  public:
    float main(float x) //class II 
    {
      if (v[0] == 0.0 && v[1] == 0.0)   //initialize
      {
        v[0] = x;
        v[1] = x;        
      }
      else
      {
        v[0] = (1 - LPF2_al) * v[0] + LPF2_al * x;
        v[1] = (1 - LPF2_al) * v[1] + LPF2_al * v[0];
      }
      
      return v[1];
    }
};

class PID{
  private:
    float errorSum = 0;
    float setPoint = 0;
    float oldError = 0;

    float P = 0.1;
    float I = 0.1;
    float D = 0.1;

  
  public:
    void setP(float p){
      this->P = p;
    }
    void setI(float i){
      this->I = i;
    }
    void setD(float d){
      this->D = d;
    }

    float getP(){
      return this->P;
    }
    float getI(){
      return this->I;
    }
    float getD(){
      return this->D;
    }
    

    PID(float P, float I, float D, float setPoint){
      this->P = P;
      this->I = I,
      this->D = D;
      this->setPoint = setPoint;
    }



    // Calculates the correction of the PID from the given error.    
    float CalculateError(float error){
        this->errorSum = this->errorSum + (error - this->setPoint)/100000; //Scaling the integral
        if (this->I*this->errorSum > 400000) //Checking if integral feedback is not too large
          this->errorSum = 400000/this->I;
        if (this->I*this->errorSum < -400000)
          this->errorSum = -400000;
        this->oldError = error;
        return this->P*(error - this->setPoint) + this->I*this->errorSum + this->D*(error - this->oldError); //all you have to do is replace this with NN output
    } 
};




int r = 0; //index used for sine
int Refl =0;
float error = 0;
float errorsum = 0;
float olderror = 0;
float C = 0; //DC correction in 12-bit units
int phi = 20;

const int samplefreq = 1000000/Tsample; //sampling freq for timer in Hertz

volatile boolean flag = false;
void flagpost(){
  flag = true;
}

//For sinewaves
const int freq = 3000; //freq of sinewave in Hertz 
float ampl =0.005;//Pk-Pk amplitude of sinewave in Volt 
//Scaling is a bit off. Accurate for 0.5-2.0V, 0.025 will give 40mV
const int sinetablesize = (samplefreq -(samplefreq%freq))/freq;
static int sinetable[36][sinetablesize]={};

float AC_ampl_bits = 0;
float dV = 0;
float Volt=0;
float volt_start;
bool flag_dV_reduced = false;
int monitor_shift = 0;

const int max_bits=524288-1;

unsigned long oldCtrl_c = 2097152;
FilterBuLp2 LPF;
LPF2 lpf2_1(lpf2_freq1);
LPF2 lpf2_2(lpf2_freq2);

double volt_limit_down = 100000;
double volt_limit_up = 300000;
float DAC2_offset = 10;//find the proper voltage by delta source
//PDH LPF 1.28kHz

//works best with 1 kHz ZI LPF
//float P = 1.0;   //0.5
//float I = 0.1;  //0.4
//float D = 0.0; //0.15

//works best with 9 Hz ZI LPF, up to 3 Hz
//float P = 0.5;   //0.5
//float I = 0.1;  //0.4

//140 V on membrane piezo params: minimal I=60n


const float P = 0.1;   //0.5
const float I = 2;  //320 -> 250 Hz in ZI LPF min; 32 -> 15 Hz in ZI LPF min
const float D = 0.05; //0.15



void setup() 
{
  pinMode(A8, INPUT);
  Serial.begin(38400);
  SPI.begin();
  pinMode(reset1, OUTPUT);
  pinMode(clr1  , OUTPUT);
  pinMode(ldac1 , OUTPUT);
  pinMode(sync1 , OUTPUT);
  pinMode(reset2, OUTPUT);
  pinMode(clr2  , OUTPUT);
  pinMode(ldac2 , OUTPUT);
  pinMode(sync2 , OUTPUT);

  digitalWrite(ldac1,LOW);
  digitalWrite(reset1,HIGH);
  digitalWrite(clr1,HIGH);
  digitalWrite(sync1,HIGH);
  digitalWrite(ldac2,LOW);
  digitalWrite(reset2,HIGH);
  digitalWrite(clr2,HIGH);
  digitalWrite(sync2,HIGH);
  
  SPI.beginTransaction(SPISettings(3800000, MSBFIRST, SPI_MODE1));
     
  long status = AD5791_GetRegisterValue(AD5791_REG_CTRL);  
  status = AD5791_GetRegisterValue(AD5791_REG_CTRL);
 
  unsigned long oldCtrl = status;
  oldCtrl = oldCtrl & ~(AD5791_CTRL_LINCOMP(-1) | AD5791_CTRL_SDODIS | AD5791_CTRL_BIN2SC | AD5791_CTRL_RBUF | AD5791_CTRL_OPGND);
 
  status = AD5791_SetRegisterValue(AD5791_REG_CTRL, oldCtrl);  
  status = AD5791_GetRegisterValue(AD5791_REG_CTRL);
  long d=1;
  digitalWrite(ldac,LOW);

  ldac=ldac1;
  reset=reset1;
  clr=clr1;
  sync=sync1; 
  status = AD5791_SetRegisterValue(AD5791_REG_CTRL, oldCtrl_c);      
  AD5791_SetRegisterValue(AD5791_REG_DAC, 1);

  ldac=ldac2;
  reset=reset2;
  clr=clr2;
  sync=sync2; 
  status = AD5791_SetRegisterValue(AD5791_REG_CTRL, oldCtrl_c);      
  AD5791_SetRegisterValue(AD5791_REG_DAC, 1);

  //---------------------------------------
  analogReadResolution(12); // 1 point corresponds to 3.3V/2^(12) 
  analogReadAveraging(3);
  myTimer.begin(flagpost, Tsample); //reset flag to true every Tsample
  delay(1000);
  
  AC_ampl_bits = max_bits*ampl/5;
  volt_start = AC_ampl_bits;
  Volt = 0;
  dV = 1.0/500.0;       



  for (int h=0;h<36;h++) //sinetable is generated
    {
      for (int j =0; j<sinetablesize; j++)
      {
        sinetable[h][j] = max_bits*(ampl*sin(j*2*pi/sinetablesize+2*h*pi/36))/10;
      }
    }

  


   //switch to DAC2 and initialize
  ldac=ldac2;
  reset=reset2;
  clr=clr2;
  sync=sync2; 
  status = AD5791_SetRegisterValue(AD5791_REG_CTRL, oldCtrl_c);  
  AD5791_SetRegisterValue(AD5791_REG_DAC, 1);
  AD5791_SetRegisterValue(AD5791_REG_DAC, DAC2_offset);//DC offset for DAC2   
  
  

  // initialize DAC1
  volt_start=250000;//scan from 5V
  ldac=ldac1;
  reset=reset1;
  clr=clr1;
  sync=sync1; 
  status = AD5791_SetRegisterValue(AD5791_REG_CTRL, oldCtrl_c);      
  AD5791_SetRegisterValue(AD5791_REG_DAC, 1);

 
  
  Serial.println("initialized");
  delay(10000);
}

int engage = 0; //switch involved in tranfer between scan and feedback

const float setpoint = 0;

int print_index = 0;
bool maximum_reached = false;
float flag_monitor = 0;
bool scanning_done = true;   //if false, pid on after scanning
bool freeze = true;
int time_index = 0;
int state_flag = 0;

//freeze parameters
int pid_on_loops = 100000;    //2 sec 
int freeze_loops = 5000;    //1 sec
int freeze_multiplier = 20;  //30;
int cycle_index_freeze_extension = 30;
float I_drift = 0.15;    //pid I component for the drift
float C_end_sum = 0;
int index_loop = 0;   //index_loop * Tsample = time since the program started
int index_loop_cycle_start = 0;
float drift = 0;
int cycle_index = 0;
bool freeze_duration_increase_happened = false;

bool DAC2_finished = false;
bool DAC1_finished = false;
float volt_out_DAC2 = 1;
float dvolt_DAC2 = 0.5;
long status;
double last_resonance_volt = 0;
float dvolt_DAC1 = 0.5;
double volt_out_DAC1 = -100;
int number_scans_DAC1 = 0;
float temp_volt = 0;
float temp_volt_d = 0.0005;
int temp_flag = 0;
int n=0;
float voltoutacc=0;
float t1=0;
int volt_out = 0;
byte incomingByte = 122;

PID DefaultPid(P, I, D, setpoint);

void loop() 
{ 
  if (Serial.available())
  {
    incomingByte = Serial.read();  // will not be -1
    //Serial.println(incomingByte);   //  l=108; s=115;  z=122
  }
  if (flag)
  {
    flag = false;
    n++; 
    if (volt_out_DAC1 == -100)
      volt_out_DAC1 = volt_limit_down + 1;      
    Refl = analogRead(A8)*50;
    error = -LPF.main(sinetable[phi][r]*Refl)/25.0 * 20;   
    //error = 2 * Refl;//14000-27000  
    //error = 2 * lpf2_1.main(Refl);//14000-27000  

    r = (r+1) % sinetablesize;
    
    AD5791_SetRegisterValue(AD5791_REG_DAC, volt_out_DAC1);
  
      
///////////////////////////////////////////////////////////////
    if (incomingByte == 122)  //"z" for zero
    {
      //Serial.println("z");
        if (volt_out_DAC1 > (volt_limit_up + volt_limit_down) / 2.0 + abs(dvolt_DAC1))
        {         
            dvolt_DAC1 = -abs(dvolt_DAC1);
            volt_out_DAC1 = volt_out_DAC1 + dvolt_DAC1;
        }
        
        if (volt_out_DAC1 < (volt_limit_up + volt_limit_down) / 2.0 - abs(dvolt_DAC1))
        {
            dvolt_DAC1 = abs(dvolt_DAC1);
            volt_out_DAC1 = volt_out_DAC1 + dvolt_DAC1;
        }   
        errorsum = 0;    
        DAC1_finished = false;
    }
///////////////////////////////////////////////////////////////  
    if (incomingByte == 115)  //"s" for scan
    {
        if(volt_out_DAC1 > volt_limit_up || volt_out_DAC1 < volt_limit_down)        
           dvolt_DAC1 = -dvolt_DAC1;             
          
        volt_out_DAC1 = volt_out_DAC1 + dvolt_DAC1; //+ sinetable[0][r];
        errorsum = 0;
        DAC1_finished = false;
    }
    ///////////////////////////////////////////////////////////////  
    if (incomingByte == 100)  //"d" for down, decreases DAC2 voltage
    {
        float DAC2_step=20000;
        if(DAC2_offset > DAC2_step)
        {
            DAC2_offset = DAC2_offset - DAC2_step;
            ldac=ldac2;
            reset=reset2;
            clr=clr2;
            sync=sync2; 
            status = AD5791_SetRegisterValue(AD5791_REG_CTRL, oldCtrl_c);      
            AD5791_SetRegisterValue(AD5791_REG_DAC, DAC2_offset);

              // initialize DAC1            
            ldac=ldac1;
            reset=reset1;
            clr=clr1;
            sync=sync1; 
            status = AD5791_SetRegisterValue(AD5791_REG_CTRL, oldCtrl_c);     
        }                 
    }
    ///////////////////////////////////////////////////////////////  
    if (incomingByte == 117)  //"u" for up, increases DAC2 voltage
    {
        float DAC2_step=20000;
        if(DAC2_offset < 500000 - DAC2_step)
        {
            DAC2_offset = DAC2_offset + DAC2_step;
            ldac=ldac2;
            reset=reset2;
            clr=clr2;
            sync=sync2; 
            status = AD5791_SetRegisterValue(AD5791_REG_CTRL, oldCtrl_c);      
            AD5791_SetRegisterValue(AD5791_REG_DAC, DAC2_offset);

              // initialize DAC1            
            ldac=ldac1;
            reset=reset1;
            clr=clr1;
            sync=sync1; 
            status = AD5791_SetRegisterValue(AD5791_REG_CTRL, oldCtrl_c);     
        }                 
    }
///////////////////////////////////////////////////////////////   
    if (incomingByte == 108 && DAC1_finished == false ) //lock
    {
        if(volt_out_DAC1 > volt_limit_up || volt_out_DAC1 < volt_limit_down)        
           dvolt_DAC1 = -dvolt_DAC1; 
        volt_out_DAC1 = volt_out_DAC1 + dvolt_DAC1 + sinetable[0][r];
  
     //////////////////once the resonace found, scan a small range/////////////////
          if (Refl < 5000)  //last_resonance_volt initialized 0
          {  
              Serial.println("resonance found");
              DAC1_finished = true;
              //dvolt_DAC1 = dvolt_DAC1 / 8.0;
              last_resonance_volt = volt_out_DAC1;                                                       
          }               
    }
///////////////////////////////////////////////////////////////
    if(incomingByte == 80 && DAC1_finished) // enter "P" in the serial port followed by a float value.
    {
        // try to read the P value from the serial
        float p = Serial.parseFloat();
        DefaultPid.setP(p);
    }

    if(incomingByte == 73 && DAC1_finished) // enter "I" in the serial port followed by a float value.
    {
        // try to read the P value from the serial
        float i = Serial.parseFloat();
        DefaultPid.setI(i);
    }

    if(incomingByte == 68 && DAC1_finished) // enter "D" in the serial port followed by a float value.
    {
        // try to read the P value from the serial
        float d = Serial.parseFloat();
        DefaultPid.setD(d);
    }

    if (incomingByte == 108 && DAC1_finished) // lock, the byte values come from the ASCII table: https://www.cs.cmu.edu/~pattis/15-1XX/common/handouts/ascii.html
    {      
      engage = 1;
      flag = false;
      scanning_done = true;
      
      
      
      //restrict locking range
      if (volt_out_DAC1 > volt_limit_up)
        volt_out_DAC1 = volt_limit_up;
      if (volt_out_DAC1 < volt_limit_down)
        volt_out_DAC1 = volt_limit_down;
      AD5791_SetRegisterValue(AD5791_REG_DAC, volt_out_DAC1);
  
             
  
      if(engage == 1)   //feedback on
          {
            time_index = time_index + 1;          
  //              if (state_flag == 0)    //switch pid off  state_flag==0
  //              {
  //                  if (freeze && time_index > pid_on_loops)
  //                  {
  //                    flag_monitor = 1000;
  //                    state_flag = 1; 
  //                    time_index = 0;          
  //                  }
  //              }
            if(state_flag == 0)// PID
            {       
              C = DefaultPid.CalculateError(error);
            }        
          }

        volt_out_DAC1 = last_resonance_volt + C + sinetable[0][r];
    }
    
    print_index++;              
    if (print_index == 1000)
    {      
      Serial.print(volt_out_DAC1/100);   
      Serial.print(",");
      Serial.print(Refl);   
      Serial.print(",");   
      Serial.print(error); 

      if(plotPIDValues){
        Serial.print(",");

        Serial.print(DefaultPid.getP());
        Serial.print(",");

        Serial.print(DefaultPid.getI());
        Serial.print(",");

        Serial.print(DefaultPid.getD());
        Serial.print(",");
      } 
      Serial.println(); 
      print_index = 0;
    }
  }
}
