#include <PdhSignalTracker.h>
#include <Point.h>

#include <SPI.h>
#include <string.h>
#include <math.h>
#include <IntervalTimer.h>

const int reset1 = A0;
const int clr1 = A1;
const int ldac1 = A2;
const int sync1 = A3;

const int reset2 = A4;
const int clr2 = A5;
const int ldac2 = A6;
const int sync2 = A7;

const bool plotPIDValues = true;

#define AD5791_NOP 0 // No operation (NOP).
#define AD5791_REG_DAC 1 // DAC register.
#define AD5791_REG_CTRL 2 // Control register.
#define AD5791_REG_CLR_CODE 3 // Clearcode register.
#define AD5791_CMD_WR_SOFT_CTRL 4 // Software control register(Write only).

typedef enum { ID_AD5760, ID_AD5780, ID_AD5781, ID_AD5790, ID_AD5791, } AD5791_type;

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

//-----------------------------------------------------------------------

#define Tsample 25 //sample time for timer in microseconds
#define pi 3.14159

const int samplefreq = 1000000 / Tsample; //sampling freq for timer in Hertz
const int freq = 3000; //freq of sinewave in Hertz 
const int sinetablesize = (samplefreq - (samplefreq % freq)) / freq;
static int sinetable[36][sinetablesize] = {};
const int max_bits = 524288 - 1;


//Low pass butterworth filter order=2 alpha1=0.0003 ==> 15Hz
class  FilterBuLp2
{
public:
    FilterBuLp2()
    {
        v[0] = 0.0;
        v[1] = 0.0;
        v[2] = 0.0;
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

const unsigned long oldCtrl_c = 2097152;

volatile bool flag = false;

void flagpost() {
    flag = true;
}

class  LPF2
{
public:
    LPF2(float corner_freq)
    {
        v[0] = 0.0;
        v[1] = 0.0;
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

class PID {
private:
    float errorSum = 0;
    float setPoint = 0;
    float oldError = 0;

    float P = 0.1;
    float I = 0.0;
    float D = 0.0;


public:
    void setP(float p) {
        this->P = p;
    }
    void setI(float i) {
        this->I = i;
    }
    void setD(float d) {
        this->D = d;
    }
    void setSetpoint(float setPoint) {
        this->setPoint = setPoint;
    }

    float getP() {
        return this->P;
    }
    float getI() {
        return this->I;
    }
    float getD() {
        return this->D;
    }

    void resetSum() {
        this->errorSum = 0;
    }

    PID(float P, float I, float D, float setPoint) {
        this->P = P;
        this->I = I;
        this->D = D;
        this->setPoint = setPoint;
    }


    // Calculates the correction of the PID from the given error.    
    float CalculateCorrection(float error) {
        this->errorSum = this->errorSum + (error - this->setPoint) / 100000; //Scaling the integral
        if (this->I * this->errorSum > 400000) //Checking if integral feedback is not too large
            this->errorSum = 400000 / this->I;
        if (this->I * this->errorSum < -400000)
            this->errorSum = -400000;
        this->oldError = error;
        return this->P * (error - this->setPoint) + this->I * this->errorSum + this->D * (error - this->oldError); //all you have to do is replace this with NN output
    }
};

class Dac {
private:
    unsigned long voltOut = 100000;
    float dvoltOut = 1;

    // voltage limits
    unsigned long voltLowerLimit = 100000;
    unsigned long voltUpperLimit = 300000;

    // pin definitions
    int reset;
    int clr;
    int ldac;
    int sync;

    //offsets
    long offsetStep = 20000;
    long offset = 10;

    bool isFinished = false;
    bool scanFinished = false;

public:
    Dac(int resetPin, int clearPin, int ldacPin, int syncPin) {
        this->reset = resetPin;
        this->clr = clearPin;
        this->ldac = ldacPin;
        this->sync = syncPin;
    }

    bool IsScanFinished() {
        return scanFinished;
    }

    void SetVoltLowerLimit(double voltLowerLimit) {
        this->voltLowerLimit = voltLowerLimit;
    }

    void SetVoltUpperLimit(double voltUpperLimit) {
        this->voltUpperLimit = voltUpperLimit;
    }

    /// <summary>
    /// set the voltage to output with the DAC. This will always respect the limits set by using the upper, and lower limits.
    /// this will not output that voltage automatically!
    /// </summary>
    void SetVoltOut(unsigned long voltOut) {
        if (voltOut > voltUpperLimit) {
            this->voltOut = voltUpperLimit - 1;
        }
        else if (voltOut < voltLowerLimit) {
            this->voltOut = voltLowerLimit + 1;
        }
        else {
            this->voltOut = voltOut;
        }
    }

    long GetVoltOut() {
        return voltOut;
    }

    void SendVoltageToRegister() {
        WriteDacValue(voltOut);
    }

    /// <summary>
    /// Resets the voltage to the lower limit + 1.
    /// </summary>
    void ResetVoltageToLowerLimit() {
        SetVoltOut(this->voltLowerLimit + 1);
    }

    unsigned long GetCurrentControlValue() {
        // AD5791_GetRegisterValue(AD5791_REG_CTRL); //why is this done twice?
        return AD5791_GetRegisterValue(AD5791_REG_CTRL);
    }

    // writes the value in the DAC registry not the control registry.
    int WriteDacValue(unsigned long dacValue) {
        return AD5791_SetRegisterValue(AD5791_REG_DAC, dacValue);
    }

    // saves the value in a register with the given dac.
    int WriteControlValue(unsigned long controlValue) {
        return AD5791_SetRegisterValue(AD5791_REG_CTRL, controlValue);
    }

    long GetControlValue() {
        return AD5791_GetRegisterValue(AD5791_REG_CTRL);
    }

    void SetUpPins() {
        pinMode(reset, OUTPUT);
        pinMode(clr, OUTPUT);
        pinMode(ldac, OUTPUT);
        pinMode(sync, OUTPUT);

        digitalWrite(ldac, LOW);
        digitalWrite(reset, HIGH);
        digitalWrite(clr, HIGH);
        digitalWrite(sync, HIGH);
    }

    /// <summary>
    /// This sets the LDAC to low, activating the DAC from the registry.
    /// </summary>
    /// <param name="registerAddress"></param>
    /// <param name="registerValue"></param>
    /// <returns></returns>
    void LoadDac() {
        digitalWrite(ldac, LOW);
    }

    void Zero() {
        if (voltOut > (voltUpperLimit + voltLowerLimit) / 2.0 + abs(dvoltOut))
        {
            dvoltOut = -abs(dvoltOut);
            SetVoltOut(voltOut + dvoltOut);
        }

        if (voltOut < (voltUpperLimit + voltLowerLimit) / 2.0 - abs(dvoltOut))
        {
            dvoltOut = abs(dvoltOut);
            SetVoltOut(voltOut + dvoltOut);
        }
        isFinished = false;
        scanFinished = false;
    }

    void ResetOffset() {
        offset = 10;
        WriteControlValue(oldCtrl_c);
        WriteDacValue(offset);
    }

    void StepDown() {
        if (offset > offsetStep)
        {
            offset -= offsetStep;
            WriteControlValue(oldCtrl_c);
            WriteDacValue(offset);
        }
    }

    void StepUp() {
        if (offset < 500000 - offsetStep)
        {
            offset += offsetStep;
            WriteControlValue(oldCtrl_c);
            WriteDacValue(offset);
            //AD5791_SetRegisterValue(AD5791_REG_DAC, DAC2_offset);

            //// initialize DAC1            
            //ldac = ldac1;
            //reset = reset1;
            //clr = clr1;
            //sync = sync1;
            //status = AD5791_SetRegisterValue(AD5791_REG_CTRL, oldCtrl_c);
        }
    }

    void Scan(int offsetScan = 0) {
        float newVoltage = voltOut + dvoltOut + offsetScan;
        if (newVoltage > voltUpperLimit || newVoltage < voltLowerLimit)
            dvoltOut = -dvoltOut;
        SetVoltOut(voltOut + dvoltOut + offsetScan);
        isFinished = false;
        if (dvoltOut < 0) {
            scanFinished = true;
        }
        else {
            scanFinished = false;
        }
    }


    bool IsFinished() {
        return isFinished;
    }

    void SetIsFinished(bool finished) {
        isFinished = finished;
    }


    int AD5791_SetRegisterValue(unsigned char registerAddress, unsigned long registerValue) {
        unsigned char writeCommand[3] = { 0, 0, 0 };
        unsigned long spiWord = 0;
        spiWord = AD5791_WRITE | AD5791_ADDR_REG(registerAddress) | (registerValue & 0xFFFFF);
        writeCommand[0] = (spiWord >> 16) & 0x0000FF;
        writeCommand[1] = (spiWord >> 8) & 0x0000FF;
        writeCommand[2] = (spiWord >> 0) & 0x0000FF;

        digitalWrite(sync, LOW);
        SPI.transfer(writeCommand[0]);
        SPI.transfer(writeCommand[1]);
        SPI.transfer(writeCommand[2]);
        digitalWrite(sync, HIGH);

        return 0;
    }
    long AD5791_GetRegisterValue(unsigned char registerAddress) {
        unsigned char registerWord[3] = { 0, 0, 0 };
        unsigned long dataRead = 0x0;

        registerWord[0] = (AD5791_READ | AD5791_ADDR_REG(registerAddress)) >> 16;

        digitalWrite(sync, LOW);
        SPI.transfer(registerWord[0]);
        SPI.transfer(registerWord[1]);
        SPI.transfer(registerWord[2]);
        digitalWrite(sync, HIGH);
        registerWord[0] = 0x00;
        registerWord[1] = 0x00;
        registerWord[2] = 0x00;
        digitalWrite(sync, LOW);
        registerWord[0] = SPI.transfer(0x00);
        registerWord[1] = SPI.transfer(0x00);
        registerWord[2] = SPI.transfer(0x00);
        digitalWrite(sync, HIGH);
        dataRead = ((long)registerWord[0] << 16) | ((long)registerWord[1] << 8) | ((long)registerWord[2] << 0);
        return dataRead;
    }
};

class SlopeTracker {
private:
    long y1 = 0;
    long y2 = 0;

    int biningCount = 3;
    int count = 0;
    bool firstPoint = true;

    bool ready = false;

public:
    /// <summary>
    /// Adds a point to the slope tracker. This assumes that you call this in the same time interval of unit 1.
    /// </summary>
    /// <param name="y">The point height</param>
    /// <returns></returns>
    void AddPoint(long y) {
        if (count > biningCount) {
            firstPoint = !firstPoint;
            count = 0;
            ready = true;
                            
        }
        else {
            ready = false;
        }

        if (firstPoint) {
            y1 += y;
        }
        else {
            y2 += y;
        }

        count++;
    }

    bool IsReady() {
        return ready;
    }

    bool IsSlopeRising() 
    {
        return (y2 - y1) / biningCount > 0;
    }

};

class LockSystem {
private://can only be changed in class
    int reflection;
    float error;
    int errorPhase = 20;
    int sineTableIndex = 0;
    long lastResonanceVolt = 0;

    int reflectionPin = A8;

    // Pk-Pk amplitude of sinewave in volts
    float sinAmplitude = 0.0;
    float correction = 0;

    double voltLowerLimit = 100000;
    double voltUpperLimit = 300000;

    bool engage = false;
    bool scanningDone = false;
    bool stateFlag = false;

    PdhSignalTracker PdhSignalTracker;

    IntervalTimer* intervalTimer;

    


    Dac* dac1;
    Dac* dac2;
    FilterBuLp2* lpf;
    PID* defaultPid;

    void GenerateSinetable(float amplitude) {
        for (int h = 0; h < 36; h++) //sinetable is generated
        {
            for (int j = 0; j < sinetablesize; j++)
            {
                sinetable[h][j] = max_bits * (amplitude * sin(j * 2 * pi / sinetablesize + 2 * h * pi / 36)) / 10;
            }
        }
    }

    float CalculateAcBits() {
        return max_bits * sinAmplitude / 5; // why divided by 5?
    }
        

    void GenerateSinetable() {
        GenerateSinetable(sinAmplitude);
    }
    void SetUpPins() {
        pinMode(A8, INPUT);
        Serial.begin(38400);
        SPI.begin();

        dac1->SetUpPins();
        dac2->SetUpPins();

        SPI.beginTransaction(SPISettings(3800000, MSBFIRST, SPI_MODE1));
    }
    void SetUpDacs() {

        unsigned long oldCtrl = dac1->GetControlValue();

        oldCtrl = oldCtrl & ~(AD5791_CTRL_LINCOMP(-1) | AD5791_CTRL_SDODIS | AD5791_CTRL_BIN2SC | AD5791_CTRL_RBUF | AD5791_CTRL_OPGND);

        dac1->WriteControlValue(oldCtrl);
        dac1->GetControlValue();
        dac1->LoadDac();

        dac1->WriteControlValue(oldCtrl_c);
        dac1->WriteDacValue(1);

        dac2->WriteControlValue(oldCtrl_c);
        dac2->WriteDacValue(1);

        //---------------------------------------
        analogReadResolution(12); // 1 point corresponds to 3.3V/2^(12) 
        analogReadAveraging(3);
        intervalTimer->begin(flagpost, Tsample); //reset flag to true every Tsample
        delay(1000);

        GenerateSinetable();

        dac2->WriteControlValue(oldCtrl_c);
        dac2->WriteDacValue(1);
        dac2->ResetOffset(); // DC offset for DAC2

        // initialize DAC1
        dac1->WriteControlValue(oldCtrl_c);
        dac1->WriteDacValue(1);
        dac1->ResetVoltageToLowerLimit();

        Serial.println("initialized");
        delay(1000);
    }
    
public:
    LockSystem(IntervalTimer* intervalTimer, PID* defaultPid, Dac* dac1, Dac* dac2, FilterBuLp2* lpf);

    void SetReflectionPin(int pin) {
        reflectionPin = pin;
    }

    void SetErrorPhase(int errorPhase) {
        this->errorPhase = errorPhase;
    }

    void SetVoltLowerLimit(double voltLowerLimit) {
        this->voltLowerLimit = voltLowerLimit;
    }

    void SetVoltUpperLimit(double voltUpperLimit) {
        this->voltUpperLimit = voltUpperLimit;
    }

    int GetReflection() {
        return reflection;
    }

    float GetError() {
        return error;
    }
    void SetUp() {
        SetUpPins();
        SetUpDacs();
    }

    int ReadReflectionValue() {
        return analogRead(reflectionPin) * 50;
    }

    float CalculateError(float reflection) {
        return reflection;
    }

    //    float CalculateError(float reflection) {
    //        return -lpf->main(sinetable[errorPhase][sineTableIndex] * reflection) / 50.0;
    //    }

    float CalculateError() {
        return CalculateError(ReadReflectionValue());
    }

    void MeasureReflectionAndCalculateError() {
        reflection = ReadReflectionValue();
        error = CalculateError(reflection);
    }

    void ExecuteVoltages() {
        sineTableIndex = (sineTableIndex + 1) % sinetablesize;
        SendVoltageToRegister(1); // sends the voltage to the first dac.
    }

    void SendVoltageToRegister(int dac) {
        if (dac == 1) {
            dac1->SendVoltageToRegister();
        }
        else if (dac == 2) {
            dac2->SendVoltageToRegister();
        }
    }

    void Zero() {
        dac1->Zero();
    }

    void OnScan() {
        dac1->Scan();
    }


    void OnDecreaseOffset() {
        dac2->StepDown();
    }

    void OnIncreaseOffset() {
        dac2->StepUp();
    }


    long count = 0;
    int pass = 0;
    void OnLock() {
        if (dac1->IsFinished()) {
            if (!engage) {
                defaultPid->setSetpoint(lastResonanceVolt);
            }

            engage = true;
            scanningDone = true;
            if (engage == 1)   //feedback on
            {
                // time_index = time_index + 1;
                //              if (state_flag == 0)    //switch pid off  state_flag==0
                //              {
                //                  if (freeze && time_index > pid_on_loops)
                //                  {
                //                    flag_monitor = 1000;
                //                    state_flag = 1; 
                //                    time_index = 0;          
                //                  }
                //              }
                if (stateFlag == false)// PID
                {
                    correction = defaultPid->CalculateCorrection(error);
                }
            }

            dac1->SetVoltOut(correction + sinetable[0][sineTableIndex]);
            dac1->SendVoltageToRegister();
        }
        else {
            if (dac1->IsScanFinished()) {
                pass++;
                // limit the scan range
                dac1->SetVoltUpperLimit(PdhSignalTracker.Maxima.GetZ());
                dac1->SetVoltLowerLimit(PdhSignalTracker.Minima.GetZ());
                dac1->Zero();
                PdhSignalTracker.Clear();
                PdhSignalTracker.SetPass(pass);
                count = 0;
            }
            else {
                dac1->Scan(sinetable[0][sineTableIndex]);

                float error = GetError();

                PdhSignalTracker.AddPoint(Point(count, error, dac1->GetVoltOut()));
                count++;


                //////////////////once the resonace found, scan a small range/////////////////
                //if (GetReflection() < 5000)  //last_resonance_volt initialized 0
                if (PdhSignalTracker.GetPassNumber() > 2)
                {
                    Serial.println("resonance found");
                    dac1->SetIsFinished(true);
                    //dvolt_DAC1 = dvolt_DAC1 / 8.0;
                    lastResonanceVolt = PdhSignalTracker.GetSetPoint();
                    //dac1->GetVoltOut();
                }
            }
            
        }
    }
};


LockSystem::LockSystem(IntervalTimer* intervalTimer, PID* defaultPid, Dac* dac1, Dac* dac2, FilterBuLp2* lpf) {
    this->intervalTimer = intervalTimer;
    this->defaultPid = defaultPid;
    this->dac1 = dac1;
    this->dac2 = dac2;
    this->lpf = lpf;
};

const float setpoint = 101000;
const float P = 0.1;   //0.5
const float I = 0.0;  //320 -> 250 Hz in ZI LPF min; 32 -> 15 Hz in ZI LPF min
const float D = 0.0; //0.15

FilterBuLp2 LPF;
Dac Dac1(reset1, clr1, ldac1, sync1);
Dac Dac2(reset2, clr2, ldac2, sync2);
IntervalTimer myTimer;
PID DefaultPid(P, I, D, setpoint);

LockSystem MainLock(&myTimer, &DefaultPid, &Dac1, &Dac2, &LPF);//build Mainlock, type LockSystem

LPF2 lpf2_1(lpf2_freq1);
LPF2 lpf2_2(lpf2_freq2);

//double volt_limit_down = 100000;
//double volt_limit_up = 300000;
//float DAC2_offset = 10;//find the proper voltage by delta source
//PDH LPF 1.28kHz

//works best with 1 kHz ZI LPF
//float P = 1.0;   //0.5
//float I = 0.1;  //0.4
//float D = 0.0; //0.15

//works best with 9 Hz ZI LPF, up to 3 Hz
//float P = 0.5;   //0.5
//float I = 0.1;  //0.4

//140 V on membrane piezo params: minimal I=60n






void setup()
{

    MainLock.SetUp();

}

int print_index = 0;
byte incomingByte = 122;

void loop()
{
    if (Serial.available())
    {
        incomingByte = Serial.read();  // will not be -1
        //Serial.println(incomingByte);   //  l=108; s=115;  z=122 https://www.rapidtables.com/code/text/ascii-table.html
        if (incomingByte == 80) // enter "P" in the serial port followed by a float value.
        {
            // try to read the P value from the serial
            float p = Serial.parseFloat();
            DefaultPid.setP(p);
        }

        if (incomingByte == 73) // enter "I" in the serial port followed by a float value.
        {
            // try to read the P value from the serial
            float i = Serial.parseFloat();
            DefaultPid.setI(i);
        }

        if (incomingByte == 68) // enter "D" in the serial port followed by a float value.
        {
            // try to read the P value from the serial
            float d = Serial.parseFloat();
            DefaultPid.setD(d);
        }

        if (incomingByte == 102) // enter "f" to change the phase of the error signal
        {
            float phase = Serial.parseFloat();
            MainLock.SetErrorPhase(phase);
        }

        if (incomingByte == 83) // enter "S" for setpoint
        {
            float setPoint = Serial.parseFloat();
            DefaultPid.setSetpoint(setPoint);
        }

        if (incomingByte == 85)
        {
            float upper = Serial.parseFloat();
            Dac1.SetVoltUpperLimit(upper);
        }

        if (incomingByte == 76)
        {
            float lower = Serial.parseFloat();
            Dac1.SetVoltLowerLimit(lower);
        }
    }


    if (flag)
    {
        //Serial.print("flag");
        flag = false;

        // not needed anymore. Because we have added the reset to lower limit to the setup code.
        //if (volt_out_DAC1 == -100)
        //    volt_out_DAC1 = volt_limit_down + 1; // going to rename volt_limit_down to voltLowerLimit

        MainLock.MeasureReflectionAndCalculateError();
        //TODO: make this into its own function inside the MainLock

       //error = 2 * Refl;//14000-27000  
       //error = 2 * lpf2_1.main(Refl);//14000-27000  



        MainLock.ExecuteVoltages();



        ///////////////////////////////////////////////////////////////
        if (incomingByte == 122)  //"z" for zero
        {
            //Serial.println("z");
            MainLock.Zero();
            DefaultPid.resetSum();
        }
        ///////////////////////////////////////////////////////////////  
        if (incomingByte == 115)  //"s" for scan
        {
            //Serial.println("scan");
            MainLock.OnScan();
            //+ sinetable[0][r];
            DefaultPid.resetSum();
        }
        ///////////////////////////////////////////////////////////////  
        if (incomingByte == 100)  //"d" for down, decreases DAC2 voltage
        {
            MainLock.OnDecreaseOffset();
        }
        ///////////////////////////////////////////////////////////////  
        if (incomingByte == 117)  //"u" for up, increases DAC2 voltage
        {
            MainLock.OnIncreaseOffset();
        }
        ///////////////////////////////////////////////////////////////   
        if (incomingByte == 108) //lock
        {
            MainLock.OnLock();
        }

        print_index++;
        if (print_index == 100)
        {
            Serial.print("DAC1:");
            Serial.print(Dac1.GetVoltOut());
            Serial.print(",");
            Serial.print("Reflection:");
            Serial.print(MainLock.GetReflection());
            Serial.print(",");
            Serial.print("Error:");
            Serial.print(MainLock.GetError());

            if (plotPIDValues) {
                Serial.print(",");
                float scale = 1000;
                Serial.print("P:");
                Serial.print(scale * DefaultPid.getP() + scale);
                Serial.print(",");
                Serial.print("I:");
                Serial.print(scale * DefaultPid.getI());
                Serial.print(",");
                Serial.print("D:");
                Serial.print(scale * DefaultPid.getD() - scale);
            }
            Serial.println();
            print_index = 0;
        }
    }
}
