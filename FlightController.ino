#include <ESP32Servo.h>
#include "I2Cdev.h"
#include <SimpleKalmanFilter.h>
#include "MPU6050_6Axis_MotionApps20.h"
#include "MPU6050.h" // not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high

/* =========================================================================
   NOTE: In addition to connection 3.3v, GND, SDA, and SCL, this sketch
   depends on the MPU-6050's INT pin being connected to the Arduino's
   external interrupt #0 pin. On the Arduino Uno and Mega 2560, this is
   digital I/O pin 2.
 * ========================================================================= */

/* =========================================================================
   NOTE: Arduino v1.0.1 with the Leonardo board generates a compile error
   when using Serial.write(buf, len). The Teapot output uses this method.
   The solution requires a modification to the Arduino USBAPI.h file, which
   is fortunately simple, but annoying. This will be fixed in the next IDE
   release. For more info, see these links:

   http://arduino.cc/forum/index.php/topic,109987.0.html
   http://code.google.com/p/arduino/issues/detail?id=958
 * ========================================================================= */



// uncomment "OUTPUT_READABLE_QUATERNION" if you want to see the actual
// quaternion components in a [w, x, y, z] format (not best for parsing
// on a remote host such as Processing or something though)
//#define OUTPUT_READABLE_QUATERNION

// uncomment "OUTPUT_READABLE_EULER" if you want to see Euler angles
// (in degrees) calculated from the quaternions coming from the FIFO.
// Note that Euler angles suffer from gimbal lock (for more info, see
// http://en.wikipedia.org/wiki/Gimbal_lock)
//#define OUTPUT_READABLE_EULER

// uncomment "OUTPUT_READABLE_YAWPITCHROLL" if you want to see the yaw/
// pitch/roll angles (in degrees) calculated from the quaternions coming
// from the FIFO. Note this also requires gravity vector calculations.
// Also note that yaw/pitch/roll angles suffer from gimbal lock (for
// more info, see: http://en.wikipedia.org/wiki/Gimbal_lock)
#define OUTPUT_READABLE_YAWPITCHROLL

// uncomment "OUTPUT_READABLE_REALACCEL" if you want to see acceleration
// components with gravity removed. This acceleration reference frame is
// not compensated for orientation, so +X is always +X according to the
// sensor, just without the effects of gravity. If you want acceleration
// compensated for orientation, us OUTPUT_READABLE_WORLDACCEL instead.
//#define OUTPUT_READABLE_REALACCEL

// uncomment "OUTPUT_READABLE_WORLDACCEL" if you want to see acceleration
// components with gravity removed and adjusted for the world frame of
// reference (yaw is relative to initial orientation, since no magnetometer
// is present in this case). Could be quite handy in some cases.
//#define OUTPUT_READABLE_WORLDACCEL

// uncomment "OUTPUT_TEAPOT" if you want output that matches the
// format used for the InvenSense teapot demo
//#define OUTPUT_TEAPOT

#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false,arm=false;
long tim1,tim2,tim;
// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
int32_t GyroData[3];
// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };



// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}


// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================
SimpleKalmanFilter thrustInputFilter(2,2,0.06),rollInputFilter(2, 2, 0.07),pitchInputFilter(2,2,0.07),yawInputFilter(2,2,0.07),estRollO(2,2,2),estPitchO(2,2,2),estGyroY(2,2,0.001);
SimpleKalmanFilter estGyroX(2,2,0.001),estGyroZ(2,2,0.001);
Servo ESC1,ESC2,ESC3,ESC4;
int ran,speeds=1000;
float thrust,estThrust,tempRoll,tempPitch,tempYaw,previousThrust,prevRoll,prevPitch,estRoll,estPitch,estYaw,roll,pitch,yaw,resetYaw,prevYaw,inputRoll,prevInputRoll,prevInputPitch,prevInputYaw,estInputRoll,estInputPitch,estInputYaw,inputYaw,inputPitch;
float tempInputYaw,tempInputPitch,tempInputRoll;
float MFRvalue,MFLvalue,MBRvalue,MBLvalue,prevMFR,prevMFL,prevMBR,prevMBL,tempMFRvalue,tempMFLvalue,tempMBRvalue,tempMBLvalue;
float rollMotor=0,pitchMotor=0,yawMotor=0,rollMotorMax = 400;
bool flagDer = 1;
//Servo MBR, MFL, MFR, MBL;
float rollIntegeral,rollDerivative,pitchIntegeral,yawIntegeral,pitchDerivative,lastRollError,lastPitchError,yawDerivative,lastYawError;
float rollKp=1.1,pitchKp=1.1,yawKp=4,rollKi=0.03,pitchKi=0.03,pitchKd=35,rollKd=35,yawKd=0,yawKi=0.0009;
unsigned long timer_1, timer_2, timer_3, timer_4, current_time1,current_time2,current_time3,current_time4,maintainTime;
int receiver[5];
int check = 1;
float yawMinus,gy,estgy,gx,estgx,gz,estgz;
/*struct receiverFilterRoll
{
  float receiverRoll;
  struct receiverFilterRoll *next;
}*head;
*/
void IRAM_ATTR Ch1() {
    if(digitalRead(16) == HIGH)
    {                                                                                         
       current_time1 = micros();
    }
    else
    { 
      timer_1 = micros(); 
      receiver[1] = timer_1 - current_time1; 
    }
}

void IRAM_ATTR Ch2() {
    if(digitalRead(4) == HIGH)
    {                                                                                         
       current_time2 = micros();
    }
    else
    { timer_2 = micros(); 
      receiver[2] = timer_2 - current_time2; 
    }
}

void IRAM_ATTR Ch3() {
    if(digitalRead(5) == HIGH)
    {                                                                                         
       current_time3 = micros();
    }
    else
    { timer_3 = micros(); 
      receiver[3] = timer_3 - current_time3; 
    }
}
void IRAM_ATTR Ch4() {
    if(digitalRead(17) == HIGH)
    {                                                                                         
       current_time4 = micros();
    }
    else
    { timer_4 = micros(); 
      receiver[4] = timer_4 - current_time4; 
    }
}

void setup() {
  pinMode(16, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(16), Ch1, CHANGE);
  pinMode(4, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(4), Ch2, CHANGE);
  pinMode(5, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(5), Ch3, CHANGE);
  pinMode(17, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(17), Ch4, CHANGE);

  pinMode(12,OUTPUT); // for 3.3 volt to logic converter
  digitalWrite(12,HIGH);
  pinMode(15,OUTPUT);
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  ESC1.setPeriodHertz(400);
  ESC2.setPeriodHertz(400);
  ESC3.setPeriodHertz(400);
  ESC4.setPeriodHertz(400);
  ESC1.attach(23,700,2000); //Front Left
  ESC2.attach(32,700,2000); //Front Right
  ESC3.attach(26,700,2000); //Back Right
  ESC4.attach(33,700,2000); //Back Left
  ESC1.writeMicroseconds(1000);
  ESC2.writeMicroseconds(1000);
  ESC3.writeMicroseconds(1000);
  ESC4.writeMicroseconds(1000);
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    // (115200 chosen because it is required for Teapot Demo output, but it's
    // really up to you depending on your project)
    Serial.begin(500000);
    while (!Serial); // wait for Leonardo enumeration, others continue immediately

    // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3V or Arduino
    // Pro Mini running at 3.3V, cannot handle this baud rate reliably due to
    // the baud timing being too misaligned with processor ticks. You must use
    // 38400 or slower in these cases, or use some kind of external separate
    // crystal solution for the UART timer.

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

    // verify connection
//    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
      
    // wait for ready
    /*Serial.println(F("\nSend any character to begin DMP programming and demo: "));
    while (Serial.available() && Serial.read()); // empty buffer
    while (!Serial.available());                 // wait for data
    while (Serial.available() && Serial.read()); // empty buffer again
    */
    // load and configure the DMP
//    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // Calibration Time: generate offsets and calibrate our MPU6050
        //mpu.CalibrateAccel(6);
        //mpu.CalibrateGyro(6);
        //mpu.PrintActiveOffsets();
        // turn on the DMP, now that it's ready
//        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
/*        Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
        Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
        Serial.println(F(")..."));                                   */
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
 //       Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
/*        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));   */
    }
      //delay(16000);
    // configure LED for output
 //   pinMode(LED_PIN, OUTPUT);
    //pinMode(5,OUTPUT);
}



// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
    //maintainTime = micros() + 10500;
  //digitalWrite(5,HIGH);
    // if programming failed, don't try to do anything
    if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {
        if (mpuInterrupt && fifoCount < packetSize) {
          // try to get out of the infinite loop 
          fifoCount = mpu.getFIFOCount();
        }
        if(receiver[4] < 2020 && receiver[4] > 990)
        {
          thrust = receiver[4];
        }
        //inputRoll = mapi(receiver[2],1000,2000,-170,170);
        if(receiver[2] > 1520 && receiver[2] < 2010) 
        {
          tempInputRoll = (receiver[2] - 1520)*0.03 + prevInputRoll*0.97;
        }
       else if(receiver[2] < 1480 && receiver[2] > 990)
       {
         tempInputRoll = (receiver[2] - 1480)*0.03 + prevInputRoll*0.97;
       }
       else
       {
         tempInputRoll = prevInputRoll*0.97;
       }
        inputRoll = tempInputRoll/4;

       if(receiver[1] > 1520 && receiver[1] < 2010)
       {
          tempInputPitch = (receiver[1] - 1520)*0.03 + prevInputPitch*0.97;
       }
       else if(receiver[1] < 1480 && receiver[1] > 990)
       {
          tempInputPitch = (receiver[1] - 1480)*0.03 + prevInputPitch * 0.97;
       }  
       else
       {
         tempInputPitch = prevInputPitch*0.97;
       }
       
        inputPitch = -tempInputPitch/4;
            
        if(receiver[3] > 1520 && receiver[3] < 2010)
       {
          tempInputYaw = (receiver[3] - 1520)*0.03 + prevInputYaw*0.97;
       }
       else if(receiver[3] < 1480 && receiver[3] > 990)
       {
          tempInputYaw = (receiver[3] - 1480)*0.03 + prevInputYaw * 0.97;
       }
       else
       {
          tempInputYaw = prevInputYaw*0.97;
       }
        inputYaw = tempInputYaw/12;


        if(thrust < 1100 &&  estInputYaw < -20)
        {
          arm = true;
          if(check == 1)
          {
           yawMinus = yaw+yawMinus;
           check = 0; 
          }
        }
        if(thrust < 1100 && estInputYaw > 20)
        {
          arm = false;
          yawIntegeral = 0;
          yawMinus = 0;
          check = 1;
        }

        if(estThrust < 1100 || arm == false)
        {
          ESC1.writeMicroseconds(1000);
          ESC2.writeMicroseconds(1000);
          ESC3.writeMicroseconds(1000);
          ESC4.writeMicroseconds(1000);
          pitchIntegeral = 0;
          rollIntegeral = 0;
          yawIntegeral = 0;
        }else 
        { 
        //ran = 2;
        ESC2.writeMicroseconds(int(MFRvalue));
        ESC3.writeMicroseconds(int(MBRvalue));
        ESC4.writeMicroseconds(int(MBLvalue));
        ESC1.writeMicroseconds(int(MFLvalue));
        /*ESC1.writeMicroseconds(thrust+inputPitch-inputRoll);
        ESC2.writeMicroseconds(thrust);
        ESC3.writeMicroseconds(thrust-inputPitch+inputRoll);
        ESC4.writeMicroseconds(thrust);*/
        }
        // .
        // .
        // .
        // if you are really paranoid you can frequently test in between other
        // stuff to see if mpuInterrupt is true, and if so, "break;" from the
        // while() loop to immediately process the MPU data
        // .
        // .
        // .
    }

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();
  if(fifoCount < packetSize){
          //Lets go back and wait for another interrupt. We shouldn't be here, we got an interrupt from another event
      // This is blocking so don't do it   while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
  }
    // check for overflow (this should never happen unless our code is too inefficient)
    else if ((mpuIntStatus & (0x01 << MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount >= 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
      //  fifoCount = mpu.getFIFOCount();  // will be zero after reset no need to ask
//        Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & (0x01 << MPU6050_INTERRUPT_DMP_INT_BIT)) {

        // read a packet from FIFO
  while(fifoCount >= packetSize){ // Lets catch up to NOW, someone is using the dreaded delay()!
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;
  }
        
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            //gy = mpu.getRotationY();
            //gx = mpu.getRotationX();
            //gz = mpu.getRotationZ();
            mpu.dmpGetGyro(GyroData,fifoBuffer);
            roll = ypr[2]*180/M_PI;
            pitch = -ypr[1]*180/M_PI;
            yaw = ypr[0]*180/M_PI - yawMinus;
            prevInputRoll = tempInputRoll;
            prevInputPitch = tempInputPitch;
            prevInputYaw = tempInputYaw;
            //thrust = 1400;
            estThrust = thrustInputFilter.updateEstimate(thrust);
            estInputRoll = rollInputFilter.updateEstimate(inputRoll);
            estInputPitch = pitchInputFilter.updateEstimate(inputPitch);
            estInputYaw = yawInputFilter.updateEstimate(inputYaw);
            estRoll = estRollO.updateEstimate(roll/4);
            estPitch = estPitchO.updateEstimate(pitch/4);
            //estRoll = roll/4;
            //estPitch = pitch/4;
            estYaw = yaw/4;
            estgx = estGyroX.updateEstimate(GyroData[0]/50000);
            estgy = estGyroY.updateEstimate(GyroData[1]/50000);
            estgz = estGyroZ.updateEstimate(GyroData[2]/50000);
            //estgx = GyroData[0]/50000;
            //estgy = GyroData[1]/50000;
            //estgz = GyroData[2]/50000;
              //Serial.print(tim);
              //Serial.print("\t");
          // if(roll>0 || pitch >0 )
          // {
            /*Serial.print("ypr\t");
            Serial.print(ypr[0] * 180/M_PI);
            Serial.print("\t");
            Serial.print(ypr[1] * 180/M_PI);
            Serial.print("\t");
            Serial.println(tim); */
            // .........................................................//
           //Serial.print("ypr\t");
/*            Serial.print(rollDerivative);
            Serial.print("\t");
            Serial.print(ypr[2]*180);
            Serial.print("\t");*/
            
           //}
            /*Serial.print(receiver[1]);
            Serial.print("\t");
            Serial.print(receiver[2]);
            Serial.print("\t");
            Serial.print(receiver[3]);
            Serial.print("\t");  
            Serial.println(receiver[4]);*/
            //estThrust = 1400;
        Serial.println(estgx);
        //Serial.print("\t");
        //Serial.println(estPitch);     
        PIDroll(-estgx-(estRoll - estInputRoll),-estRoll + estInputRoll/20);
        PIDpitch(-estgy-(estPitch - estInputPitch),-estPitch + estInputPitch/20);
        PIDyaw(estgz-(estInputYaw),-estYaw - estInputYaw/12);
        MotorMixing();     
       
    } 
    tim2 = micros();
    tim = int(tim2 - tim1);
    tim1 = micros();
}

float mapi(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
void MotorMixing()
{ 
  tempMFRvalue = estThrust+yawMotor-pitchMotor-rollMotor;
  tempMFLvalue = estThrust-yawMotor-pitchMotor+rollMotor;
  tempMBRvalue = estThrust-yawMotor+pitchMotor-rollMotor+60;
  tempMBLvalue = estThrust+yawMotor+pitchMotor+rollMotor+60;
  if(tempMFRvalue < 1000) 
  {
    MFRvalue = 1000;
  }else if(tempMFRvalue > 2000)
  {
    MFRvalue = 2000;
  } else {
    MFRvalue = tempMFRvalue;
  }

  if(tempMFLvalue < 1000) 
  {
    MFLvalue = 1000;
  }else if(tempMFLvalue > 2000)
  {
    MFLvalue = 2000;
  }else{
    MFLvalue = tempMFLvalue;
  }

  if(tempMBLvalue < 1000) 
  {
    MBLvalue = 1000;
  }else if(tempMBLvalue > 2000)
  {
    MBLvalue = 2000;
  }else{
    MBLvalue = tempMBLvalue;
  }

  if(tempMBRvalue < 1000) 
  {
    MBRvalue = 1000;
  }else if(tempMBRvalue > 2000)
  {
    MBRvalue = 2000;
  }else{
    MBRvalue = tempMBRvalue;
  }
  //Serial.println(MFLvalue);
}
                            //     1 - 0
void PIDroll(float RollError,float autoLevelRollError)
{
  //if((error-lastRollError) < 20 && lastRollError-error)
  rollDerivative = (RollError-lastRollError);
    lastRollError = RollError;
    rollIntegeral = (rollIntegeral + (autoLevelRollError)*rollKi);
  if(rollIntegeral > 150)
  {
    rollIntegeral = 150; 
  }
   if(rollIntegeral < -150)
  {
    rollIntegeral = -150; 
  }
  //Serial.print("\t");Serial.print(rollMotor);Serial.print("\t");Serial.print(pitchMotor);Serial.print("\n");
  float tempRollMotor = (rollKp*RollError + rollKd*rollDerivative+ rollIntegeral);
  if(tempRollMotor < 400 && tempRollMotor > -400)
  {
    rollMotor = tempRollMotor;
  }
  //Serial.print(rollDerivative);Serial.print("\t\t\t");
  //Serial.println(rollDerivative*1000);
}

void PIDpitch(float PitchError,float autoLevelPitchError)
{ 
  pitchDerivative = (PitchError-lastPitchError);
  lastPitchError = PitchError;
  //Serial.println(pitchDerivative*100);
    pitchIntegeral = (pitchIntegeral + (autoLevelPitchError)*pitchKi);
  if(pitchIntegeral > 150)
  {
    pitchIntegeral = 150; 
  }
   if(pitchIntegeral < -150)
  {
    pitchIntegeral = -150; 
  }
  float tempPitchMotor = (pitchKp*PitchError + pitchKd*pitchDerivative+pitchIntegeral);
  if(tempPitchMotor  > -400 && tempPitchMotor < 400)
  {
    pitchMotor = tempPitchMotor;
  }
  // YAW INTEGRAL
  /*if(yawError>-25 && yawError<25)
  {
    yawIntegeral = (yawIntegeral+(yawError)*elapsedTime*0.008);
  }
  if(pitchIntegeral > 40)
  {
    yawIntegeral = yawIntegeral - 2; 
  }
   if(pitchIntegeral < -40)
  {
    yawIntegeral = yawIntegeral + 2; 
  }*/
  //Serial.print(pitchDerivative);Serial.print("\n");
  //Serial.println(String(pitchMotor)+"\t"+String(rollMotor));
}
void PIDyaw(float error, float autoYawLevelError)
{
  yawDerivative = (error-lastYawError);
  lastYawError = error;
  yawIntegeral = yawIntegeral + (autoYawLevelError)*yawKi;
  if(yawIntegeral > 100)
  {
    yawIntegeral = 100;
  }
  else if(yawIntegeral < -100)
  {
    yawIntegeral = -100;
  }
  float tempYawMotor = (yawKp*error+yawKd*yawDerivative + yawIntegeral);
  if(tempYawMotor < 300 && tempYawMotor > -300)
  {
    yawMotor  = tempYawMotor;
  }
}
