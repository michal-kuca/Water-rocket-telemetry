

#include <Adafruit_BMP280.h>
#include <Wire.h>
#include <SoftwareSerial.h>

#define xPin  (A1) //pins in arduino that go into x y and z values in accelerometer
#define yPin  (A2)
#define zPin  (A3)

#define USE_BMP280

 

Adafruit_BMP280 climate_sensor;

//HC-12 Wireless Transceiver
SoftwareSerial HC12(7,6); //7 is TX and 6 is RX

//thresholds for acceleration (m/s^2)
#define VELOCITY_INTEGRATION_THRESHOLD 3 //threshold for velocity calculation
#define ABSOLUTE_ACCELERATION_THRESHOLD 2 //assumes stationary if lower

//threshold for state machine (m)
#define LAUNCH_ALTITUDE_TRIGGER 1.5 //trigger for mode switch to launch
#define DESCEND_ALTITUDE_DECREASE 1 //trigger for mode switch to descending

#define REST_ACCELERATION_NOISE 4 //tolerated acceleration noise, in detecting landing
#define LANDED_ALTITUDE 1//maximum landed altitude



//velocity starts at 0
float velocity_x = 0;
float velocity_y = 0;
float velocity_z = 0;
float velocity_abs = 0;


float Altitude_max;
float Acceleration_ascent_max=0;
float Acceleration_descent_max=0;
float Flight_duration;
float maxVelocity_zAscent = 0;
float maxVelocity_zDescent = 0;
float ground_pressure;
float launch_temp;
float Temp_min=100;

float rocket_mass=0.25;//mass in kg

float force=0;

float T_prelaunch=0;

float T_accel_ascent;

float T_Altitude_max;

float T_accel_descent;

//smoothing out data function
const int filterSize = 6;
float xAccelBuffer[filterSize]={0};
float yAccelBuffer[filterSize]={0};
float zAccelBuffer[filterSize]={0};
float pressureBuffer[filterSize]={0};
float temperatureBuffer[filterSize]={0};
float altitudeBuffer[filterSize]={0};

//functions
float calculatevelocity(float v,float a,float dt);
float movingAverage(float* buffer, float newValue);
float readAcceleration(float rawValue, float offset, float scale);
//add rocket state

int landedwait=0;


//Measured values after the Accel Calibration111
 

const int X1gMax = 396; //g acting on x
const int X1gMin = 258; 

const int Y1gMax = 400; //g acting on y
const int Y1gMin = 264; 

const int Z1gMax = 433; //g acting on z
const int Z1gMin = 296;



 // offset and scale calculations
 float Z_offset = (Z1gMax + Z1gMin)/2;
 float Z_scale = (Z1gMax - Z1gMin)/2;

 float Y_offset = (Y1gMax + Y1gMin)/2;
 float Y_scale = (Y1gMax - Y1gMin)/2;

 float X_offset = (X1gMax + X1gMin)/2;
 float X_scale = (X1gMax - X1gMin)/2;

float dt=0;

float sumP=0; //for finding ground pressure and temperature
float sumT=0;

 unsigned long Tlast;
 unsigned long Tnew;
 
unsigned long Sampling_interval=100; //sampling time in ms
unsigned long nextSampleTime;

int DES_CNT=0;
int ASC_CNT=0;
int LAN_CNT=0;

const int PRELAUNCH =0;
const int ASCENT =1;
const int DESCENT =2;
const int LANDED =3;

int mode = PRELAUNCH; //current mode

int threshold=0;

  void setup() {
    

  

  Tlast=millis(); //should be the time right before the main loop starts running
    // put your setup code here, to run once:
      HC12.begin(9600); //begins printing to transciever
      HC12.println("mode,Y acceleration,Absolute acceleration,Y velocity,Absolute velocity,Altitude,Temperature,Force,Time since launch");
 
    
      Serial.begin(9600); //begins printing to code


      //check if BME is working
      Wire.begin();
      if (!climate_sensor.begin(0x76)) {  //if climate sensor fails
          Serial.println("Climate Sensor sensor not found!");
          while (1);
      }
      Serial.println("BME280 working");


      //find ground pressure at start

      for(int i=0;i<10;i++){

        float P=climate_sensor.readPressure()/100;
        
        float T=climate_sensor.readTemperature();


        sumP=sumP+P;
        sumT=sumT+T;
        delay(50); //the pressure readings will be done in 0.5s
      }
       ground_pressure= sumP/10; //takes the average
       launch_temp= sumT/10;

       nextSampleTime = millis(); //starts timer
    }




  void loop() {

    // put your main code here, to run repeatedly:


      // reads current values
      float rawValueY= analogRead(yPin);
      float rawValueZ= analogRead(zPin);
      float rawValueX= analogRead(xPin);

      //reads values from function
      float Z_accel = readAcceleration( rawValueZ,  Z_offset,  Z_scale);
      float X_accel = readAcceleration( rawValueX,  X_offset,  X_scale);
      float Y_accel = readAcceleration( rawValueY,  Y_offset,  Y_scale)-9.81;


      float altitude=climate_sensor.readAltitude(ground_pressure); //measures m above reference (ground) point

      float pressure = climate_sensor.readPressure()/100;//converts to kilopascals

      float temperature = climate_sensor.readTemperature();
      
      float temp_smooth=movingAverage(temperatureBuffer,temperature);

      //normalizes accel values
      float X_smooth=movingAverage(xAccelBuffer, X_accel);
      float Y_smooth=movingAverage(yAccelBuffer, Y_accel);
      float Z_smooth=movingAverage(zAccelBuffer, Z_accel);

            
      float alt_smooth=movingAverage(altitudeBuffer, altitude);

  
     
      float pressure_smooth=movingAverage(pressureBuffer,pressure);
      



      

      //prints normalised values for accel
 

      //finds absolute acceleration using pythagoras
      float ab_accel=(sqrt(X_smooth*X_smooth+Y_smooth*Y_smooth+Z_smooth*Z_smooth));

        if (ab_accel<ABSOLUTE_ACCELERATION_THRESHOLD){ //if acceleration is less than threshold, assume stationary to prevent drift
          ab_accel=0;
        }

        if (mode>0){ //only start integrating after launch
        threshold=1; //threshold for integrating velocity, only starts once 6m/s is reached
        }

      //timer for integration: in function to be synced with acceleration   
        Tnew=millis();
        dt=(Tnew-Tlast)/1000.0;
      //changed into: (seconds)
      
      if (threshold==1){
        velocity_x=calculatevelocity(velocity_x,X_smooth,dt, ab_accel); //calculates (x) velocity

    

        velocity_y=calculatevelocity(velocity_y,Y_smooth,dt, ab_accel); //calculates (y) velocity

    

        velocity_z=calculatevelocity(velocity_z,Z_smooth,dt, ab_accel); //calculates (z) velocity

        velocity_abs=sqrt(velocity_x*velocity_x+velocity_y*velocity_y+velocity_z*velocity_z);  //calculates (absolute) velocity
        
        }

      //read and prints pressure values - 
    
 
      force = rocket_mass*ab_accel;


      //find altitude from this
     

      // altitude_calc= also use formula and compare?

      //climate sensor
    
    

    


    
      //finds current mode
mode = modefunc(mode,ab_accel,alt_smooth);

HC12.print(mode);
HC12.print(",");
HC12.print(Y_smooth);
HC12.print(",");
HC12.print(ab_accel);
HC12.print(",");
HC12.print(velocity_y);
HC12.print(",");
HC12.print(velocity_abs);
HC12.print(",");
HC12.print(alt_smooth);
HC12.print(",");
HC12.print(temp_smooth);
HC12.print(",");
HC12.print(force);
HC12.print(",");
HC12.println((millis()/1000)-T_prelaunch);

  if(alt_smooth>Altitude_max){

  Altitude_max=alt_smooth;
 T_Altitude_max=millis()/1000.0;
  }

   if(temp_smooth<Temp_min){

  Temp_min=temp_smooth;
  }

  
  

  if (mode==PRELAUNCH){

  //timer before launch


    T_prelaunch=(millis()/1000);


 
  


  }

  else if(mode==ASCENT){ //capture max ascent acceleration and velocity, print velocity ascent etc

     if(ab_accel>Acceleration_ascent_max){

      Acceleration_ascent_max=ab_accel;

      T_accel_ascent=millis()/1000.0;
    }
    if(velocity_abs>maxVelocity_zAscent){

      maxVelocity_zAscent=velocity_abs;

      
    }

  }

  else if(mode==DESCENT){ //capture descent acceleration and decceleration (due to parashute), print velocity descent
  //calculate change in accel to see when parashute is deployed?

      if(ab_accel>Acceleration_descent_max){

        Acceleration_descent_max=ab_accel;
        T_accel_descent = millis()/1000;

        
      }
      if(velocity_abs>maxVelocity_zDescent){

        maxVelocity_zDescent=velocity_abs;
      }


  } 
  else if(mode==LANDED){

    Flight_duration=(millis()/1000)-T_prelaunch;

  landedwait++;

   if (landedwait>40){

    


    //print stats for the flight (the max velocity etc...), time in air, state variables
      HC12.print("the rocket launch was excecuted at a temperature of:  ");
      HC12.println(launch_temp);

      HC12.print("the maximum acceleration during its accent was:  ");
      HC12.print(Acceleration_ascent_max);
      HC12.print("m/s^2  , at time: T+ ");
      HC12.println(T_accel_ascent);


      HC12.print("the maximum ascent velocity reached was:  ");
      HC12.print(maxVelocity_zAscent);
      HC12.println("  m/s ");

      HC12.print("the maximum altitude rea0ched was:  ");
      HC12.print(Altitude_max);
      HC12.print("  m , at time: T+ ");
      HC12.println(T_Altitude_max);

      HC12.print("At a chilly: ");
      HC12.print(Temp_min);
      HC12.println("  C");

      HC12.print("the maximum acceleration during its descent was:  ");
      HC12.print(Acceleration_descent_max);
      HC12.print("m/s^2  , at time: T+ ");
      HC12.println(T_accel_descent);

      HC12.print("the maximum altitude reached was:  ");
      HC12.print(Altitude_max);
      HC12.print("  , at time: T+ ");
      HC12.println(T_Altitude_max);

      HC12.print("The rocket landed safely at a time of: T+ ");
      HC12.print(Flight_duration);
      HC12.print(" s");
      HC12.print("With a maximum descent velocity of:  ");
      HC12.print(maxVelocity_zDescent);
      HC12.println("  m/s ");
       HC12.print("The maximum thrust was: ");
      HC12.print(Acceleration_ascent_max*rocket_mass);
      HC12.println(" N");


     delay(10000);
   }
       
  }
      


    Tlast=Tnew;
    nextSampleTime = nextSampleTime + Sampling_interval; //sets the next sampling timestamp

    long remainingTime = nextSampleTime - millis(); ///difference between sampling ttime and current time
    if (remainingTime > 0) {
        delay(remainingTime); //actively delays by the interval-processing time
    } else {
        //if processing delay is enough, don't delay at all
        nextSampleTime = millis();  // resync
    }
  }

//functions

 //function for converting current values to acceleration
float readAcceleration(float rawValue, float offset, float scale){
  float accel = 0.0;
  accel = (rawValue - offset)*(9.81 / scale);

  return accel;
}

//returns moving average num
float movingAverage(float* buffer, float newValue){
  float sum=0;

  //store previous 5 values
  for (int i=filterSize-1;i>0;i--){

    //shifts values by 1
    buffer[i]=buffer[i-1];

  }
    buffer[0]= newValue;
    //adds in new value

  for (int i=0;i<filterSize;i++){
    sum=sum + buffer[i];
   

  }
   return sum/filterSize;
  }


//calculates velocity
float calculatevelocity(float v,float a,float dt, float abs){
  if(abs<VELOCITY_INTEGRATION_THRESHOLD){ //does nothing if acceleration threshold isnt reached
   return v*0.98;
  }

  else{
    return (v+a*dt)*0.98;

}
}

  int modefunc(int mode, float ab_accel,float alt_smooth){
    int modeNEW = mode;   //keep the same mode if conditions are not met

    if((mode==0)&&(alt_smooth>LAUNCH_ALTITUDE_TRIGGER))
    {
     
      ASC_CNT++;
      
      //descending //ascending: z acceleration,
    }
    else if((mode==1)&&(alt_smooth < Altitude_max-DESCEND_ALTITUDE_DECREASE)&&(millis()>2000))
    { 

        DES_CNT++;
        
    }

    else if ((mode==2)&&((ab_accel < REST_ACCELERATION_NOISE)||(alt_smooth < LANDED_ALTITUDE)||(millis()>40000)))
    {
     
      LAN_CNT++;
      
    }
    else{}

    

 if(LAN_CNT>2) //change to whichever axis is alighned with g
    {
     modeNEW = LANDED; //ascending: z acceleration,
    }
    
    else if(DES_CNT>2)
    { 
     modeNEW = DESCENT; //descending
    }
    
    else if(ASC_CNT>2)
    { 
     modeNEW = ASCENT; //descending
    }
    else{}

   mode=modeNEW;

    return mode;


   }



