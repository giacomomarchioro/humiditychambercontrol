// Example testing sketch for various DHT humidity/temperature sensors
// Written by ladyada, public domain

// REQUIRES the following Arduino libraries:
// - DHT Sensor Library: https://github.com/adafruit/DHT-sensor-library
// - Adafruit Unified Sensor Lib: https://github.com/adafruit/Adafruit_Sensor

#include "DHT.h"

#define DHTPIN 13     // Digital pin connected to the DHT sensor

// Uncomment whatever type you're using!
#define DHTTYPE DHT11   // DHT 11
//#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321
//#define DHTTYPE DHT21   // DHT 21 (AM2301)
// DHT11 works with +5 or 3,3V 


// Initialize DHT sensor.
// Note that older versions of this library took an optional third parameter to
// tweak the timings for faster processors.  This parameter is no longer needed
// as the current DHT reading algorithm adjusts itself to work on faster procs.
DHT dht(DHTPIN, DHTTYPE);





// connect motor controller pins to Arduino digital pins
// motor one
int enA = 10;
int in1 = 9;
int in2 = 8;
// motor two humidity reservoir 
int enB = 5;
int in3 = 7;
int in4 = 6;
double target_HR =  40;

//PID constants
double kp = 2;
double ki = 5;
double kd = 1;
double output = 100;
unsigned long currentTime, previousTime;
double elapsedTime = 0;
double error = 0;
double lastError = 0;
double input = 0;
double cumError = 0;
double rateError = 0;
double errotimeselaps = 0;

void setup()
{
  // set all the motor control pins to outputs
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  // this is for the humidity sensor 
  // seria comunication take time
  Serial.begin(9600);
  previousTime =  millis();
  dht.begin();
}

double computePID(double inp){     
        currentTime = millis();                //get current time
        Serial.print("Previous Time: ");
        Serial.println(previousTime);
        elapsedTime = (double)(currentTime - previousTime);        //compute time elapsed from previous computation
        // Proportional part
        error = target_HR - inp;                                // determine error
        Serial.print("Eroor: ");
        Serial.println(error);
        
        Serial.print("ElapsedTime: ");
        Serial.println(elapsedTime);
        errotimeselaps = error * elapsedTime;
        Serial.print("errotimeselaps: ");
        Serial.println(errotimeselaps);
        if(isnan(cumError)){
          cumError =0;
          };
        cumError += errotimeselaps;         // compute integral
        rateError = (error - lastError)/elapsedTime;   // compute derivative
        Serial.print("Rate error: ");
        Serial.println(rateError);
        Serial.print("Cum error: ");
        Serial.println(cumError);
        double out = kp*error + ki*cumError + kd*rateError;                //PID output               
 
        lastError = error;                                //remember current error
        previousTime = currentTime;                        //remember current time
 
        return out;                                        //have function return the PID output
}


void demoOne()
{
  // this function will run the motors in both directions at a fixed speed
  // turn on motor A
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  // set speed to 200 out of possible range 0~255
  analogWrite(enA, 200);
  // turn on motor B
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  // set speed to 200 out of possible range 0~255
  analogWrite(enB, 200);
  delay(2000);
  // now change motor directions
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);  
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH); 
  delay(2000);
  // now turn off motors
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);  
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}
void demoTwo()
{
  // this function will run the motors across the range of possible speeds
  // note that maximum speed is determined by the motor itself and the operating voltage
  // the PWM values sent by analogWrite() are fractions of the maximum speed possible 
  // by your hardware
  // turn on motors
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);  
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH); 
  // accelerate from zero to maximum speed
  for (int i = 0; i < 256; i++)
  {
    analogWrite(enA, i);
    analogWrite(enB, i);
    delay(20);
  } 
  // decelerate from maximum speed to zero
  for (int i = 255; i >= 0; --i)
  {
    analogWrite(enA, i);
    analogWrite(enB, i);
    delay(20);
  } 
  // now turn off motors
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);  
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);  
}

void feedbck()
{
  // Reading temperature or humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  float h = dht.readHumidity();
  // Read temperature as Celsius (the default)
  float t = dht.readTemperature();
  // Read temperature as Fahrenheit (isFahrenheit = true)
  float f = dht.readTemperature(true);
  // Compute heat index in Celsius (isFahreheit = false)
  float hic = dht.computeHeatIndex(t, h, false);

  Serial.print("T = ");
  Serial.print(t);
  Serial.print(" HR = ");
  Serial.print(h);
  Serial.println(" OVER");
  output = computePID(h);
  delay(500);
  Serial.print(" Motor output = ");
  Serial.println(output);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  analogWrite(enB, output);
 
  }
void loop()
{
  //demoOne();
  //delay(1000);
  //demoTwo();
  //delay(1000);
  feedbck();
}


