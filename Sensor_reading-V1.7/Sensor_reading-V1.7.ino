/*
 * Sensor reading code version 1.0
 * Written by Ahmad Patooghy 7/2020, University of Central Arkansas, apatooghy@uca.edu
 * Use the switches to control the program
 * 1- verbus to get logs at the serial monitor
 * 2- Sampling_Delay to tune the reading frequency
 * 3- Noise_Removal to select either of 3 noise removal algorithms or no noise removal options
 * 4- Samples_per_Average to control the number of reading for each data point for Averaging_Noise_Removal option 
 * 5- history_length to control the length of sensor readings the program stores
 * 6- smoothing_factor to control the adaptation speed of the exponential smoothing filter used in the Exponential_Smothing_Noise_Removal option
 * 7- noise_sensitivity_thresold to control the system's inertia to sudden movements 
 */


#include "arduino-timer.h"
#include "rollBackMechanism.h"

#define verbus 0  //1 verbus mode, 0 normal mode
#define SensorPin A0
#define Sampling_Delay 100000  //in microseconds
#define Noise_Removal 0   // 0 -> no noise removal
                          // 1 -> averaging noise removal, this options reduces the actual sampling rate by 10x
                          // 2 -> running average noise removal
                          // 3 -> Low-pass filtering, it works basedo on exponential smothing
#define Samples_per_Average 10
#define history_length 10
#define smoothing_factor  0.8  //should be (0..1)
                               // closer to 1 -> more of the recent reading
                               // closer to 0 -> more of the reading history
#define noise_sensitivity_thresold 100  // can be any positive value
                                        // closer to 0 -> the system has higher inertia and does not let fast/sudden movements
                                        // infinity  -> no inertia
                                        // a normal value based on the sensor is something between 50-150

//const int visualizerFaultLED = 13;
int visualizerFaultLEDState = 0;

int Sensor_readings[history_length]={0,0,0,0,0,0,0,0,0,0};
bool new_reading;


                           
//auto timer = timer_create_default(); // create a timer with default settings
Timer<2, micros> timer;


int SensorReading(){
  return(analogRead(SensorPin));
}

void Sensor_log(int last_reading){
  if (verbus) Serial.print("Sensor_log: ");
  for (int i=0; i<history_length-1; i++)
  {
      Sensor_readings[i] = Sensor_readings[i+1];
      if (verbus)  Serial.print(Sensor_readings[i]);
  }
  if (verbus)  Serial.println(last_reading);
  Sensor_readings[history_length-1] = last_reading;
  return;
}
    
//#define Noise_Removal 0
bool No_Noise_Removal(void *){
  int Raw_Measurement = 0;
  if (new_reading){
    Serial.println("Sampling frequency is not supported!!");
    return false;
  }
  Raw_Measurement = SensorReading();
  if (verbus)  Serial.print("No_Noise_Removal: ");
  if (verbus)  Serial.println(Raw_Measurement);
  Sensor_log(SensorReading());
  new_reading = true;
  return true;
}

//#define Noise_Removal 1
bool Averaging_Noise_Removal(void *){
  int Raw_Measurement = 0;
  int temp;
  if (new_reading){
    Serial.println("Sampling frequency is not supported!!");
    return false;
  }
  if (verbus)  Serial.print("Averaging_Noise_Removal sensor reading: ");  
  for (int i=0; i< Samples_per_Average; i++){
      temp = SensorReading();
      Raw_Measurement += temp;
      if (verbus)  Serial.print(temp);
      delayMicroseconds(Sampling_Delay/1000); //?
  }
  Raw_Measurement /= Samples_per_Average;
  if (verbus)  Serial.println("");
  if (verbus)  Serial.print("Averaging_Noise_Removal average: ");
  if (verbus)  Serial.println(Raw_Measurement);
  Sensor_log(Raw_Measurement);
  //Serial.println(Raw_Measurement);
  new_reading = true;
  return true;
}

//#define Noise_Removal 2
bool Running_Average_Noise_Removal(void *){
  int Raw_Measurement = SensorReading();
  if (new_reading){
    Serial.println("Sampling frequency is not supported!!");
    return false;
  }
  if (verbus) Serial.print("Running_Average_Noise_Removal sensor reading: ");
  if (verbus) Serial.println(Raw_Measurement);
  for (int i=0; i<history_length; i++)
      Raw_Measurement += Sensor_readings[i];
  Raw_Measurement /= history_length + 1;
  if (verbus) Serial.print("Running_Average_Noise_Removal running average: ");
  if (verbus) Serial.println(Raw_Measurement);
  Sensor_log(Raw_Measurement);
  new_reading = true;
  return true;
}

//#define Noise_Removal 3
bool Exponential_Smothing_Noise_Removal(void *){
  if (new_reading){
    Serial.println("Sampling frequency is not supported!!");
    return false;
  }
  int temp = SensorReading();
  if (verbus) Serial.print("Exponential_Smothing_Noise_Removal sensor reading: ");
  if (verbus) Serial.println(temp);
  int Raw_Measurement = (int) ((Sensor_readings[history_length-1]* (1-smoothing_factor)) + (temp * smoothing_factor));
  if (verbus) Serial.print("Exponential_Smothing_Noise_Removal with factor ");
  if (verbus) Serial.print(smoothing_factor);
  if (verbus) Serial.print(": ");
  if (verbus) Serial.println(Raw_Measurement);
  if (verbus) Serial.print("difference: ");
  if (verbus) Serial.println(Raw_Measurement-temp);
  Sensor_log(Raw_Measurement);
  //Serial.println(Raw_Measurement);
  new_reading = true;
  return true;
}

bool Is_Sensor_Connected(void){ //checks the standard deviation of readings to decide if the sensor is connected/disconnected
  int total = 0;
  int average = 0;
  if (verbus) Serial.print("Is_Sensor_Connected logs are: ");
  for (int i=0; i<history_length; i++){
      total += Sensor_readings[i];  
      if (verbus)  Serial.print(Sensor_readings[i]);   
  }
  if (verbus) Serial.println("");
  average = total/(history_length);
  if (verbus) Serial.print("Is_Sensor_Connected average is: ");
  if (verbus) Serial.println(average);
  total = 0;
  for (int i=0; i<history_length; i++)
      total += (long) ((Sensor_readings[i]-average)*(Sensor_readings[i]-average));  
  total /= history_length;
  if (verbus) Serial.print("Is_Sensor_Connected std^2 is: ");
  if (verbus) Serial.println(total);
  long std=sqrt(total);
  if (verbus) Serial.print("Is_Sensor_Connected variance is: ");
  if (verbus) Serial.println(std);
  if ((std>=0 ) && (std<noise_sensitivity_thresold))    //as our sensor readings range between 0 to 1000 standard deviation above 100 means random noise 
    return true;  //sensor is connected 
  else
    return false; //sensor is not connected
}


//BE
//Handle Comms from Visualizer
void handleComms(char c){

  if (c=='`'){
    showBackups();
  } 
  // ES request rollback
  if (c=='$'){
    //rbFlag=true;
    negotiateRollbackES(0);
    rbFlag=true;
  }
  // Visualizer Request rollback received
  else if (c=='r'){
    negotiateValue = Serial.parseInt();
    //negotiateValue = Serial.read();
    if (negotiateValue >= 0){
      //digitalWrite(visualizerFaultLED, HIGH);
      //delay(100);
      //digitalWrite(visualizerFaultLED, LOW);
      negotiateRollback(negotiateValue);
      negotiateFlag = false;
      chkptFlag = false;
      rbFlag=true;
    }
  }
}

void setup() {
  // put your setup code here, to run once:
   pinMode(visualizerFaultLED, OUTPUT);
  new_reading = false;
  switch(Noise_Removal){
    case 0:
      timer.every(Sampling_Delay, No_Noise_Removal);
      break;     
    case 1:
      timer.every(Sampling_Delay, Averaging_Noise_Removal);
      break;
    case 2:
      timer.every(Sampling_Delay, Running_Average_Noise_Removal);
      break;
    case 3:
      timer.every(Sampling_Delay, Exponential_Smothing_Noise_Removal);
      break;
  }

  Serial.begin(38400);


  //BE
  //Time Driven Checkpoint
  //How to Handle both heckpoint registers, and checkpoint memory variables.???
  timer.every(1000000, setCheckpoint); // call the rollback function every 1000000 micros (1 second)

  //Manual Driven Recovery
  //Attach interrupt to DigitalPin 2
  //Call recovery everytime button is pressed
  pinMode(interruptPin,INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPin), insertFault, FALLING);
  
  //Recovery will be changed to be called after negotiating with the Visualizer
  
  
}

/*
 * Replace Serial.printlns with calls to sensorDataWrite()
 * This funciton:
 *  -Only backup if the new value is different than the previous version.
 *  
 */


void loop() {
  // put your main code here, to run repeatedly:
  timer.tick(); // tick the timer
  int temp=0;

  
  if ((new_reading) && (Is_Sensor_Connected())) {
    new_reading = false;
    temp = Sensor_readings[history_length-1];
    //Serial.println(temp);
    sensorDataWrite(temp, micros());
  }
  else if ((new_reading) && (!Is_Sensor_Connected())) {
    new_reading = false;
    //Serial.println("-1");
    sensorDataWrite("-1", micros());
  }

  //Check for SerialComms
  else if (Serial.available()){
    int c = Serial.read();
    handleComms(c);
  }

  if(negotiateFlag){
    negotiateFlag=false;
    negotiateRollbackES(0);
    
  }

  if(chkptFlag){
    chkptFlag=false;
    checkpoint();
  }
      //A Rollback or Checkpoint is needed
  if(rbFlag){
    rbFlag=false;
    recovery();
    //delay(500);
  }

    //digitalWrite(visualizerFaultLED, HIGH);
    //delay(50);
    //digitalWrite(visualizerFaultLED, LOW);

}
