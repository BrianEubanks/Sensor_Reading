// Brian Eubanks
// Checkpoint and Rollback Mechanism



//Enable Checkpoints
#define CHKPT

//Fault Injection Macros
//#define INSERT_FAULT

//#define ZERO_FAULT
#define ONES_FAULT
//#define FLIP_FAULT

#define ADDR_FAULT *address3

#define FLIP_VALUE 0x01

//Sample Counter
//Shows the Sample Count on the Serial Monitor
//#define Show_Sample_Count
#define MAXSampleCount 100




//Funciton Definitions

// Checkpoint stores GP registers, Sensor Value, and Time Stamp into a save slot.
void checkpoint();

// Recover GP registers, Sensor Value, and Time Stamp from a save slot.
void recovery();

// Insert a fault into a GP Register before Recovery
void malicious();

// Sets the Checkpoint flag to create a checkpoint
// Called from timer interrupt to checkpoint every 5 seconds.
void setCheckpoint();

// Sets the RollBack flag
// Enabled when a rollback is requested from ES or Visualizer
void setRBFlag();

// sensorDataWrite sends the current sensor data and time stamp to the visualizer.
void sensorDataWrite(int sensorData, unsigned long sensorTimeStamp);

// Determines if a checkpoint with the value 'rbValue' is avaliable.
// Returns the SAVE INDEX of the value if it is avaliable. -1 otherwise
int checkpointAvaliable(long rbValue);

// Negotiate Rollback
// negotiates with the visualizer 
void negotiateRollback(long rbValue);





// Variable Definitions
const int visualizerFaultLED = 13;

//Pin for ES to insert fault and request rollback
const int interruptPin = 2;

// Byte to insert in to register to simulate fault
unsigned int badReg = 0; 


// Flags to determine whether  checkpoint, negotiate, or rollback.
volatile bool chkptFlag = false;
volatile bool negotiateFlag = false;
volatile bool rbFlag = false;



// Used to check if the sensor has changed position.
// Used for checkpoint values.
int currSensorValue = 0;
unsigned long currSensorTime = 0;


int negotiateValue;
int restoreIndex = 0;



//MEMORY ADDRESS POINTERS:
//32 GP Registers:
volatile byte *address0 = 0x00;
volatile byte *address1 = 0x01;
volatile byte *address2 = 0x02;
volatile byte *address3 = 0x03;
volatile byte *address4 = 0x04;
volatile byte *address5 = 0x05;
volatile byte *address6 = 0x06;
volatile byte *address7 = 0x07;
volatile byte *address8 = 0x08;
volatile byte *address9 = 0x09;
volatile byte *address10 = 0x0A;
volatile byte *address11 = 0x0B;
volatile byte *address12 = 0x0C;
volatile byte *address13 = 0x0D;
volatile byte *address14 = 0x0E;
volatile byte *address15 = 0x0F;
volatile byte *address16 = 0x10;
volatile byte *address17 = 0x11;
volatile byte *address18 = 0x12;
volatile byte *address19 = 0x13;
volatile byte *address20 = 0x14;
volatile byte *address21 = 0x15;
volatile byte *address22 = 0x16;
volatile byte *address23 = 0x17;
volatile byte *address24 = 0x18;
volatile byte *address25 = 0x19;
volatile byte *address26 = 0x1A;
volatile byte *address27 = 0x1B;
volatile byte *address28 = 0x1C;
volatile byte *address29 = 0x1D;
volatile byte *address30 = 0x1E;
volatile byte *address31 = 0x1F;



//SAVE SLOTS
//SAVE_SIZE determines number of backup slots
const int SAVE_SIZE=5;
int saveCount = 0;

//struct to save GPRegs
struct GPRegBackup{
  //SAVED MEMORY VALUES
  volatile byte adr00;
  volatile byte adr01;
  volatile byte adr02;
  volatile byte adr03;
  volatile byte adr04;
  volatile byte adr05;
  volatile byte adr06;
  volatile byte adr07;
  volatile byte adr08;
  volatile byte adr09;
  volatile byte adr0A;
  volatile byte adr0B;
  volatile byte adr0C;
  volatile byte adr0D;
  volatile byte adr0E;
  volatile byte adr0F;
  volatile byte adr10;
  volatile byte adr11;
  volatile byte adr12;
  volatile byte adr13;
  volatile byte adr14;
  volatile byte adr15;
  volatile byte adr16;
  volatile byte adr17;
  volatile byte adr18;
  volatile byte adr19;
  volatile byte adr1A;
  volatile byte adr1B;
  volatile byte adr1C;
  volatile byte adr1D;
  volatile byte adr1E;
  volatile byte adr1F;

};
struct GPRegBackup regSaves[SAVE_SIZE];

//Store timer Values
unsigned long timerBackup[SAVE_SIZE];
//Store sensor Values
int sensorBackup[SAVE_SIZE];
//Index to access backup slots
int backupIndex = 0;





void setCheckpoint(){
  if (!negotiateFlag){
    chkptFlag=true;
  }
}

void setRBFlag(){
  rbFlag=true;
}


/* Checkpoint Function */
void checkpoint() {
  
  //At this routine you should save all register somewhere in memory for later recovery 
  //Store Memory at address[i] to memoryAddress[i]
  
  cli();
  
  //Backup Counter(Sensor) Value
  sensorBackup[backupIndex] = currSensorValue;
  //Backup Timer Value - Used for rollback negotiation
  
  timerBackup[backupIndex] = currSensorTime;
  
  regSaves[backupIndex].adr00 = *address0;
  regSaves[backupIndex].adr01 = *address1;
  regSaves[backupIndex].adr02 = *address2;
  regSaves[backupIndex].adr03 = *address3;
  regSaves[backupIndex].adr04 = *address4;
  regSaves[backupIndex].adr05 = *address5;
  regSaves[backupIndex].adr06 = *address6;
  regSaves[backupIndex].adr07 = *address7;
  regSaves[backupIndex].adr08 = *address8;
  regSaves[backupIndex].adr09 = *address9;
  regSaves[backupIndex].adr0A = *address10;
  regSaves[backupIndex].adr0B = *address11;
  regSaves[backupIndex].adr0C = *address12;
  regSaves[backupIndex].adr0D = *address13;
  regSaves[backupIndex].adr0E = *address14;
  regSaves[backupIndex].adr0F = *address15;
  regSaves[backupIndex].adr10 = *address16;
  regSaves[backupIndex].adr11 = *address17;
  regSaves[backupIndex].adr12 = *address18;
  regSaves[backupIndex].adr13 = *address19;
  regSaves[backupIndex].adr14 = *address20;
  regSaves[backupIndex].adr15 = *address21;
  regSaves[backupIndex].adr16 = *address22;
  regSaves[backupIndex].adr17 = *address23;
  regSaves[backupIndex].adr18 = *address24;
  regSaves[backupIndex].adr19 = *address25;
  regSaves[backupIndex].adr1A = *address26;
  regSaves[backupIndex].adr1B = *address27;
  regSaves[backupIndex].adr1C = *address28;
  regSaves[backupIndex].adr1D = *address29;
  regSaves[backupIndex].adr1E = *address30;
  regSaves[backupIndex].adr1F = *address31;

  if (backupIndex == SAVE_SIZE-1){
    backupIndex = 0;
  }
  else{
    backupIndex++;
  }

 
  if (saveCount < SAVE_SIZE){
     saveCount++;
  }

  
  Serial.println("!");
  sei();
  return;
}





/* Recovery Function */
void recovery() {

  Serial.println("*");
  //cli();


  /* 
   *  This is a function that I will write it later.
   *  I will not tell you what I will be doing in this function, it is going to 
   *  destroy some registers of the processor.
   *  The idea is that the checkpoint/recovery should be able to cancel this function out.
   */
  //malicious(); 



  /*
   * Implement the actual recovery here: you should restor all registers 
   * from what you saved in the checkpoint funciton. 
   */

  // Decrement Backup Index
  // Use Most Recent Backup
  /*
  if (backupIndex == 0){
    backupIndex = SAVE_SIZE - 1;
  }
  else{
    backupIndex --;
  }
  */
  //assign backupIndex to restoreIndex
  //restore Index is the index negotiated by the ES and visualizer
  backupIndex = restoreIndex;

  //Restore Sensor Value
  currSensorValue = sensorBackup[backupIndex];
  
  //Restore Timer Value - Timer may just be used for negotiation. Not needed to restore
  currSensorTime = timerBackup[backupIndex];

  //Restore GP Regs
  *address0 = regSaves[backupIndex].adr00;
  *address1 = regSaves[backupIndex].adr01;
  *address2 = regSaves[backupIndex].adr02;
  *address3 = regSaves[backupIndex].adr03;
  *address4 = regSaves[backupIndex].adr04;
  *address5 = regSaves[backupIndex].adr05;
  *address6 = regSaves[backupIndex].adr06;
  *address7 = regSaves[backupIndex].adr07;
  *address8 = regSaves[backupIndex].adr08;
  *address9 = regSaves[backupIndex].adr09;
  *address10 = regSaves[backupIndex].adr0A;
  *address11 = regSaves[backupIndex].adr0B;
  *address12 = regSaves[backupIndex].adr0C;
  *address13 = regSaves[backupIndex].adr0D;
  *address14 = regSaves[backupIndex].adr0E;
  *address15 = regSaves[backupIndex].adr0F;
  *address16 = regSaves[backupIndex].adr10;

  *address17 = regSaves[backupIndex].adr11;
  *address18 = regSaves[backupIndex].adr12;
  *address19 = regSaves[backupIndex].adr13;
  *address20 = regSaves[backupIndex].adr14;
  *address21 = regSaves[backupIndex].adr15;
  *address22 = regSaves[backupIndex].adr16;
  *address23 = regSaves[backupIndex].adr17;
  *address24 = regSaves[backupIndex].adr18;
  *address25 = regSaves[backupIndex].adr19;
  *address26 = regSaves[backupIndex].adr1A;
  *address27 = regSaves[backupIndex].adr1B;
  *address28 = regSaves[backupIndex].adr1C;
  *address29 = regSaves[backupIndex].adr1D;
  *address30 = regSaves[backupIndex].adr1E;
  *address31 = regSaves[backupIndex].adr1F;


  
  //Serial.print("Recovery done at the time:");
  //Serial.println(micros());
  //sei();    
  return;
  }    



/* Malicious Function */
void malicious(void){
  
  //I will do some malicious things here to destroy some of your registers. 
  //Your code should stand it. 

  

  #ifdef ZERO_FAULT
  ADDR_FAULT = 0x00;
  #endif

  #ifdef ONES_FAULT
  ADDR_FAULT = 0xFF;
  #endif

  #ifdef FLIP_FAULT
  ADDR_FAULT ^= FLIP_VALUE;
  #endif



}


/*  Sensor Data Write
 *   - Check if sensorData is different than currentSensorValue
 *   - If different, save sensorData value with timestamp and send it to the Visualizer
 *   - Else Do nothing
 */

void sensorDataWrite(int sensorData, unsigned long sensorTimeStamp){
  cli();
  if (sensorData != currSensorValue){
    currSensorValue = sensorData;
    currSensorTime = sensorTimeStamp;
    
    //Send New Sensor Data
    String dataString = String(sensorData); 
    dataString += "-";
    dataString += String(sensorTimeStamp);
    Serial.println(dataString);
    
  } 
  sei();
}


/*  Checkpoint Avaliable
 *  Checks if given checkpoint is avaliable
 * 
 */
 
int checkpointAvaliable(long rbValue){

  // 1 second margin of error
  // +/- 0.5 seconds.

  // .4 second margin
  // +/-0.2 max seconds
  long timestampMargin = 200;
  long timestampMax = 0;
  long timestampMin = 0;
  
  for (int i = 0; i < saveCount; i++){


    //Equality check only
    /*
    if (timerBackup[i] == rbValue){
      return i;
    }
    */

    timestampMax = timerBackup[i] + timestampMargin;
    timestampMin = timerBackup[i] - timestampMargin;
    
    
    //Check if rbValue timestamp is within timerstampMargin of error.
    
    if (rbValue <= timestampMax && rbValue >= timestampMin)
      return i;

  }
  return -1;
}


//Negotiate Visualizer Starts
void negotiateRollback(long rbValue){
//Negotiation
    sei();
    boolean rbAgree = false;
    bool response = false;
    int suggestIndex = 0;
    //Serial.println('Y');
    while (!rbAgree){
      restoreIndex = checkpointAvaliable(rbValue);
      if(restoreIndex >= 0){
        rbAgree=true;
        Serial.println('Y');
        //memInterface_Restore(restoreIndex);
        //Do Recovery
        rbFlag = true;
      }
      else{
        //No, Suggest new rollback value
        Serial.println('N');
        //Serial.println(sensorBackup[suggestIndex]);   //Use timerBackup instead. Sends Timestamp
        Serial.println(timerBackup[suggestIndex]);
        rbValue=0;
        //Wait for response
        //delay(1000);
        //while(!Serial.available()){};
        bool response = false;
        char c = Serial.read();
        while(c != 'N'){
          digitalWrite(visualizerFaultLED, HIGH);
          //delay(500);
          c = Serial.read();
          if(c=='Y'){
            rbAgree=true;
            restoreIndex=suggestIndex;
            //Serial.println("Yes");
            //delay(500);
            c='N';
            digitalWrite(visualizerFaultLED, LOW);
          }
        }
        //Serial.println("Done");
        if (!rbAgree){
          //Serial.println("No");
          rbValue=-1;
          response = false;
          while (!response){
            //Serial.print(":[");
            delay(500);
            rbValue=Serial.parseInt();
            if (rbValue > 0){
              response = true;
            }
          }
        }
        suggestIndex++;
        suggestIndex %= saveCount;
      }
    }
    //memInterface_Restore(suggestIndex);
    //Do Recovery
    rbFlag = true;
}

//Negotiate - Arduino starts
void negotiateRollbackES(long rbValue){
//Negotiation
    sei();
    boolean rbAgree = false;
    bool response = false;
    int suggestIndex = 0;
    Serial.println('?');
    while (!rbAgree){
        //Suggest  rollback value
        //Serial.println('?');
        //Serial.println(sensorBackup[suggestIndex]); //Use Timer instead. Sends timestamp
        Serial.println(timerBackup[suggestIndex]);
        rbValue=0;
        //Wait for response
        //delay(1000);
        //while(!Serial.available()){};
        bool response = false;
        char c = Serial.read();
        while(c != 'N'){
          delay(500);
          c = Serial.read();
          //Serial.print(c);
          if(c=='Y'){
            rbAgree=true;
            //Serial.println("Yes");
            restoreIndex = suggestIndex;
            delay(500);
            c='N';
          }
        }
        //Serial.println("Done");
        if (!rbAgree){
          //Serial.println("No");
          rbValue=-1;
          response = false;
          while (!response){
            //Serial.print(":[");
            delay(500);
            rbValue=Serial.parseInt();
            if (rbValue > 0){
              response = true;
              restoreIndex = checkpointAvaliable(rbValue);
              if(restoreIndex >= 0){
                rbAgree=true;
                Serial.println('Y');
              }
              else{
                Serial.println('N');
              }
            }
          }
        }
        suggestIndex++;
        suggestIndex %= saveCount;
    }
    //memInterface_Restore(suggestIndex);
    //Do Recovery
    rbFlag = true;
}



void insertFault(){
  //malicious fault
  if(digitalRead(interruptPin) == LOW){
    negotiateFlag = true;
    chkptFlag=false;
    rbFlag = false;
  }
  #ifdef INSERT_FAULT
  malicious();
  #endif
  //begin recovery
  //negotiateRollbackES(0);
}


void showBackups(){
  Serial.println("Backups");
  Serial.print("Most Recent Backup: ");
  Serial.println(backupIndex);
  for(int i = 0; i < saveCount; i++){
    Serial.print("Backup:");
    Serial.println(i);
    Serial.print("Sensor Value: ");
    Serial.println(sensorBackup[i]);
    Serial.print("Timer Value: ");
    Serial.println(timerBackup[i]);
    Serial.println(regSaves[i].adr00);
    Serial.println(regSaves[i].adr01);
    Serial.println(regSaves[i].adr02);
    Serial.println(regSaves[i].adr03);
    Serial.println(regSaves[i].adr04);
    Serial.println(regSaves[i].adr05);
    Serial.println(regSaves[i].adr06);
    Serial.println(regSaves[i].adr07);
    Serial.println(regSaves[i].adr08);
    Serial.println(regSaves[i].adr09);
    Serial.println(regSaves[i].adr0A);
    Serial.println(regSaves[i].adr0B);
    Serial.println(regSaves[i].adr0C);
    Serial.println(regSaves[i].adr0D);
    Serial.println(regSaves[i].adr0E);
    Serial.println(regSaves[i].adr0F);
    Serial.println(regSaves[i].adr10);
    Serial.println(regSaves[i].adr11);
    Serial.println(regSaves[i].adr12);
    Serial.println(regSaves[i].adr13);
    Serial.println(regSaves[i].adr14);
    Serial.println(regSaves[i].adr15);
    Serial.println(regSaves[i].adr16);
    Serial.println(regSaves[i].adr17);
    Serial.println(regSaves[i].adr18);
    Serial.println(regSaves[i].adr19);
    Serial.println(regSaves[i].adr1A);
    Serial.println(regSaves[i].adr1B);
    Serial.println(regSaves[i].adr1C);
    Serial.println(regSaves[i].adr1D);
    Serial.println(regSaves[i].adr1E);
    Serial.println(regSaves[i].adr1F);
  }
}

void showTimestampBackups(){
  Serial.println("TimeStamps Backups");
  Serial.print("Most Recent Backup: ");
  Serial.println(backupIndex);
  for(int i = 0; i < saveCount; i++){
    Serial.print("Backup:");
    Serial.println(i);
    Serial.print("Timer Value: ");
    Serial.println(timerBackup[i]);

  }
}
