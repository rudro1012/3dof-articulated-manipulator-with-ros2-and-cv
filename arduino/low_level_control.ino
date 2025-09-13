#include<stdio.h>
#include<Servo.h>

// declaring servo motors
Servo servoJoint1;
Servo servoJoint2;
Servo servoJoint3;
Servo servoJoint4;

// declaring servo pins
int servoPinJoint1=3;
int servoPinJoint2=5;
int servoPinJoint3=6;
int servoPinJoint4=9;

// initial position of each servo motor
int initialPosJoint1=90;
int initialPosJoint2=90;
int initialPosJoint3=90;
int initialPosJoint4=90;

// equivalent of 0,180 in terms of microsecond write
int minPulse=500;
int maxPulse=2500;

String inputString="";  // incoming serial data buffer
bool stringComplete=false;  // serial data status 
int commaCounter;  // counting comma in the complete string buffer

void setup() {
  Serial.begin(9600);

  // servo attachment to the arduino pins
  servoJoint1.attach(servoPinJoint1);
  servoJoint2.attach(servoPinJoint2);
  servoJoint3.attach(servoPinJoint3);
  servoJoint4.attach(servoPinJoint4);

  // pinMode(servoPinJoint1,OUTPUT);
  // pinMode(servoPinJoint2,OUTPUT);
  // pinMode(servoPinJoint3,OUTPUT);
  // pinMode(servoPinJoint4,OUTPUT);

  // intial position of all the servos
  servoJoint1.write(initialPosJoint1);
  servoJoint2.write(initialPosJoint2);
  servoJoint3.write(initialPosJoint3);
  servoJoint4.write(initialPosJoint4);

}

// generating  microseconds pulse for servos
int pulseGenerator(float angle){
  int pulse=minPulse + (angle/180)*(maxPulse-minPulse);
  return pulse;
}

void loop() {
  if (stringComplete){  //to check if the input string is complete

  float joint1,joint2,joint3,endEffJoint;
  int pulseJoint1,pulseJoint2,pulseJoint3,pulseJoint4,pulseJoint;
  int firstComma,secondComma,thirdComma;
    if(commaCounter==3){  // checks the comma count to determine how many servos are need to rotate
      
      firstComma=inputString.indexOf(',');  //locate the index of first comma in the compete input string
      secondComma=inputString.indexOf(',', firstComma+1);  //loacte the index of comma that appears after the first comma
      thirdComma=inputString.indexOf(',',secondComma+1);  //locate the next comma after the second comma

      if(firstComma>0 && secondComma>firstComma && thirdComma>secondComma){
        // checks if the index of expected commas are valid  
        
        // converting jostrings from input string, that are separated by commas
        joint1=inputString.substring(0,firstComma).toFloat();  
        joint2=inputString.substring(firstComma+1,secondComma).toFloat();
        joint3=inputString.substring(secondComma+1,thirdComma).toFloat();
        endEffJoint=inputString.substring(thirdComma+1).toFloat();


        Serial.print("joint1: ");Serial.println(joint1);
        Serial.print("joint2: ");Serial.println(joint2);
        Serial.print("joint3: ");Serial.println(joint3);
        Serial.print("endEff: ");Serial.println(endEffJoint);

        // converting float angles to integer pulse
        pulseJoint1=pulseGenerator(joint1);
        pulseJoint2=pulseGenerator(joint2);
        pulseJoint3=pulseGenerator(joint3);
        pulseJoint4=pulseGenerator(endEffJoint);

        // servo actuation
        servoJoint1.writeMicroseconds(pulseJoint1);
        servoJoint2.writeMicroseconds(pulseJoint2);
        servoJoint3.writeMicroseconds(pulseJoint3);
        servoJoint4.writeMicroseconds(pulseJoint4);
        
        Serial.println("ACK");  //printing acknowledgement for trajectory generator to know that arduino has executed what it was asked for and send next
        inputString="";
        stringComplete=false;
      }
    }

    else if(commaCounter==2){
      firstComma=inputString.indexOf(',');
      secondComma=inputString.indexOf(',',firstComma+1);
      if(firstComma>0 && secondComma>firstComma ){

        joint1=inputString.substring(0,firstComma).toFloat();
        joint2=inputString.substring(firstComma+1,secondComma).toFloat();
        joint3=inputString.substring(secondComma+1).toFloat();

        Serial.print("joint1: ");Serial.println(joint1);
        Serial.print("joint2: ");Serial.println(joint2);
        Serial.print("joint3: ");Serial.println(joint3);

        pulseJoint1=pulseGenerator(joint1);
        pulseJoint2=pulseGenerator(joint2);
        pulseJoint3=pulseGenerator(joint3);

        servoJoint1.writeMicroseconds(pulseJoint1);
        servoJoint2.writeMicroseconds(pulseJoint2);
        servoJoint3.writeMicroseconds(pulseJoint3);

        Serial.println("ACK");
        
      }
      inputString="";
      stringComplete=false;
    }
    else if(commaCounter<2){    //only for end effector actuation when no other servos are required to move

      float angle=inputString.toFloat();
      pulseJoint=pulseGenerator(angle);
      servoJoint4.writeMicroseconds(pulseJoint);
      Serial.println("ACK");
      inputString="";
      stringComplete=false;  
    }
    
  }
  //reseting the inputString and stringComplete so new joints string can appear in serial buffer
  


}


void serialEvent(){  //calling serialevent to check on serial data buffer
  while(Serial.available()){
    char inChar=(char)Serial.read();  //reading each charecter of the buffer
    if(inChar=='\n'){  //checking if '\n' appears in the buffer

      /* if any \n appears then it means that one set of joangles is complete in the buffer and 
      to execute the trajectory the string status must be changed to true */
      stringComplete=true;
      commaCounter=0;  
      for(int i=0;i<inputString.length();i++){  //checking how many commas are in the set of joangles
        char comma=inputString[i];
        if(comma==','){
          commaCounter++;
        }
      
      }
    }
    else{
      inputString=inputString + inChar;  //if inChar is not '\n' then add the next charecter until '\n' appears
    }
    
  }
}

