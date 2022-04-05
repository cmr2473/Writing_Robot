#include "QuadDecoder.h"
#include <Arduino.h>
#include "classes.h"
#include <stream.h>



robot driver;
IntervalTimer velTimer;
int telemCount=0;
void timercall(){
    //Serial.print("timer call:  ");
    driver.calcVel();
    
    if(telemCount%30 == 1){
        driver.printTelemetry();
    }

    
    telemCount++;
}

char stringData[16];
void setup()
{
    
    

    // Start Serial Monitor
    Serial.begin(9600);
    delay(100);
    //while(!Serial){
        delay(100);
    //}
    //Serial.println("ready");
    // Timer for velocity

    driver.begin();
    //Serial.println("driver began");
    Serial5.begin(9600); // Teensy Serial Port 1 - Use Pins 20 and 21 for Rx and Tx respectfully. Connect to Xbee. For this Device, Rx->Rx and Tx->Tx (even though this is not the convention)
    
    velTimer.begin(timercall, PERIOD);
    //Serial.println("timer began");
    driver.s.attach(SERVO_PIN);
    delay(500);
    driver.penUp();
    delay(1000);
}
String buffer;
void loop()
{
  /*  for(int i = 0; i < 16; i++){
        stringData[i] = 0;
    }
    
    while(Serial5.available()==0); //Stays here untill it recieves a value

    char letter = Serial5.read(); //reads value from other Xbee
    Serial.println(letter);
    switch(letter){ // implement inputing a page width/height, string to write, newLine, linefollow while no other key pressed
        case 'a': //input page width and height
            while(Serial5.available()==0);
            delay(2000);
            buffer = Serial5.readStringUntil('\n',3);
            driver.pageWidth = (double)buffer.toInt();

            while(Serial5.available()==0);
            delay(2000);
            buffer = Serial5.readStringUntil('\n',3);
            driver.pageHeight = (double)buffer.toInt();

            while(Serial5.available()==0);
            delay(2000);
            buffer = Serial5.readStringUntil('\n',5);
            driver.fontSize = (double)buffer.toFloat();

            driver.startWrite();
            break;
        case 'b': //write a string
            Serial5.clear();
            while(Serial5.available()==0);
            //delay(4000);
            buffer = Serial5.readStringUntil('\n',16);
            for(int i = 0; i < 16 ; i++){
                stringData[i] = buffer.charAt(i);
                delay(10);
            }
            delay(10);
            //Serial.println(stringData);
            //Serial.println(buffer);
            driver.writeString(stringData,16);
            
            break;
        case 'c': //newline
            driver.newline();
            driver.penDown();
            delay(100);
            driver.penUp();
            break;
        case 'd' ://linefollow
            while(Serial5.available()==0){
                driver.lineFollow();
            }
            buffer = Serial5.readString();
            break;
        
    }
    driver.idle(); 
    
          */

         //driver.quickLine(0,10);

    driver.lineFollow();
    
    //driver.goAround();
    //delay(1000);
         
          //driver.writeChar('c');
    //delay(20000);
    /*
    delay(2000);

    driver.arc(0,M_PI/2, RIGHT);
    delay(2000);
    driver.arc(0,-M_PI/2, LEFT);
    delay(2000);
    driver.arc(0,0, RIGHT);
    delay(2000);
    driver.arc(0,M_PI, LEFT);
    delay(1000);
    driver.penUp();
    delay(1000);
    driver.penDown();
    delay(1000);*/
    //driver.locate(25,-30,-M_PI/2);
    driver.writeChar('A');
    delay(2000);
    driver.writeChar('B');
    delay(2000);
    driver.writeChar('C');
    delay(2000);
    driver.writeChar('D');
    delay(2000);
    
    
    driver.writeChar('e');
    delay(2000);
    driver.writeChar('f');
    delay(2000);
    driver.writeChar('g');
    delay(2000);
    driver.writeChar('h');
    delay(2000);
    driver.writeChar('i');
    delay(20000);
    /*
    driver.quickLine(10,0);
    driver.quickLine(0,10);
    driver.quickLine(-10,0);
    driver.quickLine(0,-10);

   delay(10000);

*/
   
}


