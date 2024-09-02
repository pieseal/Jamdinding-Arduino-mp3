/*
Copyright (c) 2010, Donal Morrissey
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions
are met:

    * Redistributions of source code must retain the above copyright 
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in
      the documentation and/or other materials provided with the distribution.
    * Neither the name of the Author nor the names of its 
      contributors may be used to endorse or promote products derived
      from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

* Sketch for testing sleep mode with wake up on pin2 interrupt.

*/
#include <EEPROM.h>
#include <avr/sleep.h>
#include <avr/power.h>

#include "SoftwareSerial.h"
#include "DFRobotDFPlayerMini.h"

//Inicia a serial por software nos pinos 10 e 11
SoftwareSerial mySoftwareSerial(10, 11); // RX, TX

DFRobotDFPlayerMini myDFPlayer;

char command='p';
//int pausa = 0;

byte pin2 = 2;
int vol;
byte hour;
/***************************************************
 *  Name:        pin2Interrupt
 *
 *  Returns:     Nothing.
 *
 *  Parameters:  None.
 *
 *  Description: Service routine for pin2 interrupt  
 *
 ***************************************************/
void pin2Interrupt(void)
{
  /* This will bring us back from sleep. */
  
  /* We detach the interrupt to stop it from 
   * continuously firing while the interrupt pin
   * is low.
   */
  detachInterrupt(0);
}


/***************************************************
 *  Name:        enterSleep
 *
 *  Returns:     Nothing.
 *
 *  Parameters:  None.
 *
 *  Description: Enters the arduino into sleep mode.
 *
 ***************************************************/
void enterSleep(void)
{
  
  /* Setup pin2 as an interrupt and attach handler. */
  attachInterrupt(0, pin2Interrupt, LOW);
  delay(100);
  
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  
  sleep_enable();
  //Serial.println("sleep...");
  sleep_mode();
  
  /* The program will continue from here. */
  
  /* First thing to do is disable sleep. */
  sleep_disable();
  //asm volatile (" jmp 0"); 
}


/***************************************************
 *  Name:        setup
 *
 *  Returns:     Nothing.
 *
 *  Parameters:  None.
 *
 *  Description: Setup for the Arduino.           
 *
 ***************************************************/
uint8_t play1[10] = { 0X7E, 0xFF, 0x06, 0X03, 00, 00, 0x01, 0xFE, 0xF7, 0XEF};
int fcnt=0;

void setup()
{
  pinMode(3, OUTPUT);
  digitalWrite(3, HIGH);

  //Comunicacao serial com o modulo
  mySoftwareSerial.begin(9600);
  //Inicializa a serial do Arduino
  Serial.begin(115200);
  
  /* Setup the pin direction. */

  pinMode(pin2, INPUT_PULLUP);
  pinMode(4, INPUT);
  
  Serial.println("Initialisation complete.");
  Serial.println();
  Serial.println(F("DFRobot DFPlayer Mini"));
  Serial.println(F("Initializing DFPlayer module ... Wait!"));
  int seconds=0;
  myDFPlayer.begin(mySoftwareSerial);
  //delay(4000);
  //myDFPlayer.play(1);
  do{
    fcnt=myDFPlayer.readFileCounts();
    Serial.println(fcnt);
    delay(201);
  } while(fcnt<1);

  //delay(1100);
  Serial.print(F("DFPlayer Mini module initialized!\n\rCurrent Volume: "));
  vol=EEPROM.read(1);
  if(vol>30)vol=20;
  Serial.println(vol);
  Serial.print(F("DFPlayer Mini module initialized!\n\rCurrent Hour: "));
  hour=EEPROM.read(0)+1;
  if(hour>12)hour=1;
  Serial.println(hour);
  Serial.print("Folder Count:");
  Serial.println(fcnt);
  delay(100);
  //Serial.println(myDFPlayer.readFolderCounts());
  //mySoftwareSerial.write( play1[0] );
  //delay(2000);
  myDFPlayer.volume(vol); //Volume 5
  myDFPlayer.play(hour);
  delay(2100);
  
  menu_opcoes();
}



/***************************************************
 *  Name:        loop
 *
 *  Returns:     Nothing.
 *
 *  Parameters:  None.
 *
 *  Description: Main application loop.
 *
 ***************************************************/
void loop()
{
  //mySoftwareSerial.write( play1[0] );
  //delay(2000);
  int seconds=0;
  command=0;
      while(Serial.available()==0)
      {
        if(digitalRead(2)==LOW)
        {
          hour++;
          //hour=EEPROM.read(0)+1;
          if(hour>12)hour=1;
          myDFPlayer.play(hour);
          delay(2100);
          Serial.print("Current Hour:");
          Serial.println(hour);
          EEPROM.write(0, hour);
          break;
        }
        int st=digitalRead(4);//myDFPlayer.readType();
        if(st==1)
        {
          //st=myDFPlayer.readType() ;//== DFPlayerPlayFinished
          //Serial.print("readType:");
          seconds++;
          Serial.print(seconds);
      //vol=myDFPlayer.readVolume();
          delay(500);
          //Serial.println(seconds);
          if (seconds>=3){
            seconds=0;
            command='S';
            break;
          }
        }
      }
    //Serial.println(Serial.available());
    if ((command!='S')or(Serial.available()!=0))
    {
      command = Serial.read();
      Serial.print("command: ");
      Serial.println(command);
    }
    if ((command >= '0') && (command <= ';'))
    {
      hour = command - 48;
      if (hour==0)hour=12;
      Serial.print("Music reproduction: ");
      Serial.println(hour);
      myDFPlayer.play(play1[hour]);
      EEPROM.write(0,hour);
      //myDFPlayer.start();
      //menu_opcoes();
    }
   
    //Reproduction
    
    //Stop
    if (command == 's')
    {
      myDFPlayer.stop();
      Serial.println("Music Stopped!");
      menu_opcoes();
    }
    
    //Pausa/Continua a musica
    if (command == 'p')
    {
        myDFPlayer.start();
    }
    
    if (command == '+')
    {
      myDFPlayer.volumeUp();
      delay(200);
      vol++;
      if(vol>300)vol=30;
      Serial.print("Current volume:");
      Serial.println(vol);
      EEPROM.update(1,vol);
      //menu_opcoes();
    }
     if (command == '<')
    {
      myDFPlayer.previous(); 
      delay(200);
      Serial.println("Previous:");
      Serial.print("Current track:");
      Serial.println(myDFPlayer.readCurrentFileNumber()-1); 
      //menu_opcoes();
    }
     if (command == '>')
    {
      hour++;
      if(hour>12)hour=1;
      myDFPlayer.play(hour);
      Serial.print("Current Hour:");
      Serial.println(hour);
      EEPROM.write(0, hour);
    }
    
    //Decreases volume
    if (command == '-')
    {
      myDFPlayer.volumeDown();
      Serial.print("Current Volume:");
      vol--;
      if(vol<0)vol=0;
      Serial.println(vol);
      EEPROM.update(1,vol);
      //menu_opcoes();
    }
    if(command =='S')
    {
      Serial.println("Entering sleep ...");
      digitalWrite(3,LOW);
      digitalWrite(10,LOW);
      delay(200);
      enterSleep();
      digitalWrite(3,HIGH);
      Serial.print("Back to Wakeup...\n\rCurrent Volume: ");
      Serial.println(vol);
      while(myDFPlayer.readFileCounts()<1)delay(201);
      delay(100);
      myDFPlayer.volume(vol);
      hour++;
      if(hour>12)hour=1;
      myDFPlayer.play(hour);
      Serial.print("Current Hour:");
      Serial.println(hour);
      EEPROM.write(0, hour);
      delay(2100);
    }     
}

void menu_opcoes()
{
  Serial.println();
  Serial.println(F("=================================================================================================================================="));
  Serial.println(F("Commands:"));
  Serial.println(F(" [1-3] To select the MP3 file"));
  Serial.println(F(" [s] stopping reproduction"));
  Serial.println(F(" [p] pause/continue music"));
  Serial.println(F(" [+ or -] increases or decreases the volume"));
  Serial.println(F(" [< or >] forwards or backwards the track"));
  Serial.println(F(" [S] Sleep Mode Power Down"));
  Serial.println();
  Serial.println(F("================================================================================================================================="));
}
