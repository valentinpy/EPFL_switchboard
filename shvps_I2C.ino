/*Code for the slave boards (i.e. Single channel high voltage power supplies, SHVPS) of the Project Peta-pico-Voltron
Target platform: Arduino micro

Copyright 2016-2017 Samuel Rosset
Distributed under the terms of the GNU General Public License GNU GPLv3

This file is part of shvps.

    shvps is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    shvps is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with shvps.  If not, see  <http://www.gnu.org/licenses/>

*/
void receiveEvent(int nbBytes) //this is called when the master sends information on the bus. It decodes the command sent by the master
{
  uint8_t paramL,paramH;
  union
  {
    byte b[4];
    double d;
  }data;
  
  if (Wire.available())
  {
    I2C_cmd=Wire.read(); //first byte is the command
    if(Wire.available()==2) //a word parameter follows the command
    {
      paramL=Wire.read();
      paramH=Wire.read();
      I2C_param_w=word(paramH,paramL);
    }
    else if (Wire.available()==4)
    {
      for (byte i=0; i<4; i++)
        data.b[i]=Wire.read();
      I2C_param_d=data.d;
    }
    while(Wire.available()>0); //flushes extra byte (there shouldn't be any);
  }
}

//Here is the handling of all the communication between the master and slave boards through the I2C bus

void requestEvent() //this is called when the master requests information. The returned info depends on what command the master has sent previously
{
  uint8_t buffer[2];
  if (I2C_cmd==I2C_QVmax) //Master requests Maximum voltage value of the board
  {
    buffer[0]=lowByte(Vmax);
    buffer[1]=highByte(Vmax);
    Wire.write(buffer,2);
  }
  else if (I2C_cmd==I2C_SVset) //Master wants to set voltage value
  {
    Vset=I2C_param_w;
    buffer[0]=lowByte(Vset);
    buffer[1]=highByte(Vset);
    Wire.write(buffer,2);
  }
  else if (I2C_cmd==I2C_QVset) //Master requests voltage setpoint
  {
    buffer[0]=lowByte(Vset);
    buffer[1]=highByte(Vset);
    Wire.write(buffer,2);
  }
  else if (I2C_cmd==I2C_QVnow) //Master requests current output voltage
  {
    buffer[0]=lowByte(Vnow);
    buffer[1]=highByte(Vnow);
    Wire.write(buffer,2);
  }
  else if (I2C_cmd==I2C_QVer) //Master requests board firmware version
  {
    word firmware_version=Ver; //not likely to be a word, but makes things simpler if every response from board is on 2 bytes
    buffer[0]=lowByte(firmware_version);
    buffer[1]=highByte(firmware_version);
    Wire.write(buffer,2);
    VMode=0;  //!!!Master will ask version on initialisation. So if we get in that bit of code, it means that the board is used in a multi-board setup, and therfore voltage control must be set on internal regultation
    SwMode=2; //same idea: switching mode is set to switching: either it doesn't matter because Master controls switching via ExtF pin, or each slave board generates its switching.
    TIMSK1 = (1 << OCIE1A); //Start counter
  }
  else if (I2C_cmd==I2C_SSwSrc) //set the switching source.
  {
    SwSrc=I2C_param_w;
    buffer[0]=SwSrc;
    buffer[1]=0;
    Wire.write(buffer,2);
    if (SwSrc==0 && SwMode==2) //if switching with internal frequency source
        TIMSK1 = (1 << OCIE1A); //timer is switched on
    else
      TIMSK1=0; //timer switched off
    if (SwSrc==1) //if external frequency control
        attachInterrupt(digitalPinToInterrupt(Int_ExtFCtrl_pin), Interrupt_ExtFCtrl, CHANGE);
    else
      detachInterrupt(digitalPinToInterrupt(Int_ExtFCtrl_pin));
  }
  else if (I2C_cmd==I2C_QName) //Asks the name of the board
  {
    byte l=strlen(Name)+1;
    Wire.write(Name,l);
  }
  else if (I2C_cmd==I2C_SF) //set the switching source.
  {
    union
    {
      byte b[4];
      double d;
    }data;
    
    F=I2C_param_d;
    F=constrain(F,0.001,5000);
    set_Freq_div(F);
    set_prescaler(F);
    OCR1A=F_to_OCR(F); //set output compare registers A and B
    F=OCR_to_F(OCR1A); //to take quantification into account
    data.d=F;
    Wire.write(data.b,4);
  }
  
  else
    Wire.write(I2C_addr); //In case no specific commands are given, replies with its address. Can be used to check that the board is up and running, or for an address scan to find the boards that are connected.
  
  I2C_cmd=0xFF; //clears the command buffer. Waits for a new one
}
