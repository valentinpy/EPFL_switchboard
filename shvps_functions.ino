/*Code for the switchboard boards of the Project derivated from Peta-pico-Voltron
  Author: Valentin Py
  E-mail: valentin.py@epfl.ch / valentin.py@gmail.com
  Company: EPFL - LMTS
  Date: 17.12.2019

  source:petapicovoltron.com
  Target platform: Arduino micro + switchboard v1.0

  Copyright 2016-2019 Samuel Rosset - Valentin Py
  Distributed under the terms of the GNU General Public License GNU GPLv3

  This file is part of Switchboard.

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

double OCR_to_F(word OCR) //transforms the value of the Output compare register (16 bit) to a frequency
{
  double Freq; //to store frequency
  Freq=16.0E6/double(prescaler)/double(OCR)/double(Freq_div);
  return (Freq);
}

word F_to_OCR(double Freq) //transforms a frequency value to the corresponding OCR value
{
  word OCR;
  OCR=word(16.0E6/Freq/double(prescaler)/double(Freq_div));
  return OCR;
}

word PWM_to_V(word pwm_value) //transforms a 10 bit PWM value to a voltage, assuming linar relationship
{
  word v_output_value;
  v_output_value=word((double(pwm_value)/1023.0*double(Vmax))+0.5); //+ 0.5 to round to closest value and not to lower value
  return(v_output_value);
}

word V_to_PWM(word V_value) //transforms a voltage setpoint into a 10 bit PWM value, assuming linear relationship
{
  word pwm_output_value;
  pwm_output_value=word((1023.0*double(V_value)/double(Vmax))+0.5);//+ 0.5 to round to closest value and not to lower value
  return(pwm_output_value);
}

word FilteredInputVoltage() //function averages 10 readings of input voltage to filter noise
{
  word input_V=0; //to store the input voltage
  word PWM_out; //to store the 10 bit value corresponding to the input

  for (byte i=0; i<10; i++) //10 times. AnalogRead takes bout 0.1 ms. So it takes 1ms for the average. number of average could be decreased for faster execution
    input_V=input_V+analogRead(ExtVCtrl_pin); //no risk of overflow. analog read is on 10 bits, so the addition of 10 reads will be <2^16-1
  input_V=input_V/10; //average value
  PWM_out=input_V;
  return PWM_out;
}

word Read_Vnow(byte n) //function averages n readings of the emco output to filter noise
{
  word input_V=0; //to store the input voltage
  for (byte i=0; i<n; i++) //n times
    input_V=input_V+analogRead(Vread_pin); //to avoid overflow n should not exceed 64
  input_V=input_V/n; //average value
  return input_V;
}

void setPWMDuty(int val) //Sets the PWM duty cycle on pin 13 (connected to OCR4A) to a 10 bit value
{
  TC4H = val >> 8;
  OCR4A = 0xFF & val;
}

double read_EEPROM_double(word addr) //reads a double from the EEPROM starting at address addr (4bytes)
{
  double x;
  EEPROM.get(addr,x);
  if (isnan(x))  //can happen if EEPROM was never written (i.e. the 4 bytes are 255 255 255 255)
    x=0;
  return x;
}

void read_EEPROM_string(word addr, byte n, char * c) //reads a string from EEPROM at address addr, and with a length of n or less (n includes the termination character \0) and store it in C
{
  if(EEPROM.read(addr)==255)  //If string has not yet been defined in EEPROM
    c[0]='\0';
  else
  {
    for (byte i=0; i<n; i++)
    {
      c[i]=EEPROM.read(addr+i);
      if (c[i]=='\0')
        break;
    }
  }
}

void update_EEPROM_string(word addr, byte n, char * c) //writes string c at address addr, but not more than n bytes
{
  for (byte i=0; i<n; i++)
  {
    EEPROM.update(addr+i,c[i]);
    if(c[i]=='\0')
      break;
  }
}

void set_prescaler(double Freq) //sets the timer 1 prescaler to the optimal value, depending on the Frequency F
{
  if (Freq*Freq_div>245)
  {
    TCCR1B &= ~(1 << CS11); //sets prescaler to 1
    TCCR1B &= ~(1 << CS12); //sets prescaler to 1
    TCCR1B |= (1 << CS10); //sets prescaler to 1
    prescaler=1;
  }
  else if (Freq*Freq_div>30.6)
  {
    TCCR1B &= ~(1 << CS10); //sets prescaler to 8
    TCCR1B &= ~(1 << CS12); //sets prescaler to 8
    TCCR1B |= (1 << CS11); //sets prescaler to 8
    prescaler=8;
  }
  else if (Freq*Freq_div>3.82)
  {
    TCCR1B |= (1 << CS10); //sets prescaler to 64
    TCCR1B |= (1 << CS11); //sets prescaler to 64
    TCCR1B &= ~(1 << CS12); //sets prescaler to 64
    prescaler=64;
  }
  else if (Freq*Freq_div>0.955)
  {
    TCCR1B |= (1 << CS12); //sets prescaler to 256
    TCCR1B &= ~(1 << CS11);//sets prescaler to 256
    TCCR1B &= ~(1 << CS10); //sets prescaler to 256
    prescaler=256;
  }
  else
  {
    TCCR1B |= (1 << CS12); //sets prescaler to 1024
    TCCR1B |= (1 << CS10); //sets prescaler to 1024
    TCCR1B &= ~(1 << CS11);//sets prescaler to 1024
    prescaler=1024;
  }
}

void set_Freq_div(double Freq) //sets the software prescaler at the optimal value for the Frequency F
{
  Freq_div=byte(1+0.238422/Freq/2.0)*2;
}

void set_strobe_pulse() //sets the counter value to generate a trigger (or strobe) pulse) OCR1B is when the trigger pulse goes high. OCR1C is when the trigger pulse goes low
{
  //because of software prescaler (Freq_div), one period of the signal lasts for Freq_div*OCR1A.
  byte Strobe_down_tmp=Strobe_position+Strobe_duration; //value between 0 and 255 that indicates as function of the period when the trigger pulse must be turned off. May over flow (e.g. 250+10=5 -> signal goes down at the beginning of the period)
  unsigned long Totalcount=long (Freq_div)*long(OCR1A);
  unsigned long Strobe_up=(Totalcount*Strobe_position)/255; //at which tick number the signal must go up
  Strobe_Freq_div_counter_B=(Strobe_up/OCR1A)%Freq_div;  //how many times the counter wraps before signal must go up
  OCR1B=Strobe_up%OCR1A; //How many ticks to count after _couter_B wraps before turning output up
  unsigned long Strobe_down=(Totalcount*Strobe_down_tmp)/255; //at which tick number the signal must go down
  Strobe_Freq_div_counter_C=(Strobe_down/OCR1A)%Freq_div;  //how many times the counter wraps before signal must go down
  OCR1C=Strobe_down%OCR1A; //How many ticks to count after _couter_B wraps before turning output down
}

void dynamic_PID() //adapts the coefficient of the PID to the situation. For example turn off d term if error is small to reduce noise
{
  //decide to turn on or off the derivative term depending on the residual error
  if ((abs(100*(input-setpoint))/Vmax)>5 && HVPS_PID.GetKd()==0 ) //if error larger than 5% and derivative term is not turned on
  {
    double Kp,Ki,Kd;
    Kp=HVPS_PID.GetKp();
    Ki=HVPS_PID.GetKi();
    Kd=Kd_Save;
    HVPS_PID.SetTunings(Kp,Ki,Kd); //turns on derivative term
  }
  else if ((abs(100*(input-setpoint))/Vmax)<=5 && HVPS_PID.GetKd()>0) //if the error is now lower than 5% and derivative term is still turned on
  {
    double Kp,Ki;
    Kp=HVPS_PID.GetKp();
    Ki=HVPS_PID.GetKi();
    HVPS_PID.SetTunings(Kp,Ki,0); //turns off derivative term
  }

  //decide to turn on or off the integral term depending the current setpoint and current voltage value
  if (setpoint==0 && 100*input/Vmax<10 && HVPS_PID.GetKi()>0) //if setpoint is 0 and current voltage is lower than 10%Vmax and integral term currently turned on
  {
    double Kp,Kd;
    Kp=HVPS_PID.GetKp();
    Kd=HVPS_PID.GetKd();
    HVPS_PID.SetTunings(Kp,0,Kd); //turns off integral term
  }
  if ((setpoint!=0|| 100*input/Vmax>=10) && HVPS_PID.GetKi()==0) //if the setpoint is higher than 0 or current input higher than 10% Vmax, integral term is not actuvated
  {
    double Kp,Ki,Kd;
    Kp=HVPS_PID.GetKp();
    Kd=HVPS_PID.GetKd();
    Ki=Ki_Save;
    HVPS_PID.SetTunings(Kp,Ki,Kd); //turns on integral term
  }
}

void switching()  //Switching control of the optocouplers (if onboard control, the switching is taken care of by the timer 1 (or is permanently on or off if in DC or off mode repspectively)
{

  if (SwSrc==2) //Switching is controlled with the push button
  {
    if (LatchMode) //if in latching mode, pressing the button toggles HV on/off
    {
      if (digitalRead(Button_pin)) //Push button to reset cycles
      {
        delay(100); //debouncing
        if (digitalRead(Button_pin)) //if button hasn't changed state (debouncing)
        {
          SwState=!SwState; //toggles the latch state
          digitalWrite(ObFCtrl_pin,SwState); //writes new latch state on the pin
          while (digitalRead(Button_pin));   //wait until the button is released.
        }
      }
    }
    else //if not in latching mode, HV is on as long as button is pressed
    {
      boolean ButtonState=digitalRead(Button_pin); //reads button state
      digitalWrite(ObFCtrl_pin,ButtonState); //applies button state to output
      SwState=ButtonState; //update the value of the SwState flag
    }
  }
  else if ((SwMode==2 || SwMode==3) && Cycle_max>0 && Cycle_counter>Cycle_max) //if onboard frequency control with switching mode or waveform mode enabled and counting enabled and counter reached the targeted value
  {
    SwMode_save=SwMode; //saves the current Switching mode to be able to restore it when push button is pressed
    SwMode=0; //switched the box mode to off
    TIMSK1=0; //timer switched off
    digitalWrite(ObFCtrl_pin,LOW); //Optocoupler switched off
    SwState=false;
    Cycle_counter=0; //resets the cycle counter
    if (SwMode_save==3) //if previously in waveform mode, we must set the frequency back to its initial value, as we are now in mode 0
    {
      F=F/nb_Wave_pts; //divide interrupt frequency by number of point of waveform to have frequency of signal and not frequency of point
      F=constrain(F,0.001,5000);
      set_Freq_div(F);
      set_prescaler(F);
      OCR1A=F_to_OCR(F); //set output compare registers A and B
    }
  }

  else if (SwMode==0 && Cycle_max>0) //if the box is set up for limited number of cycles and is currently off, pressing the button restarts a series of pulses
  {
    if (digitalRead(Button_pin)) //Push button to reset cycles
    {
      delay(100); //debouncing
      if (digitalRead(Button_pin)) //if button hasn't changed state (debouncing)
      {
        if (SwMode_save>=2)
          SwMode=SwMode_save; //place board back in the previous mode (should be 2 or 3)
        else //should not happen, but set to switching in case previous mode was neither switching nor waveform
          SwMode=2;
        TCNT1 = 0; //reset counter to 0 so that the cycle starts from the beginning
        Freq_div_counter=0; //reset the software prescaler counter
        TIFR1 = (1 << OCF1A); //clear interrupt flags that may not yet have been treated
        TIMSK1 = (1 << OCIE1A); //Start counter
      }
    }
  }
}

//interrupts
ISR(TIMER1_COMPA_vect) //internal frequency source OCR1A (timer 1 is in CTC mode, so will reset to 0 after this value is reached
{
  //to achieve lower frequencies that we can have with the highest prescaler, we implement an additional software prescaler
  if (Freq_div_counter==0) //beginning of the cycle
  {
    if(SwMode!=3) //any mode but arbitrary waveform
    {
      digitalWrite(ObFCtrl_pin,LOW);
      SwState=false;
      Cycle_counter++; //increments the number of cycles
    }
    else
    {
      Vset_arb=(double(Wave_pts[Wave_pts_counter])/255.0)*double(Vset); //Wave_pts is a 8bit array. so we divide by 255 to have a 0-1 value and we scale it by Vset
      if (Wave_pts_counter==0) //is this is the first point of the cycle, we increment the cycle counter
        Cycle_counter++;
      Wave_pts_counter++; //overflows at 255, but next line takes care of the situation in which <255 points are present in the array.
      Wave_pts_counter=Wave_pts_counter%nb_Wave_pts;
      Wave_meas_flag=true; //time to save the output voltage in the measurement array (this is done in the main loop);
    }
  }
  else if ((Freq_div_counter==Freq_div/2) && SwMode==2) //middle of the cycle & only if in switching mode
  {
    digitalWrite(ObFCtrl_pin,HIGH);
    SwState=true;
  }
  Freq_div_counter++;
  Freq_div_counter=Freq_div_counter%Freq_div;

}

ISR(TIMER1_COMPB_vect)
{
  if (Freq_div_counter==Strobe_Freq_div_counter_B)
  {
    digitalWrite(Strobe_pin,HIGH);
  }
}
ISR(TIMER1_COMPC_vect)
{
  if (Freq_div_counter==Strobe_Freq_div_counter_C)
  {
    digitalWrite(Strobe_pin,LOW);
  }
}
ISR(TIMER3_COMPA_vect)
{
  Strobe_position++;
  set_strobe_pulse();
}

void Interrupt_ExtFCtrl()
{
  if (digitalRead(Int_ExtFCtrl_pin))
  {
    digitalWrite(ObFCtrl_pin,HIGH);
    SwState=true;
  }
  else
  {
    digitalWrite(ObFCtrl_pin,LOW);
    SwState=false;
  }
}
