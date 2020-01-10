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

// This function is automatically called after each main loop() iteration
void serialEventRun(void) {
  if (Serial.available()) serialEvent();
}

void serialEvent() {
  // Read until serial buffer is empty
  while (Serial.available()) {
    char readChar = Serial.read();

    // If end of line, signal it
    if (readChar == '\r') {
      serial_EOS = true;
      break;
    }
    else {
      serial_buffer[serial_buffer_i] = readChar;
      serial_buffer_i++;
      serial_buffer[serial_buffer_i] = '\0'; // Keep the array NULL-terminated
    }
    if(serial_buffer_i>=19) {
      //reset string if buffer full
      serial_buffer_i=0;
    }
  }
}

void serial_handler() {
  if (serial_EOS==true){
    //if a complete command has been read on the serial port
    char tmp[20];
    serial_EOS=false; //reset EOS value
    serial_buffer_i=0; //reset buffer pointer

    if (strncmp(serial_buffer,"QJack",5) == 0) {
      //Checks whether Power Jack is connected.
      Serial.println(digitalRead(JackMonitor_pin));
    }

    else if (strncmp(serial_buffer,"SPowJack",8) == 0){
      //set power Jack mode. 1: Power is provided through power Jack 0: power is provided through 10-pin headers (Multi-channel or touchscreen + battery)
      strcpy(tmp,&serial_buffer[8]);  //skip the "SPowJack" command
      PowerJack=byte(atoi(tmp));
      EEPROM.update(EE_PowerJack,PowerJack);
      Serial.println(PowerJack);
    }

    else if (strncmp(serial_buffer,"QPowJack",8) == 0){
      //Query power Jack mode. 1: Power is provided through power Jack 0: power is provided through 10-pin headers (Multi-channel or touchscreen + battery)
      Serial.println(PowerJack);
    }

    else if (strncmp(serial_buffer,"SI2C",4) == 0){
      //Set I2C address of the board (need to reset board)

      strcpy(tmp,&serial_buffer[4]);  //skip the "SI2C" command
      I2C_addr=byte(atoi(tmp));
      EEPROM.update(EE_I2C_addr,I2C_addr); //write value in memory only if changed
      Serial.println(I2C_addr);
    }

    else if (strncmp(serial_buffer,"QI2C",4) == 0){
      //Read I2C address of the board
      Serial.println(I2C_addr);
    }

    else if (strncmp(serial_buffer,"SVmax",5) == 0){
      //Set Max Voltage of the board
      strcpy(tmp,&serial_buffer[5]);  //skip the "SVmax" command
      Vmax=word(atoi(tmp));
      EEPROM.update(EE_Vmax,lowByte(Vmax)); //write value in memory only if changed
      EEPROM.update(EE_Vmax+1,highByte(Vmax)); //write value in memory only if changed
      Serial.println(Vmax);
    }

    else if (strncmp(serial_buffer,"QVmax",5) == 0){
      //Query Max voltage of the board
      Serial.println(Vmax);
    }

    else if (strncmp(serial_buffer,"Conf",4) == 0) {
      //initial configuration of the board to default values (only if board is one of the std ratings: 5000V, 2000V, 3000V, 1200V, 500V). Required parameter: Vmax (board V rating)
      double Ki,Kd,Kp;
      strcpy(tmp,&serial_buffer[4]);  //skip the "Conf" command
      Vmax=word(atoi(tmp));
      EEPROM.put(EE_Vmax,Vmax); //write value in memory only if changed
      Serial.println(Vmax);

      if (Vmax==5000) {
        C1=1.0503;
        Kp=0.23;
        Ki=2.2;
        Kd=0.004;
      }
      else if (Vmax==3000){
        C1=1.0356;
        Kp=0.36;
        Ki=2.9;
        Kd=0.006;
      }
      else if (Vmax==2000){
        C1=1.0442;
        Kp=0.35;
        Ki=5;
        Kd=0.004;
      }
      else if (Vmax==1200){
        C1=1.0376;
        Kp=0.51;
        Ki=8;
        Kd=0.006;
      }
      else if (Vmax==500){
        C1=1.0333;
        Kp=0.2;
        Ki=8;
        Kd=0.006;
      }
      Ki_Save=Ki;
      Kd_Save=Kd;
      HVPS_PID.SetTunings(Kp,Ki,Kd);
      C0=0;
      C2=0;
      Vset=0;
      SwSrc=0;
      SwMode=1;
      VMode=0;
      F=1;
      strcpy(Name,"MyHVPS");
      EEPROM.put(EE_Kp,Kp);
      EEPROM.put(EE_Ki,Ki_Save);
      EEPROM.put(EE_Kd,Kd_Save);
      EEPROM.put(EE_C1,C1);
      EEPROM.put(EE_C0,C0);
      EEPROM.put(EE_C2,C2);
      update_EEPROM_string(EE_Name,sizeof(Name),Name);
      EEPROM.put(EE_Vset,Vset); //save voltage setpoint
      EEPROM.update(EE_SwSrc,SwSrc); //save Switching source
      EEPROM.update(EE_SwMode,SwMode);   //save Switching mode
      EEPROM.update(EE_VMode,VMode);   //save voltage control mode
      EEPROM.put(EE_F,F);  //save the Frequency
    }

    else if (strncmp(serial_buffer,"SVset",5) == 0){
      //Set voltage setpoint
      strcpy(tmp,&serial_buffer[5]);  //skip the "SVset" command
      Vset=constrain(word(atoi(tmp)),0,Vmax);
      Serial.println(Vset);
    }
    else if (strncmp(serial_buffer,"QVset",5) == 0){
      //Query voltage setpoint
      // Serial.print("QVset: ");
      Serial.println(Vset);
    }

    else if (strncmp(serial_buffer,"QVnow",5) == 0){
      //Query voltage setpoint
      // Serial.print("QVnow: ");
      Serial.println(Vnow);
    }

    else if (strncmp(serial_buffer,"SPWM",4) == 0){
      //Set voltage setpoint as PWM signal (for open loop mainly)
      strcpy(tmp,&serial_buffer[4]);  //skip the "SPWM" command
      Vset=PWM_to_V(constrain(atoi(tmp),0,1023));
      Serial.println(V_to_PWM(Vset));
    }

    else if (strncmp(serial_buffer,"QPWM",4) == 0){
      //Query voltage setpoint as PWM signal
      Serial.println(V_to_PWM(Vset));
    }

    else if (strncmp(serial_buffer,"SSwSrc",6) == 0){
      // Set the Switching source
      strcpy(tmp,&serial_buffer[6]);  //skip the "SSwSrc" command
      SwSrc=byte(atoi(tmp));
      if (SwSrc==0 && SwMode==2) //if switching with internal frequency source
      TIMSK1 = (1 << OCIE1A); //timer is switched on
      else
      TIMSK1=0; //timer switched off
      if (SwSrc==1) //if external frequency control
      attachInterrupt(digitalPinToInterrupt(Int_ExtFCtrl_pin), Interrupt_ExtFCtrl, CHANGE);
      else
      detachInterrupt(digitalPinToInterrupt(Int_ExtFCtrl_pin));
      Serial.println(SwSrc); //0: onboard (standalone) 1: external (from Master) 2: push button
    }

    else if (strncmp(serial_buffer,"QSwSrc",6) == 0){
      //Query switching source
      Serial.println(SwSrc); //0: onboard (standalone) 1: external (from Master) 2: push button
    }

    else if (strncmp(serial_buffer,"SSwMode",7) == 0){
      // Set the Switching mode
      SwMode_save=SwMode; //save current mode to check if there is a change from 3 to something else (or something else to 3)
      strcpy(tmp,&serial_buffer[7]);  //skip the "SSwMode" command
      SwMode=byte(atoi(tmp));
      if (SwSrc==0)  //if onboard frequency control
      {
        if (SwMode==2 || SwMode==3) //if switching mode or Waveform mode, the timer is switched on
        {
          TCNT1 = 0; //reset counter to 0 so that the cycle starts from the beginning
          Freq_div_counter=0; //reset the software prescaler counter
          TIFR1 = (1 << OCF1A); //clear interrupt flags that may not yet have been treated
          TIMSK1 = (1 << OCIE1A); //Start counter
          if (SwMode==3) //Waveform mode
          {
            digitalWrite(ObFCtrl_pin,HIGH); //optocoupler on
            SwState=true;
          }
          else //is in switching mode
          {
            if (StrobeMode) //if the strobe light is turned on (either mode 1: constant position, or mode 2: scanning strobe pulse)
            {
              TIMSK1 |= (1 << OCIE1B);
              TIMSK1 |= (1 << OCIE1C);
              if (StrobeMode==2) //swiping pulse mode -> timer 3 is turned on
              TIMSK3 |= (1 << OCIE3A);
              else //fixed trigger (pulse) position ->timer 3 is turned off
              TIMSK3=0;
            }
          }
        }
        else
        {
          TIMSK1=0; //timer switched off
          digitalWrite(Strobe_pin,LOW);
          if (SwMode==0) //box off
          {
            digitalWrite(ObFCtrl_pin,LOW);
            SwState=false;
          }
          else  //box in DC mode
          {
            digitalWrite(ObFCtrl_pin,HIGH);
            SwState=true;
          }
        }
      }
      if ((SwMode_save==3)&&(SwMode!=3)) //if previous mode was waveform, but new mode is not
      {
        F=F/nb_Wave_pts; //divide interrupt frequency by number of point of waveform to have frequency of signal and not frequency of point
        F=constrain(F,0.001,5000);
        set_Freq_div(F);
        set_prescaler(F);
        OCR1A=F_to_OCR(F); //set output compare registers A and B
      }
      else if ((SwMode_save!=3)&&(SwMode==3)) //previous mode is not waveform, but we now go in waveform mode
      {
        F=F*nb_Wave_pts; //multiply interrupt frequency by number of point of waveform to have 1 interrupt per point
        F=constrain(F,0.001,5000);
        set_Freq_div(F);
        set_prescaler(F);
        OCR1A=F_to_OCR(F); //set output compare registers A and B
      }
      Serial.println(SwMode); //0-off, 1-DC, 2-Switching, 3-Waveform
    }

    else if (strncmp(serial_buffer,"QSwMode",7) == 0) // Set the Switching source
    Serial.println(SwMode); //0-off, 1-DC, 2-Switching

    else if (strncmp(serial_buffer,"SVMode",6) == 0) // Set the Voltage control source
    {
      strcpy(tmp,&serial_buffer[6]);  //skip the "SVMode" command
      VMode=byte(atoi(tmp));
      if (VMode==0) //internal regulator
      HVPS_PID.SetMode(AUTOMATIC);
      else //open loop or external control
      HVPS_PID.SetMode(MANUAL);
      Serial.println(VMode);
    }

    else if (strncmp(serial_buffer,"QVMode",6) == 0) // Query the Voltage control source
    Serial.println(VMode);

    else if (strncmp(serial_buffer,"SLatchMode",10) == 0) // Set the Voltage control source
    {
      strcpy(tmp,&serial_buffer[10]);  //skip the "SVMode" command
      LatchMode=byte(atoi(tmp));
      Serial.println(LatchMode);
    }
    else if (strncmp(serial_buffer,"QLatchMode",10) == 0) // Query the Voltage control source
    Serial.println(LatchMode);

    else if (strncmp(serial_buffer,"SF",2) == 0) //set the Frequency (only useful in onboard frequency source)
    {
      strcpy(tmp,&serial_buffer[2]);  //skip the "SF" command
      F=atof(tmp);

      if(SwMode==3) //if in waveform mode
      F=F*nb_Wave_pts;  //need one interrupt per waveform point, so multiply signal frequency by the number of point in the waveform.

      F=constrain(F,0.001,5000);
      set_Freq_div(F);
      set_prescaler(F);
      OCR1A=F_to_OCR(F); //set output compare registers A and B
      set_strobe_pulse(); //sets OCR1B and OCR1C to generate the strobe pulse
      F=OCR_to_F(OCR1A); //to take quantification into account

      if(SwMode==3)  //if in waveform mode
      Serial.println(F/nb_Wave_pts,4);
      else
      Serial.println(F,4);
    }
    else if (strncmp(serial_buffer,"QF",2) == 0) //Query the Frequency
    {
      if(SwMode==3)  //if in waveform mode
      Serial.println(F/nb_Wave_pts,4);
      else
      Serial.println(F,4);
    }
    else if (strncmp(serial_buffer,"SC0",3) == 0) //Set calibration factor C0
    {
      strcpy(tmp,&serial_buffer[3]);  //skip the "SCal" command
      C0=atof(tmp);
      EEPROM.put(EE_C0,C0);
      Serial.println(C0,4);
    }
    else if (strncmp(serial_buffer,"SC1",3) == 0) //Set calibration factor C1
    {
      strcpy(tmp,&serial_buffer[3]);  //skip the "SCal" command
      C1=atof(tmp);
      EEPROM.put(EE_C1,C1);
      Serial.println(C1,4);
    }
    else if (strncmp(serial_buffer,"SC2",3) == 0) //Set calibration factor C1
    {
      strcpy(tmp,&serial_buffer[3]);  //skip the "SCal" command
      C2=atof(tmp);
      EEPROM.put(EE_C2,C2);
      Serial.println(C2,4);
    }
    else if (strncmp(serial_buffer,"QC0",3) == 0) //Query calibration C0
    Serial.println(C0,4);
    else if (strncmp(serial_buffer,"QC1",3) == 0) //Query calibration C1
    Serial.println(C1,4);
    else if (strncmp(serial_buffer,"QC2",3) == 0) //Query calibration C2
    Serial.println(C2,4);

    else if (strncmp(serial_buffer,"SCycle",6) == 0) //set the number of cycles the box should make when in Switching mode (0=unlimited number of cycles)
    {
      strcpy(tmp,&serial_buffer[6]);  //skip the "SCycles" command
      Cycle_max=word(atoi(tmp));
      Cycle_counter=0; //reset the counter.
      Serial.println(Cycle_max);
    }

    else if (strncmp(serial_buffer,"QCycle",6) == 0) //Query the cycle number, and the total number of cycle should be made (0=illimited)
    {
      Serial.print(Cycle_counter);
      Serial.print("/");
      Serial.println(Cycle_max);
    }

    else if (strncmp(serial_buffer,"SStPos",6) == 0) //set strobe pulse position (0-255 -> 0-100% of period)
    {
      strcpy(tmp,&serial_buffer[6]);  //skip the "SStPos" command
      Strobe_position=byte(atoi(tmp));
      set_strobe_pulse(); //sets the strobe pulse OCR values
      Serial.println(Strobe_position);
    }
    else if (strncmp(serial_buffer,"QStPos",6) == 0) //Query strobe pulse position (0-255 -> 0-100% of period)
    {
      Serial.println(Strobe_position);
    }
    else if (strncmp(serial_buffer,"SStDur",6) == 0) //set Strobe pulse duration (0-255 -> 0-100% of period)
    {
      strcpy(tmp,&serial_buffer[6]);  //skip the "SStDur" command
      Strobe_duration=byte(atoi(tmp));
      set_strobe_pulse(); //sets the strobe pulse OCR values
      Serial.println(Strobe_duration);
    }
    else if (strncmp(serial_buffer,"QStDur",6) == 0) //Query Strobe pulse duration (0-255 -> 0-100% of period)
    {
      Serial.println(Strobe_duration);
    }
    else if (strncmp(serial_buffer,"SStSw",5) == 0) //set Strobe sweep time (only used in mode 2) time in ms in over which to sweep the pulse on a full period. 17-65535.
    {
      word sweep_time;
      strcpy(tmp,&serial_buffer[5]);  //skip the "SStDur" command
      sweep_time=constrain(atoi(tmp),17,65535);
      Strobe_speed=sweep_time/16.32;
      OCR3A=Strobe_speed;
      Serial.println(sweep_time);
    }
    else if (strncmp(serial_buffer,"SStMode",7) == 0) //Set strobe mode: 0->off, 1->at a fixed location, 2->slow sweeping
    {
      strcpy(tmp,&serial_buffer[7]);  //skip the "SStMode" command
      StrobeMode=byte(atoi(tmp));
      if (StrobeMode && SwMode==2) //if the strobe light is turned on (either mode 1: constant position, or mode 2: scanning strobe pulse) and the HVPS is in Switching mode
      {
        TIMSK1 |= (1 << OCIE1B);
        TIMSK1 |= (1 << OCIE1C);
        if (StrobeMode==2) //swiping pulse mode -> timer 3 is turned on
        TIMSK3 |= (1 << OCIE3A);
        else //fixed trigger (pulse) position ->timer 3 is turned off
        TIMSK3=0;
      }
      else //is strobe light is turned off or HVPS not in Switching mode
      {
        TIMSK1 &= ~(1 << OCIE1B); //stop the interrupt to generate the pulse
        TIMSK1 &= ~(1 << OCIE1C);
        digitalWrite(Strobe_pin,LOW);
        TIMSK3=0; //stop pulse sweeping
      }
      Serial.println(StrobeMode);
    }
    else if (strncmp(serial_buffer,"QStMode",7) == 0) //Query strobe mode: 0->off, 1->at a fixed location, 2->slow sweeping
    {
      Serial.println(StrobeMode);
    }
    else if (strncmp(serial_buffer,"QStDat",6) == 0)
    {
      Serial.println(OCR1A);
      Serial.println(Freq_div);
      Serial.println(OCR1B);
      Serial.println(Strobe_Freq_div_counter_B);
      Serial.println(OCR1C);
      Serial.println(Strobe_Freq_div_counter_C);
    }
    else if (strncmp(serial_buffer,"SKp",3) == 0)
    {
      double Kp,Ki,Kd;

      strcpy(tmp,&serial_buffer[3]);  //skip the "SCycles" command
      Kp=atof(tmp);
      //Ki=HVPS_PID.GetKi();
      //Kd=HVPS_PID.GetKd();
      HVPS_PID.SetTunings(Kp,Ki,Kd);
      EEPROM.put(EE_Kp,Kp);
      Serial.print(Kp);
      Serial.print(",");
      Serial.print(Ki_Save);
      Serial.print(",");
      Serial.println(Kd_Save);
    }

    else if (strncmp(serial_buffer,"SKi",3) == 0)
    {
      double Kp,Ki,Kd;

      strcpy(tmp,&serial_buffer[3]);  //skip the "SCycles" command
      Ki=atof(tmp);
      Ki_Save=Ki;
      Kp=HVPS_PID.GetKp();
      //Kd=HVPS_PID.GetKd();
      HVPS_PID.SetTunings(Kp,Ki,Kd);
      EEPROM.put(EE_Ki,Ki_Save);
      Serial.print(Kp);
      Serial.print(",");
      Serial.print(Ki_Save);
      Serial.print(",");
      Serial.println(Kd_Save);
    }

    else if (strncmp(serial_buffer,"SKd",3) == 0)
    {
      double Kp,Ki,Kd;

      strcpy(tmp,&serial_buffer[3]);  //skip the "SCycles" command
      Kd=atof(tmp);
      Kd_Save=Kd;
      //Ki=HVPS_PID.GetKi();
      Kp=HVPS_PID.GetKp();
      HVPS_PID.SetTunings(Kp,Ki,Kd);
      EEPROM.put(EE_Kd,Kd_Save);
      Serial.print(Kp,3);
      Serial.print(",");
      Serial.print(Ki_Save,3);
      Serial.print(",");
      Serial.println(Kd_Save,4);
    }

    else if ((strncmp(serial_buffer,"QKp",3) == 0)||(strncmp(serial_buffer,"QKi",3) == 0)||(strncmp(serial_buffer,"QKd",3) == 0))
    {
      Serial.print(HVPS_PID.GetKp(),3);
      Serial.print(",");
      Serial.print(Ki_Save,3);
      Serial.print(",");
      Serial.println(Kd_Save,4);
    }

    else if (strncmp(serial_buffer,"SP",2) == 0)
    {
      strcpy(tmp,&serial_buffer[2]);  //skip the "SP" command
      if (tmp[0]=='X' || tmp[1]=='X') //PX (or P X) is used to reset the waveform input
      {
        if (SwMode==3) //if in waveform mode, sets the frequency back to its normal value
        {
          F=F/nb_Wave_pts; //divide interrupt frequency by number of point of waveform to have frequency of signal and not frequency of point
          F=constrain(F,0.001,5000);
          set_Freq_div(F);
          set_prescaler(F);
          OCR1A=F_to_OCR(F); //set output compare registers A
        }
        nb_Wave_pts=0; //reset number of points to 0, so system ready for new waveform input.
        Serial.println(0);
      }
      else
      {
        Wave_pts[nb_Wave_pts]=atoi(tmp); //add one point to the array
        Serial.print(nb_Wave_pts+1); //we return the number of points saved, and not the index of the array
        Serial.print(",");
        Serial.println(Wave_pts[nb_Wave_pts]); //returns index,value
        if(nb_Wave_pts<255) //if number of points smaller than 255, increment the pointer
        nb_Wave_pts++;
        if (SwMode==3) //if in waveform mode
        {
          //adjust frequency according to nb pts (not very elegant, but current implementation of the P function doesn't provide a way to know if a point will be the last one or not, so we need to recalculate the frequency each time a point is added to the array
          if (nb_Wave_pts>1)  //if only 1 point, the frequency remains unchanged
          {
            F=F*nb_Wave_pts/(nb_Wave_pts-1); //correct the frequency of timer to have 1 interrupt per point of the waveform
            F=constrain(F,0.001,5000);
            set_Freq_div(F);
            set_prescaler(F);
            OCR1A=F_to_OCR(F); //set output compare registers A according to value of F
          }
        }
      }
    }
    else if (strncmp(serial_buffer,"QP",2) == 0) //QPtot to know how many points are saved in the waveform array. QPx (with0<=x<nb_Wave_pts-1) to have the setpoint value of index x
    {
      strcpy(tmp,&serial_buffer[2]);  //skip the "QP" command
      if (tmp[0]=='t' || tmp[1]=='t') //QPtot (or QP tot) is used to query the total number of points loaded
      Serial.println(nb_Wave_pts);  //returns the number of points
      else
      {
        int index=atoi(tmp);
        index=constrain(index,0,nb_Wave_pts-1);
        Serial.println(Wave_pts[index]); //returns the queried point
      }
    }

    else if (strncmp(serial_buffer,"SName",5) == 0) // Set HVPS name
    {
      strncpy(Name,&serial_buffer[6],sizeof(Name));  //skip the "SName" command and the space after (this is important here as we don't convert to a number to the space would stay in the name
      update_EEPROM_string(EE_Name,sizeof(Name),Name);
      Serial.println(Name);
    }

    else if (strncmp(serial_buffer,"QName",5) == 0) // Query HVPS name
    {
      Serial.println(Name);
    }

    else if (strncmp(serial_buffer,"Save",4) == 0)
    {
      EEPROM.put(EE_Vset,Vset); //save voltage setpoint
      EEPROM.update(EE_SwSrc,SwSrc); //save Switching source
      EEPROM.update(EE_SwMode,SwMode);   //save Switching mode
      EEPROM.update(EE_VMode,VMode);   //save voltage control mode
      EEPROM.put(EE_F,F);  //save the Frequency
      EEPROM.update(EE_LatchMode,byte(LatchMode)); //save latch mode
      EEPROM.put(EE_Cycle,Cycle_max); //save the number of cycles
      EEPROM.update(EE_nb_Wave_pts,nb_Wave_pts);
      for (byte i=0; i<nb_Wave_pts; i++)
      EEPROM.update(EE_Wave_pts+i,Wave_pts[i]);
      Serial.println("1"); //returns 1, so that every command returns something. No response therefore means: command not understood.
    }

    else if (strncmp(serial_buffer,"QVer",3) == 0) //Query version info
    {
      char version_string[10]="slave ";
      char version_string2[4];

      itoa(Ver,version_string2,10); //version is limited to 999 to hold on 3 digits
      strcat(version_string,version_string2);
      Serial.println(version_string);
    }

    else if (strncmp(serial_buffer,"Debug",5) == 0)
    {
      debug=!debug;
      Serial.println(debug);
    }

    else if (strncmp(serial_buffer,"QMem",4) == 0)//Query content of EEPROM memory
    {//Format: Voltage,frequency,Switching source,Switching mode,Voltage Mode,Regulator gain p, Regulator gain i, Regulator gain d ,Voltage cal. factor C0, Voltage cal. factor C1,Voltage cal. factor C2 ,number of cycles, Button Latching mode
      byte SWMode_mem=EEPROM.read(EE_SwMode);
      Serial.print(word(EEPROM.read(EE_Vset+1),EEPROM.read(EE_Vset)),DEC);
      Serial.print(",");
      if(SWMode_mem==3)//waveform mode is in memory the frequency is the frequency between point in the waveform, so the frequency of the signal if F/number of points in the waveform
      Serial.print(read_EEPROM_double(EE_F)/nb_Wave_pts,4);
      else
      Serial.print(read_EEPROM_double(EE_F),4);
      Serial.print(",");
      Serial.print(EEPROM.read(EE_SwSrc),DEC);
      Serial.print(",");
      Serial.print(SWMode_mem,DEC);
      Serial.print(",");
      Serial.print(EEPROM.read(EE_VMode),DEC);
      Serial.print(",");
      Serial.print(read_EEPROM_double(EE_Kp),3);
      Serial.print(",");
      Serial.print(read_EEPROM_double(EE_Ki),3);
      Serial.print(",");
      Serial.print(read_EEPROM_double(EE_Kd),4);
      Serial.print(",");
      Serial.print(read_EEPROM_double(EE_C0),4);
      Serial.print(",");
      Serial.print(read_EEPROM_double(EE_C1),4);
      Serial.print(",");
      Serial.print(read_EEPROM_double(EE_C2),4);
      Serial.print(",");
      Serial.print(word(EEPROM.read(EE_Cycle+1),EEPROM.read(EE_Cycle)),DEC);
      Serial.print(",");
      Serial.println((EEPROM.read(EE_LatchMode)));
    }

    else if (strncmp(serial_buffer,"QRelState",9) == 0) // Query Relays state
    {
      Rel_printState();
    }

    else if (strncmp(serial_buffer,"QRelMode",8) == 0) // Query relay mode
    {
      Serial.print("RelMode: ");
      if (RelMode == RelMode_auto_disconnect_retry){
        Serial.print(RelMode);
        Serial.print(": ");
        Serial.print(RelRetryPeriod_s);
        Serial.println("[s]");
      }
      else{
        Serial.println(RelMode);
      }
    }

    else if (strncmp(serial_buffer,"SRelOff",7) == 0) // Syntax: SRelOff n. n=-1 => all relays off
    {
      RelMode = RelMode_manual;
      RelAutoState = RelAutoStateOff;
      strcpy(tmp,&serial_buffer[7]);  //skip the "SRelOff" command
      char *endptr=NULL;
      int n=strtol(tmp, &endptr, 10); //get relays which should be turned off
      if (tmp != endptr){
        Rel_setRel(n, false);
        Rel_printState();
      }
      else{
        //no digits found
        Rel_allOff();
        Rel_printState();
      }
    }

    else if (strncmp(serial_buffer,"SRelOn",6) == 0) // Syntax: SRelOn n. n=-1 => all relays off
    {
      RelMode = RelMode_manual;
      RelAutoState = RelAutoStateOff;
      strcpy(tmp,&serial_buffer[6]);  //skip the "SRelOn" command
      char *endptr=NULL;
      int n=strtol(tmp, &endptr, 10);  //get relays which should be turned off
      if (tmp != endptr){
        Rel_setRel(n, true);
        Rel_printState();
      }
      else{
        //no digits found
        Rel_allOn();
        Rel_printState();
      }
    }
    //
    // else if (strncmp(serial_buffer,"SRelManual",10) == 0) // Syntax: SRelManual
    // {
    //   RelMode = RelMode_manual;
    //   RelAutoState = R
    //   Rel_auto_off();
    //   Serial.print("SRelManual ");
    //   Serial.println(RelMode);
    // }

    else if (strncmp(serial_buffer,"SRelAuto",8) == 0) // Syntax: SRelAuto n. If n== -1 => not recheck, if n>0, n?delay [s] betwwen rechecks
    {
      RelAutoState = RelAutoReset;
      // Rel_allOn(); //activate all relays
      // Rel_printState();
      // RelAuto_searching_short = false; // reset flags
      // RelAuto_shortcuircuit_trigger_raised_ms = 0; // reset timer

      strcpy(tmp,&serial_buffer[8]);  //skip the "SRelAuto" command
      int n=atoi(tmp); // get delay
      if (n <= 0){ //no recheck
        n=0;
        RelMode = RelMode_auto_disconnect_once;
      }
      else{
        RelMode = RelMode_auto_disconnect_retry;
        RelRetryPeriod_s = n;
        RelRetry_last_ms = millis();
      }
      Serial.print("SRelAuto: ");
      Serial.print(RelMode);
      Serial.print(": ");
      Serial.print(n);
      Serial.println("[s]");

      RelAutoState == RelAutoReset;
    }


    else if (strncmp(serial_buffer,"QR",2) == 0) //QRx (with0<=x<nb_Wave_pts-1) to have the value of the waveform voltage read by the HVPS at index x
    {                                            //Use QPtot first to know how many points need to be read. Then use QPx to now the setpoint values and QRx for the actual value. Returned value is on 8 bit mapped from 0 to 100% Vmax
      strcpy(tmp,&serial_buffer[2]);  //skip the "QR" command
      int index=atoi(tmp);
      index=constrain(index,0,nb_Wave_pts-1);
      Serial.println(Wave_pts_mes[index]); //returns the queried point
    }

    else{
      Serial.println("Err"); //returns "Err" to say that command wasn't understood
    }
  }
}
