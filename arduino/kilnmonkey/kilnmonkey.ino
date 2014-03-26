//  Kiln Monkey - Copyright 2012 by Francisco Castro <http://fran.cc>
//
//  This program is free software: you can redistribute it and/or modify
//  it under the terms of the GNU General Public License as published by
//  the Free Software Foundation, either version 3 of the License, or
//  (at your option) any later version.
//
//  This program is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.
//
//  You should have received a copy of the GNU General Public License
//  along with this program.  If not, see <http://www.gnu.org/licenses/>.

// ToDo:
//  * programmable maximum kiln temperature

#define MAXIMUM_KILN_TEMPERATURE 1100   // celsius degrees
#define PID_DEFAULT_KC  80              // 20 to 100
#define PID_DEFAULT_TI  120             // 120 to 600 seconds
#define PID_DEFAULT_TD  30              // 30 to 75 seconds (Ti/4)
#define WINDOW_SIZE 3000                // milliseconds

#define RELAY_PIN   7
#define TC_SO_PIN   12
#define TC_CS_PIN   10
#define TC_CLK_PIN  13

#define MAX_PACKET_LENGTH 640          // bytes

////////////////////////

#include "TinyPacks.h"
#include "TinyPostman.h"
#include "MemoryFree.h"

#include <SoftwareSerial.h>


// PID
#define   MAX_SAMPLES 16
uint32_t  sample_timestamp;
uint8_t   sample_index;
double    temperature_samples[MAX_SAMPLES];
double    temperature;
uint8_t   sample_errors;
uint32_t  now;
uint32_t  window_start;
#include "thermocouple.h"
MAX31855  thermocouple(TC_SO_PIN, TC_CS_PIN, TC_CLK_PIN);

// Profile
bool      position_changed;

// API
uint8_t   pack_buffer[MAX_PACKET_LENGTH];
Postman   postman;
Framer    framer(pack_buffer, MAX_PACKET_LENGTH);

// Debug
uint8_t loop_time;

// Resources
#include "about_resource.h"
About     about;
#include "debug_resource.h"
Debug     debug;
#include "profile_resource.h"
Profile   profile;
#include "pid_resource.h"
PID pid(PID_DEFAULT_KC, PID_DEFAULT_TI, PID_DEFAULT_TD, WINDOW_SIZE/1000);

// Bluetooth
bool            wireless_serial;
SoftwareSerial  Bluetooth(A0, A1);

void setup()
{
  temperature = 0;
  sample_index = 0;
  sample_errors = 0;
  sample_timestamp = millis();
  
  window_start = millis();
  pid.setInputLimits(-200, 1350);
  pid.setOutputLimits(0, WINDOW_SIZE);
  
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN,LOW);

  postman.registerResource("pid", pid);
  postman.registerResource("profile", profile);
  postman.registerResource("about", about);
  postman.registerResource("debug", debug);
  
  Serial.begin(9600);
  Bluetooth.begin(9600);  
}

void loop()
{
  // ToDo: find a better way to detect that the MAX31855 is not connected than comparing with 0.0
  now = millis();
    
  // Read thermocouple
  if(now - sample_timestamp >= WINDOW_SIZE / MAX_SAMPLES) {
    temperature = thermocouple.readThermocouple();
    if(temperature < FAULT_OPEN && temperature != 0.0) {
      temperature_samples[sample_index] = temperature;
      sample_index = (sample_index + 1) & (MAX_SAMPLES - 1);
      temperature = 0;
      for(uint8_t i = 0; i != MAX_SAMPLES; i++)
        temperature += temperature_samples[i];
      temperature /= MAX_SAMPLES;
      sample_errors = 0;
    }
    else if(++sample_errors >= MAX_SAMPLES) {
      pid.setOutput(0);
      digitalWrite(RELAY_PIN, LOW);
    }
    pid.setInput(temperature);
    pid.setReference(thermocouple.readJunction());
    sample_timestamp = now;
  }
  
  // PID
  if(temperature < FAULT_OPEN && temperature != 0.0) {
    if(now - window_start > WINDOW_SIZE) {
      window_start += WINDOW_SIZE;
      if(pid.getAutomatic())
        pid.compute();
    }
    digitalWrite(RELAY_PIN, pid.getOutput() > now - window_start);
  }

  // Maximum kiln temperature protection
  if(temperature >= MAXIMUM_KILN_TEMPERATURE && temperature < FAULT_OPEN) {
    pid.setOutput(0);
    digitalWrite(RELAY_PIN, LOW);
  }
  
  // Profile
  if(temperature < FAULT_OPEN && temperature != 0.0) {
    if(profile.getPosition()) {
      pid.setAutomatic(true);
      position_changed = profile.compute(temperature, now);
      pid.setSetpoint(profile.getSetpoint());
      if(!profile.getPosition()) {
        pid.setAutomatic(false);
        pid.setOutput(0);
      }
      else if(position_changed) {
        if(profile.getKc() || profile.getTi() || profile.getTd())
          pid.setTunings(profile.getKc(), profile.getTi(), profile.getTd());
        else
          pid.setTunings(PID_DEFAULT_KC, PID_DEFAULT_TI, PID_DEFAULT_TD);
      }
    }
    else {
      profile.compute(temperature, now);
    }
  }
  
  // API
  if(framer.getState() == TPM_SENDING) {
    if(wireless_serial)
      Bluetooth.write(framer.getByteToSend());
    else
      Serial.write(framer.getByteToSend());
  }
  while(framer.getState() == TPM_RECEIVING && Serial.available() > 0) {
    wireless_serial = false;
    if(framer.putReceivedByte(Serial.read()) && framer.getLength()) {
      framer.setLength(postman.handlePack(pack_buffer, framer.getLength(), MAX_PACKET_LENGTH));
      framer.setState(TPM_SENDING);
      break;
    }
  }
  while(framer.getState() == TPM_RECEIVING && Bluetooth.available() > 0) {
    wireless_serial = true;
    if(framer.putReceivedByte(Bluetooth.read()) && framer.getLength()) {
      framer.setLength(postman.handlePack(pack_buffer, framer.getLength(), MAX_PACKET_LENGTH));
      framer.setState(TPM_SENDING);
      break;
    }
  }
  
  // Debug
  loop_time = (loop_time + (uint8_t)(millis() - now)) >> 1;  
}

