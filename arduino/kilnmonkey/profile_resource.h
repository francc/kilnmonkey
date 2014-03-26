//  Kiln Monkey - Profile Resource - Copyright 2012 by Francisco Castro <http://fran.cc>
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

#define MAX_STEPS 14
#define MAX_PROFILE_NAME_LENGTH 24

#include <TinyPostman.h>

class Profile : public Resource {
  private:
    char      name[MAX_PROFILE_NAME_LENGTH];
    bool      hold;
    uint8_t   length;
    int8_t    position;
    int8_t    prev_position;
    double    setpoint;
    double    step_start_point;
    uint32_t  step_start_time;
    uint32_t  step_elapsed_time;
    struct {
      double   end_point;
      long     time;
      double   slope;
      bool     wait;
      int16_t  kc;
      int16_t  ti;
      int16_t  td;
    } steps[MAX_STEPS];
  public:
    Profile();
    bool compute(double current_temperature, uint32_t now);
    bool getHold() { return hold; };
    void setHold(bool value) { hold = value; };
    double getSetpoint() { return setpoint; };
    int8_t getPosition() { return position; };
    void setPosition(int8_t value) { position = value; };
    int16_t getKc() { return position ? steps[position - 1].kc : 0; };
    int16_t getTi() { return position ? steps[position - 1].ti : 0; };
    int16_t getTd() { return position ? steps[position - 1].td : 0; };

    uint8_t get(Request &request);
    uint8_t put(Request &request);
};


Profile::Profile()
{
  hold = false;
  length = 0;
  position = 0;
  prev_position = 0;
  setpoint = 0;
  step_start_point = 0;
  step_start_time = 0;
  step_elapsed_time = 0;
}

bool Profile::compute(double current_temperature, uint32_t now)
{
  bool position_changed = false;
  if(!hold) {
    if(position != prev_position) {
      prev_position = position;
      position_changed = true;
      if(position > 0 && position <= length) {
        step_start_time =  now;
        step_start_point = (position == 1 || steps[position - 1].slope) ? current_temperature : steps[position - 2].end_point;
        if(steps[position - 1].slope)
          steps[position - 1].time =  abs((steps[position - 1].end_point - step_start_point) / steps[position - 1].slope);
      }
    }  
    if(position > 0 && position <= length) {
      step_elapsed_time = now - step_start_time;
      if(step_elapsed_time < steps[position - 1].time)
        setpoint = round(((steps[position - 1].end_point - step_start_point) * step_elapsed_time / steps[position - 1].time + step_start_point) * 10.0) / 10.0;
      else {
        setpoint = steps[position - 1].end_point;
        if(!steps[position - 1].wait || abs(setpoint - current_temperature) < 1)
          position = position < length ? position + 1 : 0;
      }
    }
  }
  else
    step_start_time = now - step_elapsed_time;
    
  return position_changed;  
}

uint8_t Profile::get(Request &request)
{
  request.writer.openMap();
  request.writer.putString("name");
  request.writer.putString(name);
  request.writer.putString("position");
  request.writer.putInteger(position);
  request.writer.putString("hold");
  request.writer.putBoolean(hold);
  request.writer.putString("setpoint");
  request.writer.putReal(setpoint);
  request.writer.putString("step_elapsed_time");
  request.writer.putInteger(step_elapsed_time / 1000);
  request.writer.putString("steps");
  request.writer.openList();
  for(uint8_t i = 0; i != length; i++) {
    request.writer.openMap();
    request.writer.putString("e");
    request.writer.putReal(steps[i].end_point);
    request.writer.putString("t");
    request.writer.putInteger(steps[i].time / 1000);
    request.writer.putString("s");
    request.writer.putReal(steps[i].slope * 1000);
    request.writer.putString("w");
    request.writer.putBoolean(steps[i].wait);
    request.writer.putString("kc");
    request.writer.putInteger(steps[i].kc);
    request.writer.putString("ti");
    request.writer.putInteger(steps[i].ti);
    request.writer.putString("td");
    request.writer.putInteger(steps[i].td);
    request.writer.close();
  }
  request.writer.close();
  request.writer.close();
  return TPM_205_Content;
}
  
uint8_t Profile::put(Request &request)
{
  request.reader.next();
  if(request.reader.openMap()) {
    while(request.reader.next()) {
      if     (request.reader.match("name"))     request.reader.getString(name, MAX_PROFILE_NAME_LENGTH);
      else if(request.reader.match("position")) position = request.reader.getInteger();
      else if(request.reader.match("hold"))     hold = request.reader.getBoolean();
      else if(request.reader.match("steps"))    {
        if(request.reader.openList()) {
          length = 0;
          while(request.reader.next() && length < MAX_STEPS) {              
            if(request.reader.openMap()) {
              steps[length].end_point = 0;
              steps[length].time = 0;
              steps[length].slope = 0;
              steps[length].wait = false;
              steps[length].kc = 0;
              steps[length].ti = 0;
              steps[length].td = 0;
              while(request.reader.next()) {
                if     (request.reader.match("e"))  steps[length].end_point = request.reader.getReal();
                else if(request.reader.match("t"))  steps[length].time = request.reader.getInteger() * 1000;
                else if(request.reader.match("s"))  steps[length].slope = request.reader.getReal() / 1000;
                else if(request.reader.match("w"))  steps[length].wait = request.reader.getBoolean();
                else if(request.reader.match("kc")) steps[length].kc = request.reader.getInteger();
                else if(request.reader.match("ti")) steps[length].ti = request.reader.getInteger();
                else if(request.reader.match("td")) steps[length].td = request.reader.getInteger();
                else request.reader.next();
              }
              request.reader.close();
              length += 1;
            }
          }
          request.reader.close();
        }
      }
      else request.reader.next();        
    }
    request.reader.close();
    return TPM_204_Changed;
  }
  else
    return TPM_400_Bad_Request;
}


