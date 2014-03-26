//  Kiln Monkey - PID Resource - Copyright 2012 by Francisco Castro <http://fran.cc>
//
//  Based on the PID library for mbed by Aaron Berk Copyright (c) 2010 ARM Limited
//  ... that is a port from the Arduino PID library by Brett Beauregard <http://brettbeauregard.com>
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

#define MANUAL_MODE 0
#define AUTO_MODE   1
#define MAX_ACC_ERROR 1.0

#include <TinyPostman.h>

class PID : public Resource {
public:
    PID(float Kc, float tauI, float tauD, float interval);
    void setInputLimits(float inMin , float inMax);
    void setOutputLimits(float outMin, float outMax);
    void setTunings(float Kc, float tauI, float tauD);
    void reset(void);
    void setAutomatic(bool mode);                             // mode: false -> Manual     true -> Auto
    void setInterval(float interval);                    // interval: PID calculation peformed every interval seconds.
    void setSetpoint(float sp) { setPoint_ = sp; };      // sp: The set point as a real world value.
    void setInput(float pv)  { processVariable_ = pv; }; // pv: The process value as a real world value.
    void setBias(float bias) { bias_ = bias; usingFeedForward = 1; }; // bias: The bias for the controller output.        
    float compute(void);                                  // @return The controller output as a float between outMin and outMax.

    float getInMin() { return inMin_; };
    float getInMax() { return inMax_; };
    float getOutMin() { return outMin_; };
    float getOutMax() { return outMax_; };
    float getInterval() { return tSample_; };
    float getPParam() { return pParam_; };
    float getIParam() { return iParam_; };
    float getDParam() { return dParam_; };
    float getOutput() { return realOutput_; };

    void setPParam(float value) { setTunings(value, iParam_, dParam_); };
    void setIParam(float value) { setTunings(pParam_, value, dParam_); };
    void setDParam(float value) { setTunings(pParam_, iParam_, value); };
    void setOutput(float value) { realOutput_ = value; };
    void setReference(float value) { reference = value; };
    bool getAutomatic() { return inAuto; };
    float getInput() { return processVariable_; };
    
    uint8_t get(Request &request);
    uint8_t put(Request &request);

private:
    bool usingFeedForward;
    bool inAuto;

    //Actual tuning parameters used in PID calculation.
    float Kc_;
    float tauR_;
    float tauD_;
    
    //Raw tuning parameters.
    float pParam_;
    float iParam_;
    float dParam_;
    
    //The point we want to reach.
    float setPoint_;
    float reference;
    //The thing we measure.
    float processVariable_;  
    float prevProcessVariable_;
    //The output that affects the process variable.
    float controllerOutput_; 
    float prevControllerOutput_;

    //We work in % for calculations so these will scale from
    //real world values to 0-100% and back again.
    float inMin_;
    float inMax_;
    float inSpan_;
    float outMin_;
    float outMax_;
    float outSpan_;
    
    float accError_;    //The accumulated error, i.e. integral.
    float bias_;        //The controller output bias.
    float tSample_;     //The interval between samples.
    float realOutput_;  //Controller output as a real world value.
};

PID::PID(float Kc, float tauI, float tauD, float interval) {
    usingFeedForward = false;
    inAuto           = false;

    //Default the limits to the full range of I/O: 3.3V
    //Make sure to set these to more appropriate limits for
    //your application.
    setInputLimits(0.0, 3.3);
    setOutputLimits(0.0, 3.3);

    tSample_ = interval;

    setTunings(Kc, tauI, tauD);

    setPoint_             = 0.0;
    processVariable_      = 0.0;
    prevProcessVariable_  = 0.0;
    controllerOutput_     = 0.0;
    prevControllerOutput_ = 0.0;

    accError_ = 0.0;
    bias_     = 0.0;
    
    realOutput_ = 0.0;
}

void PID::setInputLimits(float inMin, float inMax) {
    //Make sure we haven't been given impossible values.
    if (inMin >= inMax) {
        return;
    }

    //Rescale the working variables to reflect the changes.
    prevProcessVariable_ *= (inMax - inMin) / inSpan_;
    accError_            *= (inMax - inMin) / inSpan_;

    //Make sure the working variables are within the new limits.
    if (prevProcessVariable_ > 1) {
        prevProcessVariable_ = 1;
    } else if (prevProcessVariable_ < 0) {
        prevProcessVariable_ = 0;
    }

    inMin_  = inMin;
    inMax_  = inMax;
    inSpan_ = inMax - inMin;
}

void PID::setOutputLimits(float outMin, float outMax) {
    //Make sure we haven't been given impossible values.
    if (outMin >= outMax) {
        return;
    }

    //Rescale the working variables to reflect the changes.
    prevControllerOutput_ *= (outMax - outMin) / outSpan_;

    //Make sure the working variables are within the new limits.
    if (prevControllerOutput_ > 1) {
        prevControllerOutput_ = 1;
    } else if (prevControllerOutput_ < 0) {
        prevControllerOutput_ = 0;
    }

    outMin_  = outMin;
    outMax_  = outMax;
    outSpan_ = outMax - outMin;
}

void PID::setTunings(float Kc, float tauI, float tauD) {
    //Verify that the tunings make sense.
    if (Kc == 0.0 || tauI < 0.0 || tauD < 0.0) {
        return;
    }

    //Store raw values to hand back to user on request.
    pParam_ = Kc;
    iParam_ = tauI;
    dParam_ = tauD;

    float tempTauR;

    if (tauI == 0.0) {
        tempTauR = 0.0;
    } else {
        tempTauR = (1.0 / tauI) * tSample_;
    }

    //For "bumpless transfer" we need to rescale the accumulated error.
    if (inAuto) {
        if (tempTauR == 0.0) {
            accError_ = 0.0;
        } else {
            accError_ *= (Kc_ * tauR_) / (Kc * tempTauR);
        }
    }

    Kc_   = Kc;
    tauR_ = tempTauR;
    tauD_ = tauD / tSample_;
}

void PID::reset(void) {
    float scaledBias = 0.0;

    if (usingFeedForward) {
        scaledBias = (bias_ - outMin_) / outSpan_;
    } else {
        scaledBias = (realOutput_ - outMin_) / outSpan_;
    }

    prevControllerOutput_ = scaledBias;
    prevProcessVariable_  = (processVariable_ - inMin_) / inSpan_;

    //Clear any error in the integral.
    accError_ = 0;
}

void PID::setAutomatic(bool mode) {
    //We were in manual, and we just got set to auto.
    //Reset the controller internals.
    if (mode && !inAuto) {
        reset();
    }

    inAuto = mode;
}

void PID::setInterval(float interval) {
    if (interval > 0) {
        //Convert the time-based tunings to reflect this change.
        tauR_     *= (interval / tSample_);
        accError_ *= (tSample_ / interval);
        tauD_     *= (interval / tSample_);
        tSample_   = interval;
    }
}

float PID::compute() {
    //Pull in the input and setpoint, and scale them into percent span.
    float scaledPV = (processVariable_ - inMin_) / inSpan_;

    if (scaledPV > 1.0) {
        scaledPV = 1.0;
    } else if (scaledPV < 0.0) {
        scaledPV = 0.0;
    }

    float scaledSP = (setPoint_ - inMin_) / inSpan_;
    if (scaledSP > 1.0) {
        scaledSP = 1;
    } else if (scaledSP < 0.0) {
        scaledSP = 0;
    }

    float error = scaledSP - scaledPV;

    //Check and see if the output is pegged at a limit and only
    //integrate if it is not. This is to prevent reset-windup.
    if (!(prevControllerOutput_ >= 1 && error > 0) && !(prevControllerOutput_ <= 0 && error < 0)) {
        accError_ += error;
    }
    if(accError_ > MAX_ACC_ERROR) {
        accError_ = MAX_ACC_ERROR;
    } else if(accError_ < -MAX_ACC_ERROR) {
        accError_ = -MAX_ACC_ERROR;
    }

    //Compute the current slope of the input signal.
    float dMeas = (scaledPV - prevProcessVariable_) / tSample_;

    float scaledBias = 0.0;

    if (usingFeedForward) {
        scaledBias = (bias_ - outMin_) / outSpan_;
    }

    //Perform the PID calculation.
    controllerOutput_ = scaledBias + Kc_ * (error + (tauR_ * accError_) - (tauD_ * dMeas));

    //Make sure the computed output is within output constraints.
    if (controllerOutput_ < 0.0) {
        controllerOutput_ = 0.0;
    } else if (controllerOutput_ > 1.0) {
        controllerOutput_ = 1.0;
    }

    //Remember this output for the windup check next time.
    prevControllerOutput_ = controllerOutput_;
    //Remember the input for the derivative calculation next time.
    prevProcessVariable_  = scaledPV;

    //Scale the output from percent span back out to a real world number.
    realOutput_ = (controllerOutput_ * outSpan_) + outMin_;
    return realOutput_;
}


uint8_t PID::get(Request &request)
{
    request.writer.openMap();
    request.writer.putString("automatic");
    request.writer.putBoolean(inAuto);
    request.writer.putString("setpoint");
    request.writer.putReal(setPoint_);
    request.writer.putString("reference");
    request.writer.putReal(reference);
    request.writer.putString("input");
    request.writer.putReal(processVariable_);
    request.writer.putString("output");
    request.writer.putInteger(realOutput_);
    request.writer.putString("kc");
    request.writer.putReal(pParam_);
    request.writer.putString("ti");
    request.writer.putReal(iParam_);
    request.writer.putString("td");
    request.writer.putReal(dParam_);
    request.writer.putString("acc_error");
    request.writer.putReal(accError_);
    request.writer.close();
    return TPM_205_Content;
}

uint8_t PID::put(Request &request)
{
  request.reader.next();
  if(request.reader.openMap()) {
    while(request.reader.next()) {
      if     (request.reader.match("automatic")) setAutomatic(request.reader.getBoolean());
      else if(request.reader.match("setpoint"))  setPoint_ = request.reader.getReal();
      else if(request.reader.match("reference")) reference = request.reader.getReal();
      else if(request.reader.match("input"))     processVariable_ = request.reader.getReal();
      else if(request.reader.match("output"))    realOutput_ = request.reader.getInteger();
      else if(request.reader.match("kc"))        setPParam(request.reader.getReal());
      else if(request.reader.match("ti"))        setIParam(request.reader.getReal());
      else if(request.reader.match("td"))        setDParam(request.reader.getReal());
      else request.reader.next();        
    }
    request.reader.close();
    return TPM_204_Changed;
  }
  else
    return TPM_400_Bad_Request;
}

