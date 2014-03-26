#!/usr/bin/python

import sys
import time
import json
from tinypostman import *

usage = """\nUsage: %s <serial port> [action] [parameters]

Actions:

    run [JSON file]     Runs the profile contained in the specified JSON file
                        or the profile already stored in RAM if no file is 
                        specified.
    
    pause               Pauses the running profile and holds the current 
                        temperature.
    
    continue            Resumes the paused profile.
    
    position [value]    ...
    
    setpoint [value]    Sets the pid to automatic mode and the setpoint to
                        [value] in celsius degrees or displays the current
                        setpoint if no value is specified. Stops the profile
                        if running.

    tunning [kc ti td]  ...
                        
    output [value]      Sets the pid to manual mode and the output to [value]
                        or displays the current output in no value is 
                        specified. Stops the profile if running.
    
    off                 Stops the profile if running, disables the pid 
                        automatic mode and turns off the output.
                        
    log [file]          ...
    
    debug               ...
                        
""" % sys.argv[0]



if len(sys.argv) < 2:
    print(usage)
    sys.exit()
    
pm = Postman(sys.argv[1])
pm.debug = False

def get(resource, data=None):
    response = pm.get(resource, data)
    if response[0] != TPM_205_Content:
        print "Cannot get the '%s' resource: %s" % (resource, TPM_RESPONSE_TEXT.get(response[0], str(response[0])))
        return None
    else:
        return response[1]

def put(resource, data=None):
    response = pm.put(resource, data)
    if response[0] != TPM_204_Changed:
        print("Cannot put the '%s' resource: %s" % (resource, TPM_RESPONSE_TEXT.get(response[0], str(response[0]))))
        return False
    else:
        return True


if len(sys.argv) > 2:
    action = sys.argv[2]
    if action == "run":
        if len(sys.argv) > 3:
            put("profile", json.load(open(sys.argv[3], "r")))
        put("profile", {'position': 1, 'hold': False})
    elif action == "pause":
        put("profile", {'hold': True})
    elif action == "continue":
        put("profile", {'hold': False})
    elif action == "position":
        if len(sys.argv) > 3:
            put("profile", {'position': int(sys.argv[3])})
        else:
            print("Position: %i" % get("profile")["position"])
    elif action == "setpoint":
        if len(sys.argv) > 3:
            put("profile", {'position': 0})
            put("pid", {'automatic': True, 'setpoint': float(sys.argv[3])})
        else:
            print("Setpoint: %i" % get("pid")["setpoint"])
    elif action == "output":
        if len(sys.argv) > 3:
            put("profile", {'position': 0})
            put("pid", {'automatic': False, 'output': int(sys.argv[3])})
        else:
            print("Output: %i" % get("pid")["output"])
    elif action == "tunning":
        if len(sys.argv) > 5:
            put("pid", {'kc': int(sys.argv[3]), 'ti': int(sys.argv[4]), 'td': int(sys.argv[5])})
        else:
            response = get("pid")
            print("Tunning: kc %i, ti %i, td %i" % (response["kc"], response["ti"], response["td"]))
    elif action == "off":
        put("profile", {'position': 0})
        put("pid", {'automatic': False, 'output': 0})
    elif action == "log":
        while(True):
            pid = get("pid")
            profile = get("profile")
            if pid is not None and profile is not None:
                print("Setpoint: %.01f\tInput: %.01f\tOutput: %i\tAccError: %.03f\tPosition: %i\tStep time: %i" % (pid['setpoint'], pid['input'], pid['output'], pid['acc_error'], profile['position'], profile['step_elapsed_time']))
                if len(sys.argv) > 3:
                    log = open(sys.argv[3], "a")
                    log.write("%.01f,%.01f,%i,%.03f,%i,%i\n" % (pid['setpoint'], pid['input'], pid['output'], pid['acc_error'], profile['position'], profile['step_elapsed_time']))
                    log.close()
            time.sleep(1)
    elif action == "debug":
        print("debug:")
        print(json.dumps(get("debug"), sort_keys=True, indent=4))
        print("pid:")
        print(json.dumps(get("pid"), sort_keys=True, indent=4))
        print("profile:")
        print(json.dumps(get("profile"), sort_keys=True, indent=4))
    else:
        print(usage)
        sys.exit()        
else:
    print(usage)
    sys.exit()        
