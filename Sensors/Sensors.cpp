//
//  Sensors.cpp
//  helloworld
//
//  Created by Simon Woodward on 05/12/2012.
//  Copyright (c) 2012 Simon Woodward. All rights reserved.
//

#include "Arduino.h"
#include "Servo.h"
#include "Sensors.h"

namespace Sensors {
  /***********
 sensor class
  ***********/
  void Sensor::setup(Sensors* sensors) {
    _subscriber = NULL;
    this->_sensors = sensors;
    _enabled = true;
  }

  void Sensor::setSubscriber(Subscriber* subs) {
    _subscriber = subs;
  }

  void Sensor::poll() {
    SensorData* ret = doPoll();
    if (ret && _subscriber)
      _subscriber->notify(ret);
  }


  /***********
 Sensors class
  ***********/
  void Sensors::poll() {
    for (int i = 0; i < MAX_SENSOR_COUNT && _sensors[i]; i ++) {
      if (_sensors[i]->enabled())
	_sensors[i]->poll();
    }
  }

  void Sensors::addSensor(Sensor* sensor) {
    sensor->setup(this);
    
    int i = 0;
    while (i < MAX_SENSOR_COUNT && _sensors[i])
      i++;
    
    if (i < MAX_SENSOR_COUNT)
      _sensors[i] = sensor;
  }

  /*********
	    SwitchSensor
  *********/
  SwitchSensor::SwitchSensor() {
  }

  void SwitchSensor::setup(int pin) {
    _pin = pin;    
    pinMode(pin, INPUT);
    // enable the pull-up resistor
    digitalWrite(pin, HIGH);

    _lastVal = digitalRead(_pin);
    _switchMillis = -1;
  }

  SwitchData switchRet;

  SensorData* SwitchSensor::doPoll() {
    if (_switchMillis != -1) {
      // Debounce interval is 5 milliseconds
      if (millis() - _switchMillis < 5) {
	return NULL;
      }
      _switchMillis = -1;
    }

    int newVal = digitalRead(_pin);
    SensorData* retVal = NULL;


    if (newVal != _lastVal) {
      switchRet._sensor = this;
      switchRet._val = newVal;
      retVal = &switchRet;
      // Crude debounce :(  don't want to spend the time to debounce now 
      // (we need to let the reaction to the switch happen ASAP), 
      // so set a flag to ignore readings for a bit
      _switchMillis = millis();
    }
    _lastVal = newVal;
    return retVal;
  }

  int SwitchSensor::state() {
    return digitalRead(_pin);
  }

  /***
      Wait a bit thingy
  ***/
  SensorData waiterData;
  SensorData* Waiter::doPoll() {
    if ((_from != -1) && (millis() - _from > _interval)) {
      _from = -1;
      waiterData._sensor = this;
      return &waiterData;
    } else 
      return NULL;
  }


  void Waiter::wait(int interval) {
    if (_from != -1) {
      Serial.println("Waiter::wait called while _from != -1. Rude.");
      return;
    }

    _interval = interval;
    _from = millis();
  }

  /***
      Step motor thing
  ***/
#define NO_DESTINATION -1


  //  Stop and check if we've reached our destination.
  //  Returns true if we have.
  bool StepMotor::checkForDestination() {
    // Turn the current winding off.  This is because the motor on my current 
    // project pulls 1.5A per winding, which will strain even a beefy power supply.
    digitalWrite(_pins[_currPin], LOW);

    // If we've arrived, indicate that.
    if (_destination == _pos) {
      _destination = NO_DESTINATION;
      return true;
    } else
      return false;
  }

  SensorData stepMotorData;

  SensorData* StepMotor::doPoll() {
    long now = millis();

    if (now - _millis < STEP_MOTOR_INTERVAL ||  _destination == NO_DESTINATION)
      return NULL;

    if (checkForDestination()) {
      stepMotorData._sensor = this;
      return &stepMotorData;
    }

    // Otherwise, activate the next winding in the right direction
    if (_destination < _pos) {
      _currPin --;
      _pos --;
      if (_currPin == -1)
	_currPin = 3;
    } else {
      _currPin ++;
      _pos ++;
      _currPin %= 4;
    }

    digitalWrite(_pins[_currPin], HIGH);
    _millis = now;
    return NULL;
  }

  StepMotor::StepMotor(int pin1, int pin2, int pin3, int pin4) {
    _pins[0] = pin1;
    _pins[1] = pin2;
    _pins[2] = pin3;
    _pins[3] = pin4;
    for (int i = 0; i < 4; i ++) {
      pinMode(_pins[i], OUTPUT);
    }
    _destination = NO_DESTINATION;
    _currPin = 0;
    _pos = 0;
  }

  void StepMotor::gotoStep(int step) {
    if (_destination != NO_DESTINATION) {
      Serial.println("gotoStep called while the motor's busy. Rude.");
      return;
    } 

    _destination = step;
    _millis = millis();
  }

  // Emergency stop
  void StepMotor::stop() {
    if (_destination == NO_DESTINATION)
      return;

    // Set destination to wherever we are now, and call the bit of 
    // the poll() that turns off the current winding and cleans 
    // up the state.
    _destination = _pos;
    checkForDestination();
  }

  /******
	 Wrapper around servo to slow it down a bit
  ******/
  SensorData servoData;
  void SlowServo::setup(int pin, int initialPos, int interval) {
    _interval = interval;
    _pos = initialPos;
    _dest = _pos;
    _state = ssStill;
    _servo.attach(pin);
    _servo.write(initialPos);
  }

  SensorData* SlowServo::doPoll() {
    SensorData* ret = NULL;
    if (_state != ssStill) {
      long now = millis();
      switch (_state) {
      case ssWaitingABit:
	if (now - _lastMillis >= SENSOR_WAIT_INTERVAL) {
	  servoData._sensor = this;
	  _state = ssStill;
	  ret = &servoData;
	}
	break;
      case ssGoing:
	if (_dest == _pos) {
	  _state = ssWaitingABit;
	  _lastMillis = now;
	} else if (now - _lastMillis >= _interval) {
	  int direction = (_dest < _pos ? -1 : 1);
	  _pos += direction;
	  _lastMillis = now;
	  _servo.write(_pos);
	}
	break;
      }
    }
    return ret;
  }
   
  SlowServo::SlowServo() {
  }

  void SlowServo::goTo(int degrees) {
    _dest = degrees;
    _lastMillis = 0; // Low number so the next doPoll will fire
    _state = ssGoing;
  }
  
  int SlowServo::pos() {
    return _pos;
  }

  /*******
	  VoltageDropSensor
  *******/
  VoltageDropSensor::VoltageDropSensor(int pin) {
    _pin = pin;
    _writeToSerial = false;
    setThreshold(0); // essentially disables the sensor
    _triggered = false;
  }

  VoltageDropSensor::VoltageDropSensor(int pin, int threshold) {
    _pin = pin;
    _writeToSerial = false;
    setThreshold(threshold);
    _triggered = false;
  }

  void  VoltageDropSensor::setThreshold(int threshold) {
    _threshold = threshold;
  }

  int VoltageDropSensor::threshold() {
    return _threshold;
  }

  SensorData voltageDropData;
  SensorData* VoltageDropSensor::doPoll() {
    int val = analogRead(_pin);
    if (_writeToSerial) {
      avgCount ++;
      avgTotal = (avgTotal > val ? val : avgTotal);
      if (avgCount == 2000) {
	Serial.println(avgTotal);
	avgCount = 0;
	avgTotal = 1000;
      }
    }
    if ( val < _threshold) {
      voltageDropData._sensor = this;
      _triggered = true;
      return &voltageDropData;
    } else
      return NULL;
  }

  int VoltageDropSensor::writeToSerial() {
    _writeToSerial = ! _writeToSerial;
  }

  void VoltageDropSensor::watchForTrigger() {
    _triggered = false;
  }

  bool VoltageDropSensor::triggered() {
    return _triggered;
  }

  int VoltageDropSensor::state() {
    return analogRead(_pin);
  }
} // Namespace sensors


