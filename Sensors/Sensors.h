//
//  Todo.h
//  helloworld
//
//  Created by Simon Woodward on 05/12/2012.
//  Copyright (c) 2012 Simon Woodward. All rights reserved.
//

#ifndef Sensors_h
#define Sensors_h

namespace Sensors {
  class Sensors;
  class Sensor;

  /******
	 Container to pass the data from the sensor to the subscribers.  Can be extended 
	 by the individual sensors if they need to add more data.
  *******/
  struct SensorData {
    Sensor* _sensor;
  };

  /******
	 Classes that want to receives notifs from sensor implement this interface 
	 and then pass a pointer to themselves to Sensor::addSubscriber().
  ******/
  class Subscriber {
  public:
    virtual void notify(SensorData* data) = 0;
  };

  /*******
	  A sensor thingy - the bit of code that checks a switch or a serial connection,
	  etc.  
 
	  Maintains a list of subscribers that it notifies when its thing is triggered.
  *******/
#define MAX_SUBSCRIBER_COUNT 5
  class Sensor {
  protected:
    // Ptr to the sensor collection that owns this sensor
    Sensors* _sensors;
    Subscriber* _subscriber;
    
    bool _enabled;
    
    // Check the sensor.  Returns non-null if the sensors has data to provide.
    // The SensorData in the return value belongs to the sensor; the caller
    // doesn't need to free it.
    virtual SensorData* doPoll() = 0;
    
  public:
    // Any required setup (optional)
    virtual void setup(Sensors* sensors);
    
    // Add a subscriber to the sensor; subs will be notified if this sensor has
    // data to report.
    void setSubscriber(Subscriber* subs);
    
    // Check the sensor and notify any subscribers if there's data.
    void poll();

    // Won't be polled if it the sensor is disabled
    void enable() { _enabled = true; }
    void disable() { _enabled = false; }
    bool enabled() { return _enabled; }
  };

  /*********
	    List of sensors that this program is monitoring.
  ********/
  const int MAX_SENSOR_COUNT = 10;

  class Sensors {
  private:
    Sensor* _sensors[MAX_SENSOR_COUNT];

  public:
    void poll();
    void addSensor(Sensor* sensor);
  };

  /*********
	    Sensor that watches a switch for it to be pressed.  This is 
	    a switch connected to voltage, waiting for the pin to be pulled
	    low.  It uses the arduino pull-up resistors on the digital pins.
  *********/
  struct SwitchData : public SensorData {
    int _val;
  };

  class SwitchSensor : public Sensor {
  protected:
    int _pin, _lastVal;
    long _switchMillis;
    virtual SensorData* doPoll();
  public:
    SwitchSensor();
    void setup(int pin);

    int state();
  };

  /***
      Sensor that just waits a bit and then fires
  ***/
  class Waiter : public Sensor {
  protected:
    long _from;
    int _interval;
    
    
    virtual SensorData* doPoll();

  public:
    Waiter() {
      _from = -1;
    }
    void wait(int interval);
  };

  /********
	   Thing to manage the step motor.  It's not really a sensor,
	   but it need a poll call and to notify things when it's finished
	   its current job...
  ********/
  //enum Direction {foward, backward};
#define STEP_MOTOR_INTERVAL 2
  class StepMotor : public Sensor {
  protected:
    //  Direction _direction;
    virtual SensorData* doPoll();
    // Where we're going
    int _destination;
    // Where we are
    int _pos;
    int _currPin;

    // The time we last moved a step
    long _millis;

    // Which pins our windings are on; such that activating each in 
    // order will give us a simple one-phase step
    int _pins[4];

    bool checkForDestination();

  public:
    // Pin 1 is winding 1, wire 1; 
    // Pin 2 is winding 2, wire 1;
    // Pin 3 is winding 1, wire 2;
    // Pin 4 is winding 2, wire 2.
    StepMotor(int pin1, int pin2, int pin3, int pin4);

    void gotoStep(int step);

    // Emergency stop
    void stop();
    
    // reset the position to zero.
    void zero() {_pos = 0;}
  };

  /*******
	  Wrapper around servo to make it a bit slower!
  ********/
  enum ServoState {ssStill, ssGoing, ssWaitingABit};
  #define SENSOR_WAIT_INTERVAL 200 
  class SlowServo : public Sensor {
  protected:
    Servo _servo;
    int _dest;
    int _pos;
    int _interval;
    long _lastMillis;
    ServoState _state;

    virtual SensorData* doPoll();

  public:
    SlowServo();

    void setup(int pin, int initialPos, int interval);
    void goTo(int degrees);
    int pos();
  };

  /********
	   poll when a voltage on a pin drops below a threshold.
   *******/
  class VoltageDropSensor : public Sensor {
  protected:
    int _threshold;
    bool _writeToSerial;
    int _pin;
    bool _triggered;

    int avgCount;
    int avgTotal;

    virtual SensorData* doPoll();

  public:
    VoltageDropSensor(int pin);
    VoltageDropSensor(int pin, int threshold);

    void setThreshold(int threshold);
    int threshold();

    // toggle outputting the current value from the LDR to serial if 
    // it's below threshold
    int writeToSerial();

    // Trigger mode.  Call watchForTrigger() sets the sensor to 
    // untriggered.  If the voltage on the pin drops below the threshold,
    // the sensor becomes triggered.
    void watchForTrigger();
    bool triggered();

    // Direct access to the state
    int state();
  };

}; // namespace

#endif
