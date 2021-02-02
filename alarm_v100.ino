
//#define DEBUG

// Code of remoute acces commands 
#define COMMAND_ENABLE_ALL            0x01
#define COMMAND_DISABLE_ALL           0x02
#define COMMAND_ENABLE_VIBRO_SENSOR   0x04
#define COMMAND_ENABLE_VOLUME_SENSOR  0x08

// Ports of Remoute Acces
#define RA_VT 3
#define RA_B  7   // D0 - B
#define RA_D  6   // D1 - D
#define RA_A  5   // D2 - A
#define RA_C  4   // D3 - C 

#define VIBRO_SENSOR 8  // Port of vibro sensor
#define VIBRO_SENSOR_NEXT_TRIGGED_DELAY  100
#define VIBRO_SENSOR_LAST_TRIGGED_DELAY  500

#define MOVE_SENSOR 9 // Port of move sensor
#define MOVE_SENSOR_NEXT_TRIGGED_DELAY  3000
#define MOVE_SENSOR_LAST_TRIGGED_DELAY  4000

#define BUZZER 12 // Port of buzzer
#define BUZZER_SHORT_PERIOD 250 // Duration of short signlas
#define BUZZER_LONG_PERIOD  1000  // Duration of long signals

#define LED 11  // Port of led indicator
#define LED_SHORT_PERIOD 250  // Duration of short flash
#define LED_LONG_PERIOD  1000 // Duration of long flash

/*
 * Remoute Acces 
 */
class cRemoteAcces
{
  public:
    cRemoteAcces(byte pinVr, byte pinA, byte pinB, byte pinC, byte pinD)
    {
      portVr = pinVr;
      portA = pinA;
      portB = pinB;
      portC = pinC;
      portD = pinD;
    }
  
    void init()
    {
      pinMode(portVr,  INPUT);   
      pinMode(portA,   INPUT);   
      pinMode(portB,   INPUT);   
      pinMode(portC,   INPUT);   
      pinMode(portD,   INPUT);   
    }

    void scan()
    {
      bool anyKeyIsPressing = digitalRead(portVr);

      keyAPressed = digitalRead(portA);  
      keyBPressed = digitalRead(portB);  
      keyCPressed = digitalRead(portC);  
      keyDPressed = digitalRead(portD);  

      if(anyKeyWasPressed and !anyKeyIsPressing)
        recivedCommand();    

      anyKeyWasPressed = anyKeyIsPressing;
    }

    byte getCommand()
    {
      return command;  
    }

    void resetCommand()
    {
      command = 0;  
    }

  private:
    byte command;
    byte portVr;
    byte portA;
    byte portB;
    byte portC;
    byte portD;
  
    bool anyKeyWasPressed = false;

    bool keyAPressed = false;
    bool keyBPressed = false;
    bool keyCPressed = false;
    bool keyDPressed = false;

    void recivedCommand()
    {
      byte recivedCommand = 0;
      
      recivedCommand = (byte)keyAPressed<<0 | (byte)keyBPressed<<1 | (byte)keyCPressed<<2 | (byte)keyDPressed<<3;
      
      analyze(recivedCommand);
       
      keyAPressed = false;  
      keyBPressed = false;  
      keyCPressed = false;  
      keyDPressed = false;  
      
    }

    void analyze(byte recivedCommand)
    {
      if(command == 0)
        command = recivedCommand;          
    }

};


/*
 * Sensor
 */
class bcSimpleSensor
{
  public:
    bcSimpleSensor(byte pin, bool invert, unsigned long iNextTriggedDelay, unsigned long iLastTriggedDelay)
    {
      portSensor = pin;
      nextTriggedDelay = iNextTriggedDelay;
      lastTriggedDelay = iLastTriggedDelay;
      invertInput = invert;
    }

    void init()
    {
      pinMode(portSensor,  invertInput?INPUT_PULLUP:INPUT);
      previousSensorState = digitalRead(portSensor);  
    }

    void scan()
    {
      if(enable)
        analyze(getSensorState());
    }

    void enableSensor()
    {
      enable = true; 
      //previousSensorState = digitalRead(portSensor);  
    }

    void disableSensor()
    {
      enable = false;
      resetAlarmStatus();
      resetWarningStatus(); 
    }

    bool getAlarmStatus()
    {
      return alarmStatus; 
    }

    bool getWarningStatus()
    {
      return warningStaus;  
    }

    void resetAlarmStatus()
    {
      alarmStatus = false;  
    }
    
    void resetWarningStatus()
    {
      warningStaus = false;    
    }

  private:
    bool enable = false;

    bool alarmStatus = false;
    bool warningStaus = false;
   
    bool invertInput = false;
  
    unsigned long firstSensorTriggedTime = 0;
    
    unsigned long nextSensorTriggedTime = 0;
    unsigned long nextTriggedDelay;
    
    unsigned long lastSensorTriggedTime = 0;
    unsigned long lastTriggedDelay;
  
   void analyze(bool sensorState)
   {
    unsigned long currentTime = millis();
  
    if(sensorState and enable)
    {
      if(firstSensorTriggedTime == 0)  
      {
        firstSensorTriggedTime = currentTime;
        nextSensorTriggedTime = firstSensorTriggedTime + nextTriggedDelay;  
        lastSensorTriggedTime = firstSensorTriggedTime + lastTriggedDelay;
  
        doWarning();
      }
      
      if(currentTime>=nextSensorTriggedTime and currentTime<=lastSensorTriggedTime)
        doAlarm();
        
    }      
    else
    {  
      if(currentTime >= lastSensorTriggedTime or !enable)
        firstSensorTriggedTime = 0;
    }
   }

  void doWarning()
  {
    warningStaus = true;
  }

  void doAlarm()
  {
    alarmStatus = true;
  }

 protected:
  byte portSensor;
  bool previousSensorState;
  virtual bool getSensorState();
};



/*
 * Vibro sensor
 */
class cVibroSensor : public bcSimpleSensor
{
  public:
    cVibroSensor(byte pin, unsigned long iNextTriggedDelay, unsigned long iLastTriggedDelay) : bcSimpleSensor(pin, true, iNextTriggedDelay, iLastTriggedDelay) {};

  protected:
    bool getSensorState()
    {
        bool sensorState = digitalRead(portSensor);
        if(sensorState != previousSensorState)
        {
          previousSensorState = sensorState;
          return true;
        }
        
        return false;
    }
 
};

/*
 * Move sensor
 */
class cMoveSensor : public bcSimpleSensor
{
  public:
    cMoveSensor(byte pin, unsigned long iNextTriggedDelay, unsigned long iLastTriggedDelay) : bcSimpleSensor(pin, false, iNextTriggedDelay, iLastTriggedDelay) {};

  protected:
    bool getSensorState()
    {
        bool sensorState = digitalRead(portSensor);
        
        if(!previousSensorState and sensorState)
        {
            return digitalRead(portSensor);
        }
        
        previousSensorState = sensorState;
    }
 
};


class cSimpleIndicator
{
  public:
    cSimpleIndicator(byte pin, unsigned int _shortPeriod, unsigned int _longPeriod)  
    {
      portIndicator = pin;
      shortPeriod = _shortPeriod; 
      longPeriod = _longPeriod; 
    }

    void init()
    {
       pinMode(portIndicator, OUTPUT);  
    }

    void playMessage(String _message, unsigned int repeatQuantity = 1)
    {
      disable();
      message = _message;
      length = message.length();
      counter = repeatQuantity;
    }

    virtual void disable()
    {
      off();
      counter = 0;
      position = 0;
      nextCheckTime = 0;      
    }

    bool active()
    {
      return counter > 0; 
    }
    
    void loop()
    {
      unsigned int currentTime = millis();

      if(counter<=0)
      {
        if(nextCheckTime < currentTime)
          disable();
        return;
      }

      if(nextCheckTime > currentTime)
        return;

      switch(message[position])
        {
          case '-':
            nextCheckTime = currentTime + longPeriod;
            on();
            break;
          case '.':
            nextCheckTime = currentTime + shortPeriod;
            on();
            break;
          case ' ':
            nextCheckTime = currentTime + longPeriod;
            off();
            break;
        }

      position++;
      
      if(position >= length)
      {
        counter--;
        position = 0;
      }
      
    }

  private:
    byte portIndicator;    

    unsigned int counter = 0;
    String message;
    unsigned int length = 0;
    unsigned int position = 0;
    unsigned int nextCheckTime = 0;

    unsigned int shortPeriod;
    unsigned int longPeriod;
    
    void on()
    {
      digitalWrite(portIndicator, HIGH);
    }
  
    void off()
    {
      digitalWrite(portIndicator, LOW);    
    }
};


class cBuzzer : public cSimpleIndicator
{
  public:
    cBuzzer(byte pin) : cSimpleIndicator(pin, BUZZER_SHORT_PERIOD, BUZZER_LONG_PERIOD){};

    void alarm()
    {
      playMessage("---   ", 10);
      alarmStatus = true; 
      warningStaus = true; 
    }

    void warning()
    {
      playMessage(". ", 3);
      warningStaus = true; 
    }

    void disable()
    {
      cSimpleIndicator::disable();
      alarmStatus = false;
      warningStaus = false;  
    }

    bool getAlarmStatus()
    {
      return alarmStatus;  
    }
  
    bool getWarningStatus()
    {
      return warningStaus; 
    }

  private:
    bool alarmStatus = false;
    bool warningStaus = false;
};




class cAlarmSystem
{
  public:
    cAlarmSystem()
    {
      VibroSensor = new cVibroSensor(VIBRO_SENSOR, VIBRO_SENSOR_NEXT_TRIGGED_DELAY, VIBRO_SENSOR_LAST_TRIGGED_DELAY);
      MoveSensor = new cMoveSensor(MOVE_SENSOR, MOVE_SENSOR_NEXT_TRIGGED_DELAY, MOVE_SENSOR_LAST_TRIGGED_DELAY);
      RemoteAcces = new cRemoteAcces(RA_VT, RA_A, RA_B, RA_C, RA_D);  
      Buzzer = new cBuzzer(BUZZER);
      LedIndicator = new cSimpleIndicator(LED, LED_SHORT_PERIOD, LED_LONG_PERIOD);
    }

    ~cAlarmSystem()
    {
      delete(VibroSensor);
      delete(MoveSensor);
      delete(RemoteAcces);  
      delete(Buzzer);
      delete(LedIndicator);
    }
    
    void init()
    {
      RemoteAcces->init();
      VibroSensor->init();
      MoveSensor->init();
      Buzzer->init();
      LedIndicator->init();
      }

    void loop()
    {
      RemoteAcces->scan(); 
      VibroSensor->scan();
      MoveSensor->scan();

      analyze();
      
      Buzzer->loop();

      LedIndicator->loop();
    }

  private:
    void analyze()
    {
      // Remoute Acces
      byte raCommand = RemoteAcces->getCommand();

      switch(raCommand)
      {
        case COMMAND_ENABLE_ALL:
          #if defined(DEBUG)
            Serial.println("Enable Alarm");
          #endif
          VibroSensor->enableSensor();
          MoveSensor->enableSensor();
          Buzzer->playMessage(".",1);
          LedIndicator->playMessage("--  ", 0xFFFFFFFF);
          break;
        case COMMAND_DISABLE_ALL:
          #if defined(DEBUG)
            Serial.println("Disable Alarm");
          #endif  
          VibroSensor->disableSensor();
          MoveSensor->disableSensor();
          Buzzer->disable();
          Buzzer->playMessage(" .", 2);
          LedIndicator->disable();
          break;
        case COMMAND_ENABLE_VIBRO_SENSOR:
          #if defined(DEBUG)
            Serial.println("Enable vibro sensor"); 
          #endif  
          VibroSensor->enableSensor();   
          Buzzer->playMessage(".");
          LedIndicator->playMessage("-  ", 0xFFFFFFFF);
          break;
        case COMMAND_ENABLE_VOLUME_SENSOR:
          #if defined(DEBUG)
            Serial.println("Enable volume sensor"); 
          #endif  
          MoveSensor->enableSensor();
          Buzzer->playMessage(".");
          LedIndicator->playMessage("-  ", 0xFFFFFFFF);
          break;
      }
      RemoteAcces->resetCommand();

      // Sensors
      unsigned int currentTime = millis();
      // Vibro sensor
      if((VibroSensor->getAlarmStatus() or MoveSensor->getAlarmStatus()) and !Buzzer->getAlarmStatus())
        {
          #if defined(DEBUG)
            Serial.println("Alarm!");
          #endif  
          Buzzer->alarm();
          LedIndicator->playMessage(". ", 0xFFFFFFFF);
        }
      
      if((VibroSensor->getWarningStatus() or MoveSensor->getWarningStatus()) and !Buzzer->getWarningStatus() and !Buzzer->getAlarmStatus())
        {
          #if defined(DEBUG)
            Serial.println("Warning!");
          #endif  
          Buzzer->warning();
        }



      VibroSensor->resetAlarmStatus();
      VibroSensor->resetWarningStatus();
      MoveSensor->resetAlarmStatus();
      MoveSensor->resetWarningStatus();
    }
  
    cRemoteAcces *RemoteAcces;
    cVibroSensor *VibroSensor;
    cMoveSensor *MoveSensor;
    cBuzzer *Buzzer;
    cSimpleIndicator *LedIndicator;
};


cAlarmSystem AlarmSystem;

void setup() {
  // put your setup code here, to run once:

  #if defined(DEBUG)
    Serial.begin(9600);
  #endif  

  for(byte pin=0; pin<=13; pin++)
    pinMode(pin,  OUTPUT);

  AlarmSystem.init();

}

void loop() {
  // put your main code here, to run repeatedly:

  AlarmSystem.loop();
  
}
