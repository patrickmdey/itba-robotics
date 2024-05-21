bool goingForward = true;
bool turning = false;
bool lightAttracted = false;
static unsigned int LIGHT_THRESHOLD = 380;

void print_value(char* string, char* value) {
  Serial.print(string);
  Serial.println(value);
}

class LightSensor {
  int sensorPin;
  int val;
  uint32_t senseInterval;
  unsigned long previousMillis;

public:
  LightSensor(int pin, uint32_t interval) {
    sensorPin = pin;
    senseInterval = interval;
    previousMillis = millis();
  }

  int Update() {
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis > senseInterval) {
      val = analogRead(sensorPin);
      previousMillis = currentMillis;
    }

    return val;
  }
};

enum MotorActionType {
  GO_FORWARD,
  GO_BACKWARD,
  DO_NOTHING
};

// Define a struct to hold an action and its parameters
struct MotorAction {
  MotorActionType type;
  int speed;  // Only used for GO_FORWARD, ignored for other actions

  // Constructor for DO_NOTHING
  public:
    MotorAction(MotorActionType actionType)
      : type(actionType), speed(0) {}

    // Constructor for GO_FORWARD or GO_BACKWARD with speed
  public:
    MotorAction(MotorActionType actionType, int actionSpeed)
      : type(actionType), speed(actionSpeed) {}
};

class Motor {
  int motorPinBack;
  int motorPinForward;
  int motorPinSpeed;
  MotorAction currentAction;

  public:
    Motor(int pinSpeed, int pinBack, int pinForward)
      : motorPinSpeed(pinSpeed),
        motorPinBack(pinBack),
        motorPinForward(pinForward),
        currentAction(MotorActionType::GO_FORWARD, 120) {
      analogWrite(motorPinSpeed, 120);
      pinMode(pinSpeed, OUTPUT);
      pinMode(pinBack, OUTPUT);
      pinMode(pinForward, OUTPUT);
      digitalWrite(pinBack, LOW);
      digitalWrite(pinForward, HIGH);
    }

  void Update(MotorAction newAction) {
    if (currentAction.type == newAction.type && currentAction.speed == newAction.speed) {
      return;
    }

    switch (newAction.type) {
      case MotorActionType::GO_FORWARD:
        digitalWrite(motorPinBack, LOW);
        digitalWrite(motorPinForward, HIGH);
        break;
      case MotorActionType::GO_BACKWARD:
        digitalWrite(motorPinBack, HIGH);
        digitalWrite(motorPinForward, LOW);
        break;
      case MotorActionType::DO_NOTHING:
        digitalWrite(motorPinBack, LOW);
        digitalWrite(motorPinForward, LOW);
        break;
    }

    analogWrite(motorPinSpeed, newAction.speed);
    currentAction = newAction;
  }
};

class DistanceSensor {
  int trigPin;
  int echoPin;
  int distance;
  uint32_t onTime;
  uint32_t offTime;
  bool isOn;
  unsigned long previousMillis;

  public:
    DistanceSensor(int trigPinNumber, int echoPinNumber, uint32_t onTimeVal, uint32_t offTimeVal)
      : trigPin(trigPinNumber),
        echoPin(echoPinNumber),
        onTime(onTimeVal),
        offTime(offTimeVal),
        isOn(false),
        previousMillis(millis()) {
      pinMode(trigPin, OUTPUT);  // Sets the trigPin as an Output
      pinMode(echoPin, INPUT);   // Sets the echoPin as an Input
    }

  int Update() {
    unsigned long currentMillis = millis();
    unsigned long timePassed = currentMillis - previousMillis;

    if (isOn && timePassed > onTime) {
      digitalWrite(trigPin, LOW);
      isOn = false;
      unsigned long duration = pulseIn(echoPin, HIGH);
      distance = duration * 0.017;
      previousMillis = currentMillis;
    } else if (!isOn && timePassed > offTime) {
      digitalWrite(trigPin, HIGH);
      isOn = true;
      previousMillis = currentMillis;
    }

    return distance;
  }
};

enum CarAction {
  INIT_STATE,
  GOING_FORWARD,
  GOING_BACKWARD,
  TURNING_RIGHT,
  TURNING_LEFT,
  WAITING_TO_GO_BACKWARD,
  DOING_NOTHING
};


class Car {
  Motor leftMotor;
  Motor rightMotor;
  LightSensor leftLightSensor;
  LightSensor rightLightSensor;
  DistanceSensor distanceSensor;

  bool lightAttracted;
  unsigned long previousMillis;
  unsigned long timeToWait;
  int distance;
  int leftLightSensorVal;
  int rightLightSensorVal;
  CarAction action;

  public:
    Car(bool isLightAttracted)
      : leftMotor(3, 4, 5),
        rightMotor(9, 8, 7),
        leftLightSensor(A0, 100),
        rightLightSensor(A1, 100),
        distanceSensor(10, 2, 10, 2),
        previousMillis(millis()),
        timeToWait(200),
        action(CarAction::INIT_STATE),
        lightAttracted(isLightAttracted) {
          Serial.println();
          Serial.println("NEW RUN");
          distance = 1000;
          leftLightSensorVal = 200;
          rightLightSensorVal = 200;
        }

  void Update() {
    unsigned int currentMillis = millis();
    int elapsed = currentMillis - previousMillis;
    bool action_can_wait = (action == CarAction::INIT_STATE ||action == CarAction::WAITING_TO_GO_BACKWARD || action == CarAction::GOING_BACKWARD);
    
    distance = distanceSensor.Update();
    leftLightSensorVal = leftLightSensor.Update();
    rightLightSensorVal = rightLightSensor.Update();

    if (action_can_wait && elapsed < timeToWait) {
      return;
    }

    //Serial.print("ACTION: ");Serial.print(action);Serial.print(" --- Distance: "); Serial.print(distance);Serial.print(" | LeftLight: "); Serial.print(leftLightSensorVal);Serial.print(" | RightLight: "); Serial.println(rightLightSensorVal);

    if (action == CarAction::GOING_FORWARD && distance < 15) {
      Serial.print("PARED!") ;
      turnOffMotors();
      timeToWait = 10;
      previousMillis = currentMillis;
      action = CarAction::WAITING_TO_GO_BACKWARD;
      Serial.println("WAITING TO GO BACKWARD");
      return;
    }

    switch (action) {

      case CarAction::INIT_STATE:
        action = CarAction::GOING_FORWARD;
        break;
      // The car is going forward and it's going to check if it needs to turn
      case CarAction::GOING_FORWARD:
        Serial.println("GOING FORWARD");
        if (leftLightSensorVal > LIGHT_THRESHOLD && rightLightSensorVal <= LIGHT_THRESHOLD) {
          // Only left > LIGHT_THRESHOLD
          action = lightAttracted ? goLeft() : goRight();
        } else if (leftLightSensorVal <= LIGHT_THRESHOLD && rightLightSensorVal > LIGHT_THRESHOLD) {
          // only right > LIGHT_THRESHOLD
          action = lightAttracted ? goRight() : goLeft();
        }
        break;
      
      // The car met a wall and its going to start going backwards for 2000ms
      case CarAction::WAITING_TO_GO_BACKWARD:
        timeToWait = 2000;
        action = goBackwards();
        break;

      case CarAction::GOING_BACKWARD:
        action = goForward();
        break;
      
      case CarAction::TURNING_LEFT:
      case CarAction::TURNING_RIGHT:
        // Going forward & distance >= 15
        if ((leftLightSensorVal <= LIGHT_THRESHOLD && rightLightSensorVal <= LIGHT_THRESHOLD) || (leftLightSensorVal > LIGHT_THRESHOLD && rightLightSensorVal > LIGHT_THRESHOLD)) {
          action = goForward();
        }
        return;
    }
  }

  CarAction goForward() {
    Serial.println("GOING FORWARD");
    MotorAction goForward(MotorActionType::GO_FORWARD, 120);
    leftMotor.Update(goForward);
    rightMotor.Update(goForward);
    return CarAction::GOING_FORWARD;
  }

  CarAction goRight() {
    Serial.println("GOING RIGHT");
    leftMotor.Update(MotorAction(MotorActionType::GO_FORWARD, 120));
    rightMotor.Update(MotorAction(MotorActionType::DO_NOTHING));
    return CarAction::TURNING_RIGHT;
  }

  CarAction goLeft() {
    Serial.println("GOING LEFT");
    leftMotor.Update(MotorAction(MotorActionType::DO_NOTHING));
    rightMotor.Update(MotorAction(MotorActionType::GO_FORWARD, 120));
    return CarAction::TURNING_LEFT;
  }

  CarAction turnOffMotors() {
    Serial.println("TURNING OFF MOTORS");
    leftMotor.Update(MotorAction(MotorActionType::DO_NOTHING));
    rightMotor.Update(MotorAction(MotorActionType::DO_NOTHING));
    return CarAction::DOING_NOTHING;
  }

  CarAction goBackwards(){
    Serial.println("WAITING TO GO BACKWARD");
    MotorAction goBackwards(MotorActionType::GO_BACKWARD, 120);
    leftMotor.Update(goBackwards);
    rightMotor.Update(goBackwards);
    return CarAction::GOING_BACKWARD;
  }

};


Car car(true);

void setup() {
  Serial.begin(9600);  // Starts the serial communication
}

void loop() {
  car.Update();
}
