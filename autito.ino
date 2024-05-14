#include <Servo.h> 


// Motor A connections
// const int enA = 9;
// const int rightBack = 8;
// const int rightForward = 7;
// // Motor B connections
// const int enB = 3;
// const int leftForward = 5;
// const int leftBack = 4;

// // distance sensor connections
// const int trigPin = 10;
// const int echoPin = 2;

// // light sensor connections
// const int LEFT_LDR = A0;
// const int RIGHT_LDR = A1;

// long duration;
// int distance;
bool goingForward = true;
bool turning = false;
bool lightAttracted = false;

class LightSensor {
  int sensorPin;
  int val;
  uint32_t senseInterval;
  unsigned long previousMillis; 

  public: LightSensor(int pin, uint32_t interval) {
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
    int speed; // Only used for GO_FORWARD, ignored for other actions

    // Constructor for DO_NOTHING
    public: MotorAction(MotorActionType actionType) : type(actionType), speed(0) {}

    // Constructor for GO_FORWARD or GO_BACKWARD with speed
    public: MotorAction(MotorActionType actionType, int actionSpeed) : type(actionType), speed(actionSpeed) {}
};

class Motor {
  int motorPinBack;
  int motorPinForward;
  int motorPinSpeed;
  MotorAction currentAction;

  public: 
  Motor(int pinSpeed, int pinBack, int pinForward) 
  {
    motorPinSpeed = pinSpeed;
    motorPinBack = pinBack;
    motorPinForward = pinForward;

    MotorAction currentAction(MotorActionType::GO_FORWARD, 120);

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
      digitalWrite(motorPinBackward, LOW);
      digitalWrite(motorPinForward, HIGH);
      break;
    case MotorActionType::GO_BACKWARD:
      digitalWrite(motorPinBackward, HIGH);
      digitalWrite(motorPinForward, LOW);
      break;
    case MotorActionType::DO_NOTHING:
      digitalWrite(motorPinBackward, LOW);
      digitalWrite(motorPinForward, LOW);
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

  public: DistanceSensor(int trigPinNumber, int echoPinNumber, uint32_t onTimeVal, uint32_t offTimeVal) {
    trigPin = trigPinNumber;
    echoPin = echoPinNumber;
    onTime = onTimeVal;
    offTime = offTimeVal;
    isOn = false;
    previousMillis = millis();

    pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
    pinMode(echoPin, INPUT); // Sets the echoPin as an Input
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
  GOING_FORWARD,
  GOING_BACKWARD,
  TURNING_RIGHT,
  TURNING_LEFT,
  WAITING_TO_GO_BACKWARD,
  WAITING_TO_GO_FORWARD
};


class Car {
  Motor leftMotor(3, 4, 5);
  Motor rightMotor(9, 8, 7);
  LightSensor leftLightSensor(A0, 100);
  LightSensor rightLightSensor(A1, 100);
  DistanceSensor distanceSensor(10, 2, 10, 2);

  bool lightAttracted;
  unsigned long previousMillis;
  unsigned long timeToWait;
  int distance;
  int leftLightSensorVal;
  int rightLightSensorVal;
  CarAction action;

  public: Car(bool isLightAttracted) {
    previousMillis = millis();
    timeToWait = 0;
    action = CarAction.GOING_FORWARD;
    lightAttracted = isLightAttracted;
  }

  void Update() {
    currentMillis = millis();
    elapsed = currentMillis - previousMillis;
    if (elapsed < timeToWait) {
      return; 
    }

    distance = distanceSensor.Update();
    leftLightSensorVal = leftLightSensor.Update();
    rightLightSensorVal = rightLightSensor.Update();

    if (action == CarActionType.GOING_BACKWARD && distance < 15) {
      turnOffMotors();
      timeToWait = 200;
      previousMillis = currentMillis;
      action = CarAction.WAITING_TO_GO_BACKWARD;
      return;
    }

    switch (action) {
    case CarActionType.WAITING_TO_GO_BACKWARD:
      action = CarAction.GOING_BACKWARD;
      MotorAction goBackwards(MotorActionType.GO_BACKWARD, 120);
      leftMotor.Update(goBackwards)
      rightMotor.Update(goBackwards);

      timeToWait = 2000;
      break;
    case CarActionType.WAITING_TO_GO_FORWARD:
      goForward()

      break;
    case CarActionType.GOING_FORWARD:
      if (leftLightSensorVal > 350 && rightLightSensorVal <= 350) {
        turning = true;
        // Only left > 350
        lightAttracted ? goLeft() : goRight();
      } else if (leftLightSensorVal <= 350 && rightLightSensorVal > 350) {
        turning = true;
        // only right > 350
        lightAttracted ? goRight() : goLeft();
      } 
      break;
    case CarActionType.TURNING_LEFT:
    case CarActionType.TURNING_RIGHT:
      // Going forward & distance >= 15
      if ((leftLightSensorVal <= 350 && rightLightSensorVal <= 350) || (leftLightSensorVal > 350 && rightLightSensorVal > 350)) {
        turning = false;
        goForward();
      }
    }

    void goForward() {
      action = CarAction.GOING_FORWARD;
      MotorAction goForward(MotorActionType.GO_FORWARD, 120);
      leftMotor.Update(goForward)
      rightMotor.Update(goForward);
    }

    void goRight() {
      action = CarAction.TURNING_RIGHT;
      leftMotor.Update(MotorAction(MotorActionType.GO_FORWARD, 120))
      rightMotor.Update(MotorAction(MotorActionType.DO_NOTHING));
    }

    void goLeft() {
      action = CarAction.TURNING_LEFT;
      leftMotor.Update(MotorAction(MotorActionType.DO_NOTHING));
      rightMotor.Update(MotorAction(MotorActionType.GO_FORWARD, 120))
    }

  }

  void turnOffMotors() {
    // Turn off all motors
    // digitalWrite(leftForward, LOW);
    // digitalWrite(leftBack, LOW);
    // digitalWrite(rightForward, LOW);
    // digitalWrite(rightBack, LOW);
    leftMotor.Update(MotorAction(MotorActionType.DO_NOTHING))
    rightMotor.Update(MotorAction(MotorActionType.DO_NOTHING))
  }
};

// Motor leftMotor(3, 4, 5);
// Motor rightMotor(9, 8, 7);

// DistanceSensor distSensor(10, 2, 10, 2);

// LightSensor leftLightSensor(A0, 100);
// LightSensor rightLightSensor(A1, 100);

Car car(true);

void setup() {
	// Set all the motor control pins to outputs
	// pinMode(enA, OUTPUT);
	// pinMode(enB, OUTPUT);
	// pinMode(rightBack, OUTPUT);
	// pinMode(rightForward, OUTPUT);
	// pinMode(leftForward, OUTPUT);
	// pinMode(leftBack, OUTPUT);
	
	// // Turn off motors - Initial state
	// digitalWrite(rightBack, LOW);
	// digitalWrite(rightForward, HIGH);
	// digitalWrite(leftBack, LOW);
  // digitalWrite(leftForward, HIGH);

  // pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  // pinMode(echoPin, INPUT); // Sets the echoPin as an Input

  Serial.begin(9600); // Starts the serial communication
}

void loop() {
  car.Update()

  //  if (goingForward && distance < 15) {
  //   turnOffMotors();
  //   // delay(200);

  //   goingForward = false;

  //   // go back
  //   goBackwards();

  //   delay(1000);
  // } else if (!goingForward) {
  //   turnOffMotors();
  //   delay(200);

  //   goingForward = true;

  //   // Go forward
  //   goForward();
  // }


  // Clears the trigPin
  // digitalWrite(trigPin, LOW);
  // delayMicroseconds(2);
  // // Sets the trigPin on HIGH state for 10 micro seconds
  // digitalWrite(trigPin, HIGH);
  // delayMicroseconds(10);
  // digitalWrite(trigPin, LOW);
  // // Reads the echoPin, returns the sound wave travel time in microseconds
  // duration = pulseIn(echoPin, HIGH);

  // // Calculating the distance
  // distance = duration * 0.034 / 2;

  // analogWrite(enA, 120);
	// analogWrite(enB, 120);

  // Serial.print("Distance: ");
  // Serial.println(distance);

  // if (goingForward && distance < 15) {
  //   turnOffMotors();
  //   delay(200);

  //   goingForward = false;

  //   // go back
  //   goBackwards();

  //   delay(1000);
  // } else if (!goingForward) {
  //   turnOffMotors();
  //   delay(200);

  //   goingForward = true;

  //   // Go forward
  //   goForward();
  // } else {
  //   // Going forward & distance >= 15

  //   int left_ldr_val = analogRead(LEFT_LDR);
  //   int right_ldr_val = analogRead(RIGHT_LDR);
  //   Serial.print("LEFT LDR Value is: ");
  //   Serial.println(left_ldr_val);

  //   Serial.print("RIGHT LDR Value is: ");
  //   Serial.println(right_ldr_val);

  //   if (!turning) {
  //     if (left_ldr_val > 350 && right_ldr_val <= 350) {
  //       turning = true;
  //       // Only left > 350
  //       lightAttracted ? goLeft() : goRight();
  //     } else if (left_ldr_val <= 350 && right_ldr_val > 350) {
  //       turning = true;
  //       // only right > 350
  //       lightAttracted ? goRight() : goLeft();
  //     } 
  //   } else if ((left_ldr_val <= 350 && right_ldr_val <= 350) || (left_ldr_val > 350 && right_ldr_val > 350)) {
  //     turning = false;
  //     goForward();
  //   }
  // }

  // delay(100);

}



void turnOffMotors() {
  // Turn off all motors
  // digitalWrite(leftForward, LOW);
	// digitalWrite(leftBack, LOW);
  // digitalWrite(rightForward, LOW);
	// digitalWrite(rightBack, LOW);
  leftMotor.Update(MotorAction(MotorActionType.DO_NOTHING))
  rightMotor.Update(MotorAction(MotorActionType.DO_NOTHING))
}

void goLeft() {
  digitalWrite(leftForward, LOW);
  digitalWrite(leftBack, LOW);
  digitalWrite(rightForward, HIGH);
  digitalWrite(rightBack, LOW);
}

void goRight() {
  digitalWrite(leftForward, HIGH);
  digitalWrite(leftBack, LOW);
  digitalWrite(rightForward, LOW);
  digitalWrite(rightBack, LOW);
}

void goForward() {
  digitalWrite(leftForward, HIGH);
  digitalWrite(leftBack, LOW);
  digitalWrite(rightForward, HIGH);
  digitalWrite(rightBack, LOW);
}

void goBackwards() {
  digitalWrite(leftForward, LOW);
  digitalWrite(leftBack, HIGH);
  digitalWrite(rightForward, LOW);
  digitalWrite(rightBack, HIGH);
}

