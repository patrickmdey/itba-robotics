// Motor A connections
const int enA = 9;
const int rightBack = 8;
const int rightForward = 7;
// Motor B connections
const int enB = 3;
const int leftForward = 5;
const int leftBack = 4;

// distance sensor connections
const int trigPin = 10;
const int echoPin = 2;

// light sensor connections
const int LEFT_LDR = A0;
const int RIGHT_LDR = A1;

long duration;
int distance;
bool goingForward = true;
bool turning = false;

bool lightAttracted = true;

void setup() {
	// Set all the motor control pins to outputs
	pinMode(enA, OUTPUT);
	pinMode(enB, OUTPUT);
	pinMode(rightBack, OUTPUT);
	pinMode(rightForward, OUTPUT);
	pinMode(leftForward, OUTPUT);
	pinMode(leftBack, OUTPUT);
	
	// Turn off motors - Initial state
	digitalWrite(rightBack, LOW);
	digitalWrite(rightForward, HIGH);
	digitalWrite(leftBack, LOW);
  digitalWrite(leftForward, HIGH);

  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin, INPUT); // Sets the echoPin as an Input

  Serial.begin(9600); // Starts the serial communication
}

void loop() {
  // Clears the trigPin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);

  // Calculating the distance
  distance = duration * 0.034 / 2;

  analogWrite(enA, 120);
	analogWrite(enB, 120);

  Serial.print("Distance: ");
  Serial.println(distance);

  if (goingForward && distance < 15) {
    turnOffMotors();
    delay(200);

    goingForward = false;

    // go back
    goBackwards();

    delay(2000);
  } else if (!goingForward) {
    turnOffMotors();
    delay(200);

    goingForward = true;

    // Go forward
    goForward();
  } else {
    // Going forward & distance >= 15

    int left_ldr_val = analogRead(LEFT_LDR);
    int right_ldr_val = analogRead(RIGHT_LDR);
    Serial.print("LEFT LDR Value is: ");
    Serial.println(left_ldr_val);

    Serial.print("RIGHT LDR Value is: ");
    Serial.println(right_ldr_val);

    if (!turning) {
      if (left_ldr_val > 350 && right_ldr_val <= 350) {
        turning = true;
        // Only left > 350
        lightAttracted ? goRight() : goLeft();
      } else if (left_ldr_val <= 350 && right_ldr_val > 350) {
        turning = true;
        // only right > 350
        lightAttracted ? goLeft() : goRight();
      } 
    } else if ((left_ldr_val <= 350 && right_ldr_val <= 350) || (left_ldr_val > 350 && right_ldr_val > 350)) {
      turning = false;
      goForward();
    }
  }

  delay(100);

}

void turnOffMotors() {
  // Turn off all motors
  digitalWrite(leftForward, LOW);
	digitalWrite(leftBack, LOW);
  digitalWrite(rightForward, LOW);
	digitalWrite(rightBack, LOW);
}

void goRight() {
  digitalWrite(leftForward, LOW);
  digitalWrite(leftBack, LOW);
  digitalWrite(rightForward, HIGH);
  digitalWrite(rightBack, LOW);
}

void goLeft() {
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

