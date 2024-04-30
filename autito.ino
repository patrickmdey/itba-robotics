// Motor A connections
int enA = 9;
int rightBack = 8;
int rightForward = 7;
// Motor B connections
int enB = 3;
int leftForward = 5;
int leftBack = 4;

// distance sensor connections
const int trigPin = 10;
const int echoPin = 2;

long duration;
int distance;
bool goingForward = true;

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
	digitalWrite(rightForward, LOW);
	digitalWrite(leftForward, LOW);
	digitalWrite(leftBack, LOW);

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

  if (distance < 15 && goingForward) {
    turnOffMotors();
    delay(200);

    goingForward = false;

    // go back
    digitalWrite(leftForward, LOW);
    digitalWrite(leftBack, HIGH);
    digitalWrite(rightForward, LOW);
    digitalWrite(rightBack, HIGH);

    delay(2000);
  } else if (!goingForward) {
    turnOffMotors();
    delay(200);

    goingForward = true;

    // Go forward
    digitalWrite(leftForward, HIGH);
    digitalWrite(leftBack, LOW);
    digitalWrite(rightForward, HIGH);
    digitalWrite(rightBack, LOW);
  }
}

void turnOffMotors() {
  // Turn off all motors
  digitalWrite(leftForward, LOW);
	digitalWrite(leftBack, LOW);
  digitalWrite(rightForward, LOW);
	digitalWrite(rightBack, LOW);
}

