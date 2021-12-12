#include <Servo.h>
#include <math.h>

/********************************************************/
const int ledPin = 13;
const byte buffSize = 40;
unsigned int inputBuffer[buffSize];
const char startMarker = '<';
const char endMarker = '>';
byte bytesRecvd = 0;
boolean readInProgress = false;
boolean newDataFromPC = false;
byte coordinates[2];
/********************************************************/

Servo turnTable;  // Servo corresponding to rotational movement of the base (turnTableAngle)
Servo servoOne;   // Servo corresponding to thetaOne position
Servo servoTwo;   // Servo corresponding to thetaTwo position
Servo servoThree; // Servo corresponding to thetaThree position (FREE HANGING MAGNET????)

double xCoord;                // Variable for x-distance in inches
double yCoord;                // Variable for y-distance in inches
double turnTableAngle;        // Variable for degree measure of base rotational movement
double thetaOne;              // Variable for degree measure of servoOne rotation
double thetaTwo;              // Variable for degree measure of servoTwo rotation
double thetaThree;            // Variable for degree measure of servoThree rotation
double shapeDistance;         // Distance of the shape measured from pivot point of robot (in inches)
const double armLength = 10.0;// Length of arms of the robot
double servoPosition;         // Variable to be used when moving servo motors
const int restOne = 90;       // Rest position of servoOne
const int restTwo = 110;      // Rest position of servoTwo
const int restTable = 90;     // Rest position of turnTable servo

/********************************************************/

void setup() 
{
  Serial.begin(9600);
  pinMode(ledPin, OUTPUT);

  // Assign pin 2 to magnet and TURN ON
  pinMode(2, OUTPUT);
  digitalWrite(2, HIGH);

  // Assign signal pins on Arduino to appropriate servos
  turnTable.attach(5);
  servoOne.attach(6);
  servoTwo.attach(10);
  servoThree.attach(11);

  // Upon start, write these positions to the servos (initial position of robot)
  turnTable.write(restTable);
  servoOne.write(restOne);
  servoTwo.write(restTwo);
  servoThree.write(10);   // This servo is connected to the magnet and will always be pointing down (10 degrees)

  // Wait 5 seconds
  delay(5000);
}

void loop() 
{
  getDataFromPC();

  // Turn magnet ON
  digitalWrite(2, HIGH);

  // While receiving new data from PC, run this condition
  if (newDataFromPC)
  {
    sendSuspendCmd();
    digitalWrite(ledPin, HIGH);

    /**********************************************************************/
    // Axis Coordinates Unit Conversions
    // NOTE: Conversions made using the reference of 1/2 inch coordinate grid squares on paper
    //
    // Equation for reference:
    //
    // (# of grids in X or Y direction) / 2 = [(X or Y coords. in pixels) * ( total # of 1/2 inch grid squares in X or Y direction)] / Picture frame height or width in pixels
    //
    // NOTE: # of grids in X or Y direction divided by 2 to get distance in inches in specified axis direction
    // source frame width = 211
    // source frame height = 158
    
    xCoord = (coordinates[0] * 22 ) / 211.0;
    xCoord = xCoord / 2; // xCoord in Inches
    yCoord = (coordinates[1] * 17 ) / 158.0;
    yCoord = yCoord / 2; // yCoord in Inches  

    /**********************************************************************/
    // Math calculations & Motor Movement:
    /*NOTE 1: Robot motion designed with 3 cases in mind based on location of the current shape being targeted. Those three cases are:
     * 1) Shape lies directly in FRONT of robot
     * 2) Shape lies to the RIGHT of robot
     * 3) Shape lies to the LEFT of robot
     * 
     * Each case includes its appropriate calculations to determine robot movement.
     * 
     * NOTE 2: Calculations also use a few CONSTANT MEASUREMENTS. Those are listed below:
     * 1) 16.5 inches = distance from pivot center of robot to far edge of paper
     * 2) 8.5 inches x 11 inches = size of paper
     * 3) 10 inches = robot arm segment length
     * 4) PI = 3.141592
    */
     
    if(xCoord == 5.5)      // Shape lies directly in FRONT of robot (5.5 inches = half paper distance from right to left)
    {
      shapeDistance = 16.5 - yCoord; // Distance of shape from robot pivot center
      turnTableAngle = 90;  // Corresponds to 90 degree rotation of base (physical servo position off by about 5 degrees)

      // Loop to move to turnTableAngle position
      for(servoPosition = 0; servoPosition <= turnTableAngle; servoPosition++)
      {
        turnTable.write(servoPosition);
        delay(50);
      }

      // Calculate thetaOne & thetaTwo
      thetaOne = acos( shapeDistance / (2.0 * armLength) );
      thetaTwo = asin( (shapeDistance * sin(thetaOne)) / armLength );

      // Convert from degrees to radians
      thetaOne = thetaOne * (180.0 / 3.141592);
      thetaTwo = thetaTwo * (180.0 / 3.141592);

      /**********************************************************************/
      // Move motors
      /**********************************************************************/
      for(servoPosition = 0; servoPosition <= thetaOne; servoPosition++)
      {
        servoOne.write(servoPosition);
        delay(50);
      }

      for(servoPosition = 0; servoPosition <= thetaTwo; servoPosition++)
      {
        servoTwo.write(servoPosition);
        delay(50);
      }
      
      delay(2000);  // Delay 2 seconds

      /**********************************************************************/
      // Return to rest position and rotate
      /**********************************************************************/
      for(servoPosition = thetaOne; servoPosition <= restOne; servoPosition++)
      {
        // Increment thetaOne to return to rest position
        servoOne.write(servoPosition);
        delay(50);
      }

      for(servoPosition = thetaTwo; servoPosition <= restTwo; servoPosition++)
      {
        //Increment thetaTwo to return to rest position
        servoTwo.write(servoPosition);
        delay(50);
      }

      delay(2000);

      if(coordinates[2] == 'S')
      {
        // Move to servo position 0 degrees
        for(servoPosition = turnTableAngle; servoPosition >= 0; servoPosition--)
        {
          turnTable.write(servoPosition);
          delay(50);
        }

        delay(2000);
        digitalWrite(2, LOW); // Turn magnet off (drop shape)
      }

      if(coordinates[2] == 'T')
      {
        for(servoPosition = turnTableAngle; servoPosition <= 180; servoPosition++)
        {
          turnTable.write(servoPosition);
          delay(50);
        }

        delay(2000);
        digitalWrite(2, LOW);
      }

      delay(2000);
    }



    if(xCoord < 5.5)  // Shape lies to the LEFT of the robot
    {
      shapeDistance = sqrt(sq(xCoord) + sq(16.5 - yCoord));
      double tempTurnTableAngle1 = atan( xCoord / (16.5 - yCoord) ) * (180.0 / 3.141592);  // Convert from radians to degrees
      turnTableAngle = tempTurnTableAngle1 + 90; // Add 90 degrees to rotation

      // Loop to move to turnTableAngle position
      for(servoPosition = 0; servoPosition <= turnTableAngle; servoPosition++)
      {
        turnTable.write(servoPosition);
        delay(50);
      }

      // Calculate thetaOne & thetaTwo
      thetaOne = acos( shapeDistance / (2.0 * armLength ) );
      thetaTwo = asin( (shapeDistance * sin(thetaOne)) / armLength );

      // Convert from radians to degrees
      thetaOne = thetaOne * (180.0 / 3.141592);
      thetaTwo = thetaTwo * (180.0 / 3.141592);

      /**********************************************************************/
      // Move motors
      /**********************************************************************/
      for(servoPosition = 0; servoPosition <= thetaOne; servoPosition++)
      {
        servoOne.write(servoPosition);
        delay(50);
      }

      for(servoPosition = 0; servoPosition <= thetaTwo; servoPosition++)
      {
        servoTwo.write(servoPosition);
        delay(50);
      }
      
      delay(2000);  // Delay 2 seconds

      /**********************************************************************/
      // Return to rest position and rotate
      /**********************************************************************/
      for(servoPosition = thetaOne; servoPosition <= restOne; servoPosition++)
      {
        // Increment thetaOne to return to rest position
        servoOne.write(servoPosition);
        delay(50);
      }

      for(servoPosition = thetaTwo; servoPosition <= restTwo; servoPosition++)
      {
        //Increment thetaTwo to return to rest position
        servoTwo.write(servoPosition);
        delay(50);
      }

      delay(2000);

      if(coordinates[2] == 'S')
      {
        // Move to servo position 0 degrees
        for(servoPosition = turnTableAngle; servoPosition >= 0; servoPosition--)
        {
          turnTable.write(servoPosition);
          delay(50);
        }

        delay(2000);
        digitalWrite(2, LOW); // Turn magnet off (drop shape)
      }

      if(coordinates[2] == 'T')
      {
        for(servoPosition = turnTableAngle; servoPosition <= 180; servoPosition++)
        {
          turnTable.write(servoPosition);
          delay(50);
        }

        delay(2000);
        digitalWrite(2, LOW);
      }
    }



    if(xCoord > 5.5)  // Shape lies to the RIGHT of the robot
    {
      shapeDistance = sqrt(sq(xCoord - 5.5) + sq(16.5 - yCoord));
      double tempTurnTableAngle2 = atan( (xCoord - 5.5) / (16.5 - yCoord)) * (180.0 / 3.141592);  // Convert from radians to degrees
      turnTableAngle = 90 - tempTurnTableAngle2;  // Subtract 90 degrees

      // Loop to move to turnTableAngle position
      for(servoPosition = 0; servoPosition <= turnTableAngle; servoPosition++)
      {
        turnTable.write(servoPosition);
        delay(50);
      }

      // Calculate thetaOne & thetaTwo
      thetaOne = acos( shapeDistance / (2.0 * armLength) );
      thetaTwo = asin( (shapeDistance * sin(thetaOne)) / armLength);

      // Convert from radians to degrees
      thetaOne = thetaOne * (180.0 / 3.141592);
      thetaTwo = thetaTwo * (180.0 / 3.141592);

      /**********************************************************************/
      // Move motors
      /**********************************************************************/
      for(servoPosition = 0; servoPosition <= thetaOne; servoPosition++)
      {
        servoOne.write(servoPosition);
        delay(50);
      }

      for(servoPosition = 0; servoPosition <= thetaTwo; servoPosition++)
      {
        servoTwo.write(servoPosition);
        delay(50);
      }
      
      delay(2000);  // Delay 2 seconds

      /**********************************************************************/
      // Return to rest position and rotate
      /**********************************************************************/
      for(servoPosition = thetaOne; servoPosition <= restOne; servoPosition++)
      {
        // Increment thetaOne to return to rest position
        servoOne.write(servoPosition);
        delay(50);
      }

      for(servoPosition = thetaTwo; servoPosition <= restTwo; servoPosition++)
      {
        //Increment thetaTwo to return to rest position
        servoTwo.write(servoPosition);
        delay(50);
      }

      delay(2000);

      if(coordinates[2] == 'S')
      {
        // Move to servo position 0 degrees
        for(servoPosition = turnTableAngle; servoPosition >= 0; servoPosition--)
        {
          turnTable.write(servoPosition);
          delay(50);
        }

        delay(2000);
        digitalWrite(2, LOW); // Turn magnet off (drop shape)
      }

      if(coordinates[2] == 'T')
      {
        for(servoPosition = turnTableAngle; servoPosition <= 180; servoPosition++)
        {
          turnTable.write(servoPosition);
          delay(50);
        }

        delay(2000);
        digitalWrite(2, LOW);
      }
    }


    delay(2000);
    digitalWrite(ledPin, LOW);
    sendEnableCmd();
    sendCoordinatesToPC();
    newDataFromPC = false;
  }

}

void sendSuspendCmd(){
  // Send the suspend-true command
  Serial.println("<S1>");
}

void sendEnableCmd(){
  // Send the suspend-false command
  Serial.println("<S0>");
}

void sendCoordinatesToPC(){
  // Send the point data back to the PC
  Serial.print("<P");
  //Serial.print(coordinates[0]);
  Serial.print(xCoord);
  Serial.print(",");
  //Serial.print(coordinates[1]);
  Serial.print(yCoord);
  Serial.print(",");
  Serial.print(thetaOne);
  Serial.print(",");
  Serial.print(thetaTwo);
  Serial.println(">");
}

// Alternative to the 'readBytes' function
void getDataFromPC(){
  // Receive data from PC and save it into inputBuffer
  if(Serial.available() > 0)
  {
    char x = Serial.read();
    // The order of these IF clauses is significant
    if(x == endMarker)
    {
      readInProgress = false;
      newDataFromPC = true;
      inputBuffer[bytesRecvd] = 0;
      coordinates[0] = inputBuffer[0];
      coordinates[1] = inputBuffer[1];
      coordinates[2] = inputBuffer[2];  // Identifier for the shapes
    }

    if(readInProgress)
    {
      inputBuffer[bytesRecvd] = x;
      bytesRecvd ++;

      if(bytesRecvd == buffSize)
      {
        bytesRecvd = buffSize - 1;
      }
    }

    if(x == startMarker)
    {
      bytesRecvd = 0;
      readInProgress = true;
    }
  }
}
