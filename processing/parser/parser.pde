/**
 * Simple Read
 * 
 * Read data from the serial port and change the color of a rectangle
 * when a switch connected to a Wiring or Arduino board is pressed and released.
 * This example works with the Wiring / Arduino program that follows below.
 */


import processing.serial.*;

Serial myPort;  // Create object from Serial class
int val;      // Data received from the serial port

float xAngle = 0;
float yAngle = 0;

void setup() 
{
  size(400, 400);
  // I know that the first port in the serial list on my mac
  // is always my  FTDI adaptor, so I open Serial.list()[0].
  // On Windows machines, this generally opens COM1.
  // Open whatever port is the one you're using.
  String portName = Serial.list()[1];

  myPort = new Serial(this, portName, 115200);
}

void draw()
{
  clear();
  String input = "";
  input = myPort.readStringUntil('\n');
  if (input != "" && input != null) {

    String[] inputSplit = input.split(" ");

    if (inputSplit.length > 1) {
      float val = float(inputSplit[1]);
      print(val + " ");
      print(inputSplit[0]);

      String type = inputSplit[0];
      if (type.equals("X")) {
        xAngle = val;
      } else {
        yAngle = val;
      }
      
      xAngle = 180 - xAngle;
      yAngle = 180 - yAngle;

      point((xAngle / 180) * 400, (yAngle / 180) * 400);
      stroke(255, 255, 255);
      strokeWeight(10);
    }
  }
}