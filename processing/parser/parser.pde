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

final int msgLength = 5;
final int bufferSize = 32 * msgLength;
int[] inBuffer = new int[bufferSize];
int[] flagBuffer = new int[2];
byte bufferIndex = 0;
byte sensor = 0;
byte sensorCount = 4;

byte station;
byte skip;
byte rotor;
byte data;

double[] xAngle = new double[sensorCount];
double[] yAngle = new double[sensorCount];

final double CPU_SPEED = 96.0; // CPU speed in MHz
final double SWEEP_CYCLE_TIME = 8333; // Sweep cycle time in us
final double SWEEP_CYCLE_CLOCK_CYCLES = SWEEP_CYCLE_TIME * CPU_SPEED; // Amount of CPU cycles per sweep

void setup() 
{
  size(800, 800);
  String portName = Serial.list()[1];

  myPort = new Serial(this, portName, 115200);
}

void draw()
{
  clear();

  for (int i = 0; i < sensorCount; i++) {

    if (xAngle[i] > 0) {


      float xRad = radians(((float)xAngle[i] - 90));
      float yRad = -1 * radians(((float)yAngle[i] - 90));

      float xPos = 400 + xRad * 400;
      float yPos = 400 + yRad * 400;

      //println(xPos, yPos);
      point(xPos, yPos);

      if (i == 0) {
        stroke(255, 0, 0);
      }
      if (i == 1) {
        stroke(255, 255, 0);
      }
      if (i == 2) {
        stroke(0, 255, 0);
      }
      if (i == 3) {
        stroke(0, 255, 255);
      }

      strokeWeight(10);
    }
  }
}

void serialEvent(Serial p) {

  flagBuffer[1] = flagBuffer[0];
  flagBuffer[0] = p.read();
  inBuffer[bufferIndex] = flagBuffer[0];
  bufferIndex++;

  if (flagBuffer[0] == 255 && flagBuffer[1] == 255) {
    //println("Startflag received");
    parseData();
    bufferIndex = 0;
  }
}

void parseData() {

  //println("Parsing data");
  int sensorCnt = floor(bufferIndex / msgLength);
  //println("Sensor count", sensorCnt);

  // First byte is the metadata byte holding station, skip, rotor and data values
  int meta = inBuffer[0];

  station = (byte)getBit(meta, 3);
  skip = (byte)getBit(meta, 2);
  rotor = (byte)getBit(meta, 1);
  data = (byte)getBit(meta, 0);

  //println(station, skip, rotor, data);

  if (sensorCnt > 0) {

    for (int i = 0; i < sensorCnt; i++) {

      int readPos = (i * msgLength) + 1;

      byte sensorId = (byte) inBuffer[readPos];
      long deltaT = (long)(((inBuffer[readPos + 1] & 0xFF) << 24) | ((inBuffer[readPos + 2] & 0xFF) << 16) | ((inBuffer[readPos + 3] & 0xFF) << 8) | (inBuffer[readPos + 4] & 0xFF));

      double angle = getAngle(deltaT);

      println(rotor, sensorId, deltaT, angle);

      if (rotor == 0) {

        xAngle[sensorId] = angle;
        
      } else {

        yAngle[sensorId] = angle;
        
      }
    }
  }
}

double getAngle(long t) {

  double angle = 0;

  angle = ((double)t / SWEEP_CYCLE_CLOCK_CYCLES) * 180;

  return angle;
}

int getBit(int number, int position) {
  return (number >> position) & 1;
}

void shiftArray(int[] array) {

  System.arraycopy(array, 0, array, 1, array.length - 1);
}