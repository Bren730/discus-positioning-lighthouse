import gab.opencv.*; 
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.MatOfPoint3f;
import org.opencv.core.MatOfDouble;
import org.opencv.core.Point;
import org.opencv.core.Point3;
import org.opencv.calib3d.Calib3d;

import java.util.ArrayList;
import java.util.List;

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
byte sensorCount = 100;

byte station;
byte skip;
byte rotor;
byte prevRotor;
byte data;

double[] xAngle = new double[sensorCount];
double[] yAngle = new double[sensorCount];
boolean[] sawSweep = new boolean[sensorCount];
long[] lastSweep = new long[sensorCount];

final double CPU_SPEED = 96.0; // CPU speed in MHz
final double SWEEP_CYCLE_TIME = 8333; // Sweep cycle time in us
final double SWEEP_CYCLE_CLOCK_CYCLES = SWEEP_CYCLE_TIME * CPU_SPEED; // Amount of CPU cycles per sweep

void setup() 
{
  size(1000, 1000);
  String portName = Serial.list()[1];

  myPort = new Serial(this, portName, 115200);
}

void draw()
{
  
  clear();

  for (int i = 0; i < sensorCount; i++) {

    if (xAngle[i] > 0 && millis() - lastSweep[i] < 50) {


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
        stroke(255, 127, 0);
      }
      if (i == 2) {
        stroke(255, 255, 0);
      }
      if (i == 3) {
        stroke(127, 255, 0);
      }
      if (i == 4) {
        stroke(0, 255, 0);
      }
      if (i == 5) {
        stroke(0, 255, 127);
      }
      if (i == 6) {
        stroke(0, 255, 255);
      }
      if (i == 7) {
        stroke(0, 127, 255);
      }
      if (i == 8) {
        stroke(0, 0, 255);
      }
      if (i == 9) {
        stroke(0, 0, 127);
      }

      strokeWeight(10);
    }
  }
}

void serialEvent(Serial p) {
  
  try {
  flagBuffer[1] = flagBuffer[0];
  flagBuffer[0] = p.read();
  inBuffer[bufferIndex] = flagBuffer[0];
  bufferIndex++;
  
  } catch (Exception e) {
    print(e);
  }

  if (flagBuffer[0] == 255 && flagBuffer[1] == 255) {
    //println("Startflag received");
    parseData();
    bufferIndex = 0;
  }
}

void parseData() {

  for (int i = 0; i < sensorCount; i++) {
    
    sawSweep[i] = false;
    
  }
  
  if (inBuffer.length > 0){
  //println("Parsing data");
  int sensorCnt = floor(bufferIndex / msgLength);
  println("Sensor count", sensorCnt);

  // First byte is the metadata byte holding station, skip, rotor and data values
  int meta = inBuffer[0];

  station = (byte)getBit(meta, 3);
  skip = (byte)getBit(meta, 2);
  prevRotor = rotor;
  rotor = (byte)getBit(meta, 1);
  data = (byte)getBit(meta, 0);

  println(station, skip, rotor, data);

  if (sensorCnt > 0) {

    for (int i = 0; i < sensorCnt; i++) {

      int readPos = (i * msgLength) + 1;

      byte sensorId = (byte) inBuffer[readPos];
      long deltaT = (long)(((inBuffer[readPos + 1] & 0xFF) << 24) | ((inBuffer[readPos + 2] & 0xFF) << 16) | ((inBuffer[readPos + 3] & 0xFF) << 8) | (inBuffer[readPos + 4] & 0xFF));

      double angle = getAngle(deltaT);
      
      sawSweep[sensorId] = true;
      lastSweep[sensorId] = millis();

      println(rotor, sensorId, deltaT, angle);

      if (rotor == 0) {

        xAngle[sensorId] = angle;
        
      } else {

        yAngle[sensorId] = angle;
        
      }
    }
  }
  
  solvePnp();
  
  }
}

double getAngle(long t) {

  double angle = 0;

  angle = ((double)t / SWEEP_CYCLE_CLOCK_CYCLES) * 180;

  return angle;
}

void solvePnp() {
  
  //List<Point3> pointsList = new ArrayList();
  //Point3 point1 = new Point3(-1, -1, 0);
  //Point3 point2 = new Point3(1, -1, 0);
  //Point3 point3 = new Point3(-1, 1, 0);
  //Point3 point4 = new Point3(1, 1, 0);
  
  //pointsList.add(point1);
  //pointsList.add(point2);
  //pointsList.add(point3);
  //pointsList.add(point4);
  
  //MatOfPoint3f objPoints = new MatOfPoint3f();
  //objPoints.fromList(pointsList);
  
  //List<Point> pointArray = new ArrayList();
  
  //Point point21 = new Point(1,2);
  //Point point22 = new Point(1,2);
  //Point point23 = new Point(1,2);
  //Point point24 = new Point(1,2);
  
  //MatOfPoint2f imgPoints = new MatOfPoint2f();
  //imgPoints.fromList(pointArray);
  //Mat cameraMatrix = new Mat();
  //MatOfDouble distortionCoefficients = new MatOfDouble();
  //Mat outputR = new Mat();
  //Mat outputT = new Mat();
  
  //Calib3d.solvePnP(objPoints, imgPoints, cameraMatrix, distortionCoefficients, outputR, outputT);
  
  
}

int getBit(int number, int position) {
  return (number >> position) & 1;
}

void shiftArray(int[] array) {

  System.arraycopy(array, 0, array, 1, array.length - 1);
}