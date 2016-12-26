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

import java.text.DecimalFormat;
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
int sensor = 0;
int sensorCount = 100;
int recordedSensorCount = 0;
int[] recordedSensorIds = new int[sensorCount];

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

double prevXSweep;

OpenCV opencv;

List<Point3> objPoints;
MatOfPoint3f recordedObjPoints;

MatOfPoint2f imgPoints;
MatOfPoint2f recordedImgPoints;

final int res = 1000;
final int halfRes = res / 2;

void setup() 
{
  size(1000, 1000);
  surface.setSize(res, res);

  // VERY IMPORTANT
  // Only execute OpenCV stuff after this 
  opencv = new OpenCV(this, res, res);

  objPoints = constructObjectPoints(10, 90);

  println(Serial.list());

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

      float xPos = halfRes + xRad * halfRes;
      float yPos = halfRes + yRad * halfRes;

      //println(xPos, yPos);
      //point(xPos, yPos);

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
  } 
  catch (Exception e) {
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

  if (inBuffer.length > 0) {
    //println("Parsing data");
    int sensorCnt = floor(bufferIndex / msgLength);
    recordedSensorCount = sensorCnt;
    //println("Sensor count", sensorCnt);

    // First byte is the metadata byte holding station, skip, rotor and data values
    int meta = inBuffer[0];

    station = (byte)getBit(meta, 3);
    skip = (byte)getBit(meta, 2);
    prevRotor = rotor;
    rotor = (byte)getBit(meta, 1);
    data = (byte)getBit(meta, 0);

    if (rotor == 0) {

      DecimalFormat df = new DecimalFormat("#.00");
      //println("FPS: " + df.format( (1000 / (millis() - prevXSweep))) );
      prevXSweep = millis();
    }

    //println(station, skip, rotor, data);

    try {

      if (sensorCnt > 0) {

        List<Point> sensorPoints = new ArrayList();

        for (int i = 0; i < sensorCnt; i++) {

          int readPos = (i * msgLength) + 1;

          byte sensorId = (byte) inBuffer[readPos];
          long deltaT = (long)(((inBuffer[readPos + 1] & 0xFF) << 24) | ((inBuffer[readPos + 2] & 0xFF) << 16) | ((inBuffer[readPos + 3] & 0xFF) << 8) | (inBuffer[readPos + 4] & 0xFF));

          recordedSensorIds[i] = sensorId;

          double angle = getAngle(deltaT);
          double ratio = getRatio(deltaT);

          sawSweep[sensorId] = true;
          lastSweep[sensorId] = millis();

          //println(rotor, sensorId, deltaT, angle);

          if (rotor == 0) {

            xAngle[sensorId] = angle;
          } else {

            yAngle[sensorId] = angle;
          }

          Point point = new Point();
          sensorPoints.add(point);
        }

        List<Point3> _objPointList = new ArrayList();
        MatOfPoint2f _imgPoints = new MatOfPoint2f();
        MatOfPoint3f _objPoints = new MatOfPoint3f();

        for (int i = 0; i < recordedSensorCount; i++) {


          _objPointList.add(objPoints.get(recordedSensorIds[i]));
        }

        _imgPoints.fromList(sensorPoints);

        _objPoints.fromList(_objPointList);

        solvePnp(_objPoints, _imgPoints);
      }
    } 
    catch (Exception e) {

      println(e);
    }
  }
}

double getAngle(long t) {

  double angle = 0;

  angle = ((double)t / SWEEP_CYCLE_CLOCK_CYCLES) * 180;

  return angle;
}


double getRatio(long t) {

  double ratio = 0;
  ratio = (double)t / SWEEP_CYCLE_CLOCK_CYCLES;
  return ratio;
}

void solvePnp(MatOfPoint3f _objPoints, MatOfPoint2f _imgPoints) {

  int fx = 1;
  int fy = 1;
  int cx = 1 / 2;
  int cy = 1 / 2;
  Mat cameraMatrix = new Mat(3, 3, 0);
  cameraMatrix.put(0, 0, fx);
  cameraMatrix.put(0, 2, cx);
  cameraMatrix.put(1, 1, fy);
  cameraMatrix.put(1, 2, cy);
  cameraMatrix.put(2, 2, 1);

  MatOfDouble distortionCoefficients = new MatOfDouble(1, 4, 0.0);
  MatOfDouble derp = new MatOfDouble();

  //distortionCoefficients.put(0, 0, 0);
  //distortionCoefficients.put(1, 0, 0);
  //distortionCoefficients.put(2, 0, 0);
  //distortionCoefficients.put(3, 0, 0);

  Mat outputR = new Mat();
  Mat outputT = new Mat();

  try {
    //Calib3d.solvePnP(objPoints, imgPoints, cameraMatrix, distortionCoefficients, outputR, outputT, Calib3d.CV_EPNP);
    Calib3d.solvePnP(_objPoints, _imgPoints, cameraMatrix, derp, outputR, outputT, false, Calib3d.CV_EPNP);
    println(outputT.get(0, 0));
    println(outputT.get(1, 0));
    println(outputT.get(2, 0));
  } 
  catch (Exception e) {

    println(e);
  }
}

List<Point3> constructObjectPoints(int corners, double radius) {

  List<Point3> pointsList = new ArrayList();

  float angle = 360.0 / (float)corners;

  for (int i = 0; i < corners; i++) {

    Point3 point = new Point3(cos(radians(angle * i)) * radius, sin(radians(angle * i)) * radius, 0);
    pointsList.add(point);
  }

  MatOfPoint3f objPoints = new MatOfPoint3f();
  objPoints.fromList(pointsList);

  println(pointsList);

  return pointsList;
}

int getBit(int number, int position) {
  return (number >> position) & 1;
}

void shiftArray(int[] array) {

  System.arraycopy(array, 0, array, 1, array.length - 1);
}