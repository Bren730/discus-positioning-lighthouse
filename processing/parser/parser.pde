import gab.opencv.*; 
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.MatOfPoint3f;
import org.opencv.core.MatOfDouble;
import org.opencv.core.Point;
import org.opencv.core.Point3;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.CvType;

import java.util.ArrayList;
import java.util.List;

import java.text.DecimalFormat;

import processing.serial.*;

import java.awt.datatransfer.*;
import java.awt.Toolkit;
import processing.opengl.*;
import saito.objloader.*;


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

float viveFov = 120;
double[] xAngle = new double[sensorCount];
double[] yAngle = new double[sensorCount];

double[] xRatio = new double[sensorCount];
double[] yRatio = new double[sensorCount];

double[] xScreen = new double[sensorCount];
double[] yScreen = new double[sensorCount];

boolean[] sawSweep = new boolean[sensorCount];
long[] lastSweep = new long[sensorCount];

float[] discusPosition = new float[3];
float[] prevDiscusPosition = new float[3];
float[] discusRotation = new float[3];
float[] prevDiscusRotation = new float[3];
float posScaleFactor = 15000;

final double CPU_SPEED = 96.0; // CPU speed in MHz
final double SWEEP_CYCLE_TIME = 8333; // Sweep cycle time in us
final double SWEEP_CYCLE_CLOCK_CYCLES = SWEEP_CYCLE_TIME * CPU_SPEED; // Amount of CPU cycles per sweep

double prevXSweep;
int prevMillis;

OpenCV opencv;

List<Point3> objPoints;
MatOfPoint3f recordedObjPoints;

MatOfPoint2f imgPoints;
MatOfPoint2f recordedImgPoints;

final int res = 2000;
final int halfRes = res / 2;

OBJModel model;

void setup() 
{
  size(1000, 1000, P3D);
  surface.setSize(res, res);

  // VERY IMPORTANT
  // Only execute OpenCV stuff after this 
  opencv = new OpenCV(this, res, res);

  // Calculate 3D sensor positions
  // Amount of sensors & radius in meters
  objPoints = constructObjectPoints(10, 0.08);

  println(Serial.list());

  String portName = Serial.list()[1];

  myPort = new Serial(this, portName, 115200);

  model = new OBJModel(this);
  model.load("Discus-men-220mm-2kg.obj");
  model.scale(6);
  
}

void draw()
{

  clear();
  color c = color(0, 0, 0, 0);
  background(c);

  for (int i = 0; i < sensorCount; i++) {

    if (xAngle[i] > 0 && millis() - lastSweep[i] < 50) {


      float xRad = radians(((float)xAngle[i] + 0));
      float yRad = radians(((float)yAngle[i] + 0));

      //float xPos = halfRes + ((float)xRatio[i]) * halfRes;
      //float yPos = halfRes + ((float)yRatio[i]) * halfRes;

      float xPos = getScreenX(radians((float)xAngle[i] - 90));
      float yPos = getScreenY(radians((float)yAngle[i] - 90));

      //float xPos = res - (float)xRatio[i];
      //float yPos = res - (float)yRatio[i];

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

  float[] distances = new float[3];
  distances[0] = discusPosition[0] - prevDiscusPosition[0];
  distances[1] = discusPosition[1] - prevDiscusPosition[1];
  distances[2] = discusPosition[2] - prevDiscusPosition[2];

  double distance = sqrt(pow(distances[0], 2) + pow(distances[1], 2) + pow(distances[2], 2));

  int millis = millis();
  double v = distance / (((double)millis - (double)prevMillis) / 1000.0);
  //println("Distance: " + distance);
  //println("millis: " + (millis - prevMillis));

  prevMillis = millis;

  textSize(32);
  text("Speed: " + String.valueOf(Math.round(v * 100.0) / 100.0) + " m/s", 100, 100);
  
  camera(0, 0, 0, 0, 0, 1, 0, 1, 0);
  perspective(radians(viveFov), 1, 100, 100000);

  // Set a new co-ordinate space
  pushMatrix();

  pointLight(255, 200, 200, 400, 400, 500);
  pointLight(200, 200, 255, -400, 400, 500);
  pointLight(255, 255, 255, 0, 0, -500);

  float x = discusPosition[0] * posScaleFactor;
  float y = discusPosition[1] * posScaleFactor;
  float z = discusPosition[2] * posScaleFactor;
  translate(x, y, z);

  rotateX(discusRotation[0]);
  rotateY(discusRotation[1]);
  rotateZ(discusRotation[2]);
  //translate(halfRes, halfRes, 0);

  pushMatrix();
  noStroke();
  model.draw();
  popMatrix();
  popMatrix();
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

          if (angle != 0) {

            sawSweep[sensorId] = true;
            lastSweep[sensorId] = millis();

            //println(rotor, sensorId, deltaT, angle);

            if (rotor == 0) {

              xAngle[sensorId] = angle;
              xRatio[sensorId] = ratio * res;
              xScreen[sensorId] = getScreenX(radians((float)angle - 90));
              
            } else {

              yAngle[sensorId] = angle;
              yRatio[sensorId] = ratio * res;
              yScreen[sensorId] = getScreenY(radians((float)angle - 90));
              
            }

            Point point = new Point(xScreen[sensorId], yScreen[sensorId]);
            sensorPoints.add(point);
          }
        }


        List<Point3> _objPointList = new ArrayList();
        MatOfPoint2f _imgPoints = new MatOfPoint2f();
        MatOfPoint3f _objPoints = new MatOfPoint3f();

        for (int i = 0; i < recordedSensorCount; i++) {

          _objPointList.add(objPoints.get(recordedSensorIds[i]));
        }

        //println(sensorPoints);
        //println(_objPointList);

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
  
  float fovLimit = (180 - viveFov) / 2.0;

  angle = ((double)t / SWEEP_CYCLE_CLOCK_CYCLES) * 180;

  if (angle < fovLimit || angle > 180 - fovLimit) {

    angle = 0;
  }

  return angle;
}

float getScreenX(float angle) {
  
  angle = -1 * angle;
  
  float xPos;
  float max = tan(((float)radians(viveFov) / 2.0));
  float perc = tan(angle) / max;
  //println(max, perc, angle);
  xPos = halfRes + (perc * halfRes);
  
  return xPos;
  
}

float getScreenY(float angle) {
  
  return getScreenX(angle);
  
}


double getRatio(long t) {

  double ratio = 0;
  ratio = (double)t / SWEEP_CYCLE_CLOCK_CYCLES;
  return ratio;
}

void solvePnp(MatOfPoint3f _objPoints, MatOfPoint2f _imgPoints) {

  double fx = res / (2 * tan(viveFov / 180.0 * PI / 2.0));
  double fy = res / (2 * tan(viveFov / 180.0 * PI / 2.0));
  double cx = res / 2.0;
  double cy = res / 2.0;
  Mat cameraMatrix = new Mat(3, 3, CvType.CV_64FC1);
  cameraMatrix.put(0, 0, fx);
  cameraMatrix.put(0, 2, cx);
  cameraMatrix.put(1, 1, fy);
  cameraMatrix.put(1, 2, cy);
  cameraMatrix.put(2, 2, 1);

  MatOfDouble distortionCoefficients = new MatOfDouble(4, 1, CvType.CV_64FC1);
  MatOfDouble distCoefficients = new MatOfDouble();

  //distortionCoefficients.put(0, 0, 0);
  //distortionCoefficients.put(1, 0, 0);
  //distortionCoefficients.put(2, 0, 0);
  //distortionCoefficients.put(3, 0, 0);

  Mat outputR = new Mat(3, 1, CvType.CV_64FC1);
  Mat outputT = new Mat(3, 1, CvType.CV_64FC1);

  try {
    //Calib3d.solvePnP(_objPoints, _imgPoints, cameraMatrix, distortionCoefficients, outputR, outputT, false, Calib3d.CV_EPNP);
    Calib3d.solvePnP(_objPoints, _imgPoints, cameraMatrix, distCoefficients, outputR, outputT);

    Mat rMat = new Mat();
    Calib3d.Rodrigues(outputR, rMat);

    //double[][] viewMatrix = {{rMat.get(0, 0)[0], rMat.get(0, 1)[0], rMat.get(0, 2)[0], outputT.get(0, 0)[0]}, 
    //  {rMat.get(1, 0)[0], rMat.get(1, 1)[0], rMat.get(1, 2)[0], outputT.get(1, 0)[0]}, 
    //  {rMat.get(2, 0)[0], rMat.get(2, 1)[0], rMat.get(2, 2)[0], outputT.get(2, 0)[0]}, 
    //  {0.0, 0.0, 0.0, 1.0}};

    //double[][] inverseMatrix = {{1.0, 1.0, 1.0, 1.0}, 
    //  {-1.0, -1.0, -1.0, -1.0},
    //  {-1.0, -1.0, -1.0, -1.0},
    //  {1.0, 1.0, 1.0, 1.0}};

    //  viewMatrix = viewMatrix * inverseMatrix;


    double[] x = new double[1];
    double[] y = new double[1];
    double[] z = new double[1];

    double[] xR = new double[1];
    double[] yR = new double[1];
    double[] zR = new double[1];

    outputT.get(0, 0, x);
    outputT.get(1, 0, y);
    outputT.get(2, 0, z);

    rMat.get(0, 0, xR);
    rMat.get(1, 0, yR);
    rMat.get(2, 0, zR);

    //println("X: " + String.valueOf(x[0]));
    //println("Y: " + String.valueOf(y[0]));
    //println("Z: " + String.valueOf(z[0]));

    //println("XR: " + String.valueOf(xR[0]));
    //println("YR: " + String.valueOf(yR[0]));
    //println("ZR: " + String.valueOf(zR[0]));

    prevDiscusPosition[0] = discusPosition[0];
    prevDiscusPosition[1] = discusPosition[1];
    prevDiscusPosition[2] = discusPosition[2];

    prevDiscusRotation[0] = discusRotation[0];
    prevDiscusRotation[1] = discusRotation[1];
    prevDiscusRotation[2] = discusRotation[2];

    discusPosition[0] = (float)x[0];
    discusPosition[1] = (float)y[0];
    discusPosition[2] = (float)z[0];
    
    println(x[0], y[0], z[0]);

    discusRotation[0] = (float)xR[0];
    discusRotation[1] = (float)zR[0];
    discusRotation[2] = (float)yR[0];
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

  //println(pointsList);

  return pointsList;
}

int getBit(int number, int position) {
  return (number >> position) & 1;
}

void shiftArray(int[] array) {

  System.arraycopy(array, 0, array, 1, array.length - 1);
}