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

double xAngle = 0;
double yAngle = 0;

final int bufferSize = 10;
final int msgLength = 6;
int[] inBuffer = new int[bufferSize];
byte bufferIndex = 0;

void setup() 
{
  size(800, 800);
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
  float xRad = radians(((float)xAngle - 90));
  float yRad = -1 * radians(((float)yAngle - 90));
  
  float xPos = 400 + xRad * 400;
  float yPos = 400 + yRad * 400;
  
  println(xPos, yPos);
  point(xPos, yPos);
  stroke(255, 255, 255);
  strokeWeight(10);
}

void serialEvent(Serial p) {
  
  //print(p.read()+ ", ");
  
  inBuffer[bufferIndex] = p.read();
  
  
  // Start flag received. Previous transmission should be complete
  if(bufferIndex > 0 && inBuffer[bufferIndex] == 255 && inBuffer[bufferIndex - 1] == 255) {
    
    //println(inBuffer);
    
    if (bufferIndex == msgLength -1) {
      
      // We got the expected amount of bytes
      // Construct angle data
      // The first two bytes are the deltaT
      short deltaT = (short)(((inBuffer[0] & 0xFF) << 8) | (inBuffer[1] & 0xFF));
      
      // The next byte is the sensor id
      //print(inBuffer[2] + ", ");
      
      // Finally, the meta byte contains data about the station, skip, rotor and data
      int meta = inBuffer[3];
      int station = getBit(meta, 3);
      int skip = getBit(meta, 2);
      int rotor = getBit(meta, 1);
      int data = getBit(meta, 0);
      
      if (rotor == 0) {
        
        xAngle = ((double)deltaT / 8333.0) * 180.0;
        //println("X: "+ xAngle);
        
      } else {
        
        yAngle = ((double)deltaT / 8333.0) * 180.0;
        //println("Y: "+ yAngle);
        
      }
      
    }
    
    //Start flags found. Reset buffer and set index to 0
    inBuffer = new int[bufferSize];
    bufferIndex = 0;
    
  } else {
    
    bufferIndex++;
    
  }
  
  if (bufferIndex >= bufferSize) {
    
   bufferIndex = 0; 
    
  }
  
}

int getBit(int number, int position) {
  return (number >> position) & 1;
}