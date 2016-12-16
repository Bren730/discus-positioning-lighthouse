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

final int bufferSize = 10;
final int msgLength = 5;
int[] inBuffer = new int[bufferSize];
byte bufferIndex = 0;

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
      short int16 = (short)(((inBuffer[0] & 0xFF) << 8) | (inBuffer[1] & 0xFF));
      print(int16 + ", ");
      int meta = inBuffer[2];
      println(binary(meta));
      //print(getBit(meta, 1) + ", ");
      
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