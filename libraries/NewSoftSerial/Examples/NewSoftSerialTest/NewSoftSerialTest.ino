/*
Basic Usage

NewSoftSerial mySerial(ReceivePin, TransmitPin);
Create an instance of NewSoftSerial, using a pair of pins to receive and transmit. The receive pin must be one that supports pin change interrupts. You can create multiple NewSoftSerial ports, each on their own 2 pins, but due to the CPU usage requirements, it's only practical to use one port at a time.
mySerial.begin(baud);
Initialize the port to communicate at a specific baud rate.
mySerial.print(anything);
Print a number or text. This works the same as Serial.print().
mySerial.available();
Returns the number of bytes received, which can be read.
mySerial.read();
Reads the next byte from the port. If nothing has been received, -1 is returned.
mySerial.write(byte);
Transmit a single byte.
mySerial.flush();
Discard any received data that has not be read.
*/

#include <NewSoftSerial.h>

NewSoftSerial mySerial(17, 16);

void setup()  
{
  Serial.begin(57600);
  Serial.println("Goodnight moon!");

  // set the data rate for the NewSoftSerial port
  mySerial.begin(4800);
  mySerial.println("Hello, world?");
}

void loop()                     // run over and over again
{

  if (mySerial.available()) {
      Serial.print((char)mySerial.read());
  }
  if (Serial.available()) {
      mySerial.print((char)Serial.read());
  }
}
