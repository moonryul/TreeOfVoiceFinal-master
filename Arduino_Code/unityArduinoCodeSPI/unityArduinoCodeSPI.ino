#include <SPI.h>



// The built-in pin number of the slave, which is used within SPI.Begin()
const int ss1 = 37; // connect master pin 53 the first slave pin 53
const int ss2 = 49; // connect master pin 49 to the second slave pin 53
const int ss3 = 48; // connect master pin 48 to the third  slave pin 53
const int ss4 = 47; // connect master pin 47 to the fourth slave pin 53
//int ss5 = 46;

// A total num of LED = 186; each slave processes 40 LEDs
const int NumPixels1 = 40;
const int NumPixels2 = 44;
const int NumPixels3 = 50;
const int NumPixels4 = 52;

//const int m_totalNumOfPixels = NumPixels1 +  NumPixels2 + NumPixels3 +  NumPixels4 + 2;
const int m_totalNumOfPixels = NumPixels1 +  NumPixels2 + NumPixels3 +  NumPixels4;
// include the start 3 bytes and the end 3 bytes

const int m_totalByteSize = m_totalNumOfPixels * 3;


const int group1ByteSize = NumPixels1 * 3;
const int group2ByteSize = NumPixels2 * 3;
const int group3ByteSize = NumPixels3 * 3;
const int group4ByteSize = NumPixels4 * 3;


const int slaveSelectPin = 37;
const int sendSize = 558;	// number of bytes for one slice signal
byte tempBuffer[SERIAL_RX_BUFFER_SIZE];
byte sendBuffer[sendSize];
int index = sendSize - 1;

void setup() {
  pinMode(slaveSelectPin, OUTPUT);

  SPI.begin();
  SPI.setClockDivider(SPI_CLOCK_DIV16);

  Serial.begin(57600);
  //Serial1.begin(57600);

  Reset();
}

// ���۸� �ʱ�ȭ �ϰ�, ��� �����մ� ��ȣ�� ������.
void Reset() {
  memset(sendBuffer, 0, sendSize);
  digitalWrite(slaveSelectPin, LOW);
  SPI.transfer(sendBuffer, sendSize);
  digitalWrite(slaveSelectPin, HIGH);

}

void loop() {
  // ���� �ø��� ���ۿ� �����Ͱ� �ִ��� Ȯ���Ѵ�.
  int count = Serial.available();
  if (count == 0)
    return;

  // ���ۿ� �ִ¸�ŭ �о ���� ���ۿ� ��´�.
  Serial.readBytes(tempBuffer, count);
  for (int j = 0; j < count; ++j) {
    sendBuffer[index - j] = tempBuffer[j];
  }

  index -= count;


  if (index == -1) {
    
   // the buffer is full  
//   int idx = 558 / 3 - 1; // = 186-1
//
//    for (int i = 0; i < sendSize; i++) {
////
////      // print the received data from PC to the serial monitor via Serial1 of Mega
//      if ( i % 3 == 0) {
////
//       Serial1.print("LED=");
//        Serial1.print( idx--);
//        Serial1.print("    ");
//       Serial1.print(" r: ");
//      Serial1.print((byte)sendBuffer[i]);
//    }
//      else if ( i % 3 == 1) {
//        Serial1.print(" g: ");
//       Serial1.print((byte)sendBuffer[i]);
////
//      }
////
//     else {
//      Serial1.print(" b: ");
//      Serial1.println(sendBuffer[i]);
//
//     }
////
//   }// for

    sendLEDBytesToSlaves( sendBuffer, sendSize);

    //		digitalWrite(slaveSelectPin, LOW);
    //		SPI.transfer(sendBuffer, sendSize);
    //		digitalWrite(slaveSelectPin, HIGH);

    index = sendSize - 1;

  } //if (index == -1)

} // LOOP()

void  sendLEDBytesToSlaves( byte * totalReceiveBuffer, int totalByteSize )
{
  //
  //
  //  send the first group of data to the first slave:
  
  digitalWrite(ss1, LOW); // select the first SS line
  //
  //    // To send  a sequence of bytes to a slave arduiono via SPI, the sequence is marked by the start and the end
  //    // of the sequence with special bytes, m_startByte.
  //
 
  //SPI.transfer( m_startByte);
  SPI.transfer( (byte)255);
  //
  for (int i = 0; i < group1ByteSize; i++) {

    SPI.transfer( totalReceiveBuffer[i]);
  }
 
  digitalWrite(ss1, HIGH);

  // select the second SS Line
  digitalWrite(ss2, LOW); 
  
  // SPI.transfer( m_startByte);
  SPI.transfer( (byte)255);
 
  for (int i = 0; i < group2ByteSize; i++) {
    // SPI.transfer(LED2[i]);
    SPI.transfer( totalReceiveBuffer[ group1ByteSize + i] );
  }
  
  //
  digitalWrite(ss2, HIGH);
  //
  
  //  send the third group of data to the third slave:
 
  digitalWrite(ss3, LOW); // select the third SS line
  
  SPI.transfer( (byte)255);
  
  for (int i = 0; i < group3ByteSize; i++) {
    SPI.transfer( totalReceiveBuffer[group1ByteSize + group2ByteSize + i] );
  }
 
  digitalWrite(ss3, HIGH);

 // select the fourth SS line
  digitalWrite(ss4, LOW);   
  
  SPI.transfer( (byte)255);
  for (int i = 0; i < group4ByteSize; i++) {
    SPI.transfer( totalReceiveBuffer[group1ByteSize + group2ByteSize  + group3ByteSize +i ] );
    
  }
  //
  digitalWrite(ss4, HIGH);
 
  delay(50);
  //
  //
} //  sendLEDBytesToSlaves(totalReceiveBuffer,  m_totalByteSize )
