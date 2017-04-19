#include <Arduino.h>

#include <Wire.h>
#include <EasyTransferI2C.h>

//create object
EasyTransferI2C ET;

struct RECEIVE_DATA_STRUCTURE{
  //put your variable definitions here for the data you want to receive
  //THIS MUST BE EXACTLY THE SAME ON THE OTHER ARDUINO
  double number;
};

//give a name to the group of data
RECEIVE_DATA_STRUCTURE mydata;

//define slave i2c address
#define I2C_SLAVE_ADDRESS 9

void receive(int numBytes) {}

bool initEverything();

void setup(){
  Serial.begin(115200);
  Serial.println("yay");
  Wire.begin(I2C_SLAVE_ADDRESS);
  //start the library, pass in the data details and the name of the serial port. Can be Serial, Serial1, Serial2, etc.
  ET.begin(details(mydata), &Wire);
  //define handler function on receiving data
  Wire.onReceive(receive);

}

void loop() {
  //check and see if a data packet has come in.
  if(ET.receiveData()){
    //this is how you access the variables. [name of the group].[variable name]
    //since we have data, we will blink it out.
    Serial.println(mydata.number);
  }
}





int main()
{
    if (!initEverything())
        Serial.println("fcuk");

    while (1)
    {
        //Serial.println("dostuff");
        if(ET.receiveData()){
          //this is how you access the variables. [name of the group].[variable name]
          //since we have data, we will blink it out.
          Serial.println(mydata.number);
        }
    }

    return 0;
}

bool initEverything()
{

    setup();
    return true;
}
