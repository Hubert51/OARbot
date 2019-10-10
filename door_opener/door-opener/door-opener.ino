//project fourteen – infrared receiving tube
#include <IRremote.h>                   // insert IRremote.h library
// B1EFBA9D == 0
// DEE522C1 == 1
// 6F5974BD == 2
// 986FB325 == 3

#include <Servo.h>

Servo name_servo;

int servo_position = 0;
int RECV_PIN = 11;                           //define the pin of RECV_PIN 11
IRrecv irrecv(RECV_PIN);                 //define RECV_PIN as infrared receiver
decode_results results;                  //define variable results to save the result of infrared receiver
int count = -1;


void start_point(){
  for (servo_position=140; servo_position >= 60; servo_position -=1){
    name_servo.write(servo_position);
    delay(50);
  }
}

void end_point(){
  for (servo_position=60; servo_position <= 140; servo_position +=1){
    name_servo.write(servo_position);
    delay(50);
  }
}

  void setup(){
    name_servo.attach (9);
//    Serial.begin(9600);                        // configure the baud rate 9600
    irrecv.enableIRIn();                      //Boot infrared decoding
  }

void loop() {
    //test if receive decoding data and save it to variable results
    if (irrecv.decode(&results)) {
      if (results.value == 0xB1EFBA9D){
        count = 0;
      }
      if (results.value == 0xDEE522C1){
        count = 1;
      }
      if (results.value == 0x6F5974BD){
        count = 2;
      }
      if (results.value == 0x986FB325){
        count = 3;
      }
    // print data received in a hexadecimal
//    Serial.println(count);
    if (count == 1){
      end_point();
      count = -1;
    }

    if (count == 2) {
      start_point();
      count = -1;  
    }

    if (count == 3){
      end_point();
      delay(5000);
      start_point();
      delay(3000);  
    }
    

    irrecv.resume(); //wait for the next signal
  }
}
