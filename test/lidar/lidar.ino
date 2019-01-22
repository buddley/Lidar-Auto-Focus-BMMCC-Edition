/* this program is the interpretation routine of standard output protocol of TFmini product on Arduino. 
For details, refer to Product Specifications. 
For Arduino boards with only one serial port like UNO board, the function of software visual serial port is 
to be used. 
 */

#include<SoftwareSerial.h>//header file of software serial port 
#include<Servo.h>

SoftwareSerial Serial1(2,3); //define software serial port name as Serial1 and define pin2 as RX and pin3 as TX 
Servo servo;

/* For Arduinoboards with multiple serial ports like DUEboard, interpret above two pieces of code and 
directly use Serial1 serial port*/ 
int dist;
int raw_dist;//actual distance measurements of LiDAR 
int strength;//signal strength of LiDAR 
int check;//save check value 
int i; 
int uart[9];//save data measured by LiDAR 
const int HEADER=0x59;//frame header of data package
int offset = -8;
int sens = 300;
int false_count = 0;

int pos = 1500;
int pos_old = 1500;

void setup() 
    { 
        Serial.begin(115200);//set bit rate of serial port connecting Arduino with computer 
        Serial1.begin(115200);//set bit rate of serial port connecting LiDAR with Arduino 
        servo.attach(9);
    } 
void loop() 
    { 
    if (Serial1.available())//check if serial port has data input 
        { 
        if(Serial1.read()==HEADER)//assess data package frame header 0x59
            { 
            uart[0]=HEADER; 
            if(Serial1.read()==HEADER)//assess data package frame header 0x59 
                { 
                uart[1]=HEADER; 
                for(i=2;i<9;i++)//save data in array 
                    { 
                    uart[i]=Serial1.read(); 
                    } 
                check=uart[0]+uart[1]+uart[2]+uart[3]+uart[4]+uart[5]+uart[6]+uart[7]; 
                if(uart[8]==(check&0xff))//verify the received data as per protocol 
                    { 
                    raw_dist=uart[2]+uart[3]*256+offset;//calculate distance value 
                    strength=uart[4]+uart[5]*256;//calculate signal strength value
                    if (20 < strength) {
                        if (abs(dist - raw_dist) < 200 || false_count > sens){
                            dist = dist - ((dist - raw_dist) / 3); //smoothing
                            false_count = 0;
                        } else {
                            false_count++;
                        }
                    }
                    Serial.print("dist = "); 
                    Serial.print(dist);//output measure distance value of LiDAR 
                    Serial.print('\t');
                    
                    Serial.print(false_count);
                    Serial.print('\t');

                    Serial.print("raw_dist = "); 
                    Serial.print(raw_dist);//output measure distance value of LiDAR 
                    Serial.print('\t'); 
                    
                    Serial.print("strength = "); 
                    Serial.print(strength);//output signal strength value 
                    Serial.print('\t'); 

                    pos = map(int(dist), 10, 600, 600, 2400);
                    
                    if (abs(pos - pos_old) > 10) {
                        servo.writeMicroseconds(pos);
                        pos_old = pos;
                    } else {
                        servo.writeMicroseconds(pos_old);
                    }
                    
                    servo.writeMicroseconds(pos);
                    Serial.print("servo = ");
                    Serial.print(pos);
                    Serial.print("|");
                    Serial.println(pos_old);
                    } 
                } 
            } 
        }
    
    } 