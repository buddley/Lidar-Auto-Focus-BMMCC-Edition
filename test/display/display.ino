#include <Arduino.h>
#include <U8x8lib.h>
#include <SPI.h>

#define CS 10
#define DC 9
#define RST 8

U8X8_SSD1306_128X64_NONAME_4W_HW_SPI u8x8(10, 9, 8);

String line[4];
int line_count = 4;
int mode = 3;
int value;


#include<SoftwareSerial.h>//header file of software serial port 
//#include<Servo.h>

SoftwareSerial Serial1(2,3); //define software serial port name as Serial1 and define pin2 as RX and pin3 as TX 
//Servo servo;

/* For Arduinoboards with multiple serial ports like DUEboard, interpret above two pieces of code and 
directly use Serial1 serial port*/ 
int dist;
int raw_dist;//actual distance measurements of LiDAR
int raw_dist_prev;
int one_prev, two_prev, three_prev;
int pause_count = 0;

int strength;//signal strength of LiDAR 
int check;//save check value 
int i; 
int uart[9];//save data measured by LiDAR 
const int HEADER=0x59;//frame header of data package
int offset = -8;
int sens = 10;
int false_count = 0;

int pos = 1500;
int pos_old = 1500;


void setup(){
    Serial.begin(115200);
    Serial1.begin(115200);
    u8x8.begin();
}

void padding(int ln, bool big = false){
    int offset;
    char font[17];
    line[ln].toCharArray(font, 17);
    if (big){
        offset = (8 - strlen(font)) / 2;
    } else {
        offset = (16 - strlen(font)) / 2;
    }
    for (int i = 0; i < offset; i++){
        line[ln] = " " + line[ln] + " ";
    }
}
void refresh(){
    //u8x8.clear();
    //u8x8.setFont(u8x8_font_shylock_nbp_1x2_r);
    u8x8.setFont(u8x8_font_7x14_1x2_r);

    padding(0);
    u8x8.setCursor(0,0);
    u8x8.setInverseFont(1);
    u8x8.println(line[0]);

    padding(1);
    u8x8.setCursor(0,2);
    u8x8.setInverseFont(0);
    u8x8.println(line[1]);
    if (line_count == 4){
        padding(2);
        u8x8.setCursor(0,4);
        u8x8.println(line[2]);

        padding(3);
        u8x8.setCursor(0,6);
        u8x8.println(line[3]);
    } else {
        char big_font[9];
        u8x8.setFont(u8x8_font_inr21_2x4_r);
        
        padding(2, true);
        line[2].toCharArray(big_font, 9);
        u8x8.drawString(0,4,big_font);
    }
}

void display(){
    float sec = (float)millis() / 1000.0;
    value = value - (value - analogRead(A0)) /2;
    float volt = (float)value / 1023 * 5;

    float conv_in = (float)dist /2.54;
    float ft = conv_in /12.0;
    float in = conv_in - (int)ft * 12;
    Serial.print(ft);
    Serial.print("-");
    Serial.println(in);


    line[0] = "Mode " + String(mode) + " " + String(sec, 2);

    switch (mode)
    {
        case 5:
            line_count = 4;
            line[1] = "Mode 1 Voltage";
            line[2] = "Mode 2 Distance";
            line[3] = "Mode 3 All";
            break;
        case 1:
            line_count = 3;
            line[1] = "Voltage";
            line[2] = String(volt, 2) + "V";
            break;
        case 2:
            line_count = 3;
            line[1] = String(dist) + "cm";
            line[2] = String((int)ft) + "ft" + String((int)in) + "in";
            break;
        case 3:
            line_count = 4;
            line[1] = String(volt, 2) + "V";
            line[2] = String(dist) + "cm";
            //line[3] = String(volt, 2) + "V";
            line[3] = String((int)ft) + "ft" + String((int)in) + "in";
            break;

        default:
            break;
    }
    
    refresh();
}

void loop(){
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
                        if (abs(raw_dist_prev - raw_dist) < 100 || false_count > sens){
                            if (false_count > 0){
                                pause_count++;
                            }
                            if (pause_count > 3 || false_count == 0){
                                dist = dist - ((dist - raw_dist) / 2); //smoothing
                                false_count = 0;
                                pause_count = 0;
                                raw_dist_prev = raw_dist;
                                three_prev = two_prev;
                                two_prev = one_prev;
                                one_prev = dist;
                            }
                        } else {
                            false_count++;
                            dist = three_prev;
                        }
                    }
                    display();
                    Serial.print("dist = "); 
                    Serial.print(dist);//output measure distance value of LiDAR 
                    Serial.print('\t');
                    
                    Serial.print(false_count);
                    Serial.print('\t');

                    Serial.print(three_prev);
                    Serial.print('\t');
                    Serial.print(two_prev);
                    Serial.print('\t');
                    Serial.print(one_prev);
                    Serial.print('\t');
                    Serial.print(pause_count);
                    Serial.print('\t');

                    Serial.print("raw_dist = "); 
                    Serial.print(raw_dist);//output measure distance value of LiDAR 
                    Serial.print('\t'); 
                    
                    Serial.print("strength = "); 
                    Serial.print(strength);//output signal strength value 
                    Serial.println('\t'); 
/*
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
                    Serial.println(pos_old);*/
                    } 
                } 
            } 
        }
    
    
    if (Serial.available() > 0){
        long input = Serial.parseInt();
        if (input != 0){
            mode = input;
            u8x8.clear();
        }
    }
    

    //delay(500);
}