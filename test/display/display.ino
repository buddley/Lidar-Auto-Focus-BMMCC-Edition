#include <Arduino.h>
#include <U8x8lib.h>
#include <SPI.h>

#define CS 10
#define DC 9
#define RST 8

U8X8_SSD1306_128X64_NONAME_4W_HW_SPI u8x8(10, 9, 8);

String line[4];
int line_count = 4;
int mode = 5;
int value;

void setup(){
    Serial.begin(115200);
    u8x8.begin();
    u8x8.setFont(u8x8_font_shylock_nbp_1x2_r);
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
        
        padding(2, true);
        line[2].toCharArray(big_font, 9);
        u8x8.draw2x2String(0,4,big_font);
    }
}

void loop(){
    if (Serial.available() > 0){
        long input = Serial.parseInt();
        if (input != 0){
            mode = input;
            u8x8.clear();
        }
    }

    float sec = (float)millis() / 1000.0;
    //float volt = (float)value / 1023 * 5;
    value = value - (value - analogRead(A0)) /2;
    float conv_in = (float)value /2.54;
    float ft = conv_in /12.0;
    float in = conv_in - (int)ft * 12;

    line[0] = "Mode " + String(mode) + " " + String(sec, 2);

    switch (mode)
    {
        case 5:
            line_count = 4;
            line[1] = "Mode 1 Raw data";
            line[2] = "Mode 2 Voltage";
            line[3] = "Mode 3 Both";
            break;
        case 1:
            line_count = 3;
            line[1] = "Raw ADC data";
            line[2] = String(value);
            break;
        case 2:
            line_count = 3;
            line[1] = "Voltage  Max=5V";
            //line[2] = String(volt, 2) + "V";
            line[2] = String(ft, 0) + "ft" + String(in, 0) + "in";
            break;
        case 3:
            line_count = 4;
            line[1] = "Raw and Voltage";
            line[2] = String(value);
            //line[3] = String(volt, 2) + "V";
            line[3] = String(ft, 0) + "ft" + String(in, 0) + "in";
            break;

        default:
            break;
    }
    
    refresh();

    //delay(500);
}