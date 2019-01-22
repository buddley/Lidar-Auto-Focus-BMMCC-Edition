/*
* Blackmagic Micro Cinema Camera Controller + LIDAR Autofocus 
* v0.2
*
* 2019-01-01 by buddley
* built for Teensy LC board
*/

#include <BMC_SBUS.h>
#include <EEPROM.h>
#include <Wire.h>
#include <Servo.h>
#include <ky-040.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>
#include <spline.h>

/* pin */
#define p_run 2
#define p_enca 3
#define p_encb 4
#define p_push 5
#define p_batt A0
#define p_afon A1
#define p_focus A2
#define p_servo A3
#define p_sda A4
#define p_scl A5

/* Sbus def */
#define sbus_mid 1023
#define sbus_low 352
#define sbus_high 1696

BMC_SBUS sbus;

/* oled */
String line[3];
int disp_lines = 3;

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET 13

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

/* encoder */
ky040 encoder(p_enca, p_encb, p_push);

/* lidar */
bool lidar_conn = false;
int raw_dist, strength, checksum;
int uart[9];
const int leading = 0x59;
int dist = 200;

/* lens - hardcoded */
#define lens_count 2

int lens[lens_count][3] = {
    {24, 85, 35},
    {50, 50, 14}
    };
/* lens scale and servo position
* actual measurement needs 9 positions
* first and last values are not used - only for spline calculation*/
int scale[lens_count][11] = {
    {29,30,50,70,100,150,200,500,1000,2500,2501},
    {29,30,50,70,100,150,200,500,1000,2500,2501}
    }; //the second last one = inf
int pos[lens_count][11] = {
    {999,1000,1125,1250,1375,1500,1625,1750,1875,2000,2001},
    {999,1000,1125,1250,1375,1500,1625,1750,1875,2000,2001}
    };

int sens = 5; // 1-10
bool af = false;

Spline af_curve;
Servo servo;

/* cam control */
float voltage = 0;
unsigned long volt_check = millis();//voltage check interval
int sbus_value[18] = {sbus_mid, sbus_mid, sbus_mid, 0,0,0,sbus_mid,0,0,0,0,0,0,0,0,0,0,0}; //0-2047 (11bit)

/* menu */
bool lock = true;
bool edit = false;
int curr_menu = 0;
int curr_fps = 0;
int curr_codec = 0;
int curr_lens = 0;

#define menu_count 8
String menu[] = {"ISO", "ANGLE", "WB", "FPS", "CODEC", "LENS", "SENS", "TUNE"};
#define fps_count 8
String fps[] = {"23.98", "24.00", "25.00", "29.97", "30.00", "50.00", "59.94", "60.00"};
#define codec_count 8
String codec[] = {"RAW", "R3:1", "422HQ", "422", "422LT", "PROXY"};

/* FUNCTIONS */

// control related
void step(int dir, int channel){ // mid - high or low - mid
    channel++; //0-base to 1-base

    sbus.Servo(channel,sbus_mid);
    sbus.Update();
    sbus.Send();
    delay(5);

    if (dir > 0){
        sbus.Servo(channel,sbus_high);
        sbus.Update();
        sbus.Send();
    } else if (dir < 0){
        sbus.Servo(channel,sbus_low);
        sbus.Update();
        sbus.Send();
    }
    delay(5);

    sbus.Servo(channel,sbus_mid);
    sbus.Update();
    sbus.Send();
}
void set_value(){
    switch (curr_menu){
        case 3:
            sbus.Servo((curr_menu + 1), int(sbus_low + (sbus_high - sbus_low) / fps_count * curr_fps));
        case 4:
            sbus.Servo((curr_menu + 1), int(sbus_low + (sbus_high - sbus_low) / codec_count * curr_codec));
    }
    sbus.Update();
    sbus.Send();
}

// display related
void welcome(){
    //init + some welcome texts
    display.display();
    display.clearDisplay();

    //display.setFont(&FreeSans18pt7b);
    display.setTextSize(2); //scale
    display.setTextColor(WHITE);
    display.setCursor(0,0);
    display.println(F("BMMCC CTRL"));
    display.println(F("AF BUDDY"));
    display.println(F("V0.2"));

    display.display();
}
void lock_screen(){
    disp_lines = 2;
    if (lidar_conn){
        if (af) {
            line[0] = "AF-ON";
        } else {
            line[0] = "DIST";
        }
        line[1] = dist;
        update_disp();
    } else if (!lidar_conn && (millis() - volt_check) > 5000){ //refresh rate > 5sec
        voltage = analogRead(p_batt)*10/1023;
        line[0] = "LOCKED";
        line[1] = String(voltage, 2);
        volt_check = millis(); //reset timer
        update_disp();
    }
}
void show_menu(){
    if (!edit){ //scroll menu browsing
        disp_lines = 3;
        if (curr_menu != 0){
            line[0] = menu[curr_menu - 1];
        } else {
            line[0] = "";
        }
        line[1] = ">" + menu[curr_menu] + "<";
        if (curr_menu != (menu_count - 1)){
            line[2] = menu[curr_menu + 1];
        } else {
            line[2] = "";
        }
    } else if (edit){ //edit mode
        disp_lines = 2;
        line[0] = menu[curr_menu];
        if (curr_menu < 3){ //up-down
            line[1] = "-/+";
        } else if (curr_menu == 3){
            line[1] = fps[curr_fps];
        } else if (curr_menu == 4){
            line[1] = fps[curr_codec];
        } else if (curr_menu == 5){
            disp_lines = 3;
            line[1] = lens[curr_lens][0] + "-" + lens[curr_lens][1];
            line[2] = "f/" + lens[curr_lens][2];
        } else if (curr_menu == 6){
            line[1] = sens;
        } 
    }
    update_disp();    
}
void update_disp(){
    display.clearDisplay();
    
    display.setTextSize(2); //scale
    display.setTextColor(WHITE);
    //align center
    display.setCursor(padding(0,2), 4); 
    display.print(line[0]);
    
    switch (disp_lines){
        case 2:
            display.setTextSize(4);
            display.setCursor(padding(1,4), 26);
            display.print(line[1]);
            break;
        case 3:
            display.setCursor(padding(1,2), 24);
            display.print(line[1]);
            display.setCursor(padding(2,2), 44);
            display.print(line[2]);
            break;
    }

    display.display();
}
int padding(int ln, int font_size){
    return ((120 - sizeof(line[ln]) * 6 * font_size) / 2 + 4);
}
// button related
void record(){
    step(1, 6);
}
void check_button(){
    if (encoder.SwitchPressed()){ // depressed
        if (!digitalRead(p_push)){
            unsigned long curr = millis();
            while (!digitalRead(p_push)){
                if ((millis() - curr) > 1000){ // long push
                    flip_lock();
                }
            }
        }
        button_handler(); //short push
    }
}
void flip_lock(){
    if (lock){
        lock = false;
        show_menu();
    } else {
        lock = true;
    }
}
void button_handler(){
    if (!lock){
        if (edit){            //edit mode
            edit = false;
            set_value();
        } else if (!edit){            //menu browsing
            edit = true;
        }
        show_menu();
    }
}

// encoder related
void enc_interrupt(void){
    if (encoder.HasRotaryValueChanged()){
        int movement = encoder.GetRotaryValue(1);

        if (lock){ 
            //when locked change audio
            sbus_value[5] = sbus_value[5] + movement * 20;
            sbus.Servo(6, sbus_value[5]);
            sbus.Update();
            sbus.Send();
        } else if (!lock){
            if (!edit){
                curr_menu = constrain((curr_menu + movement), 0, (menu_count - 1));//nav menu system - free move
            } else if (edit){
                //change values
                if (curr_menu < 3){
                    step(movement, curr_menu);// up down
                } else if (curr_menu == 3){
                    curr_fps = constrain((curr_fps + movement), 0, (fps_count - 1));
                } else if (curr_menu == 4){
                    curr_codec = constrain((curr_codec + movement), 0, (codec_count - 1));
                } else if (curr_menu == 5){
                    curr_lens = constrain((curr_lens + movement), 0, (lens_count - 1));
                } else if (curr_menu == 6){
                    sens = constrain((sens + movement), 1, 10);
                }
            }
        }
        show_menu();
    }
}

// Autofocus
void read_lidar(){
    if (Serial3.read() == leading){
        uart[0] = leading;
        if (Serial3.read() == leading){
            uart[1] = leading;
            for (int i = 2; i < 9; i++){
                uart[i] = Serial3.read();
            }
            checksum = uart[0]+uart[1]+uart[2]+uart[3]+uart[4]+uart[5]+uart[6]+uart[7];
            if (uart[8] == (checksum & 0xff)){
                raw_dist = uart[3]*256 + uart[2];
                strength = uart[5]*256 + uart[4];
                if (20 < strength) {
                    dist = int( dist - (sens/10 * (dist - raw_dist)) ); //unit: cm
                }
            }
        }
    }
}
void servo_drive(){
    int focus;
    if (!digitalRead(p_afon)){
        af = true;

        float x[11], y[11];
        float z = dist;
        
        for (int i = 0; i < 11; i++){
          x[i] = scale[curr_lens][i];
          y[i] = pos[curr_lens][i];
        }
        
        af_curve.setPoints(x, y, 11);
        af_curve.setDegree( Catmull );

        focus = int(af_curve.value(z));

    } else {
        af = false;
        focus = map(analogRead(p_focus), 0, 1023, pos[curr_lens][1], pos[curr_lens][9]);
    }
    servo.writeMicroseconds(focus);
}

/* execution */

void setup(){
    Serial.begin(115200); //USB. shared with S1. debug only
    //Serial2.begin(100000); //sbus init by library
    Serial3.begin(115200); //lidar
    sbus.begin();
    
    encoder.AddRotaryCounter(1, 10, false); //max 10
    encoder.SetRotary(1);
    
    if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3D)) { // Address 0x3D for 128x64
        Serial.println(F("SSD1306 allocation failed"));
        for(;;); // Don't proceed, loop forever
    }

    welcome();

    pinMode(p_run, INPUT_PULLUP);
    pinMode(p_push, INPUT_PULLUP);
    pinMode(p_enca, INPUT_PULLUP);
    pinMode(p_encb, INPUT_PULLUP);
    pinMode(p_afon, INPUT_PULLUP);

    pinMode(p_batt, INPUT);
    pinMode(p_focus, INPUT);
    
    attachInterrupt(digitalPinToInterrupt(p_run), record, FALLING); //active low
    attachInterrupt(digitalPinToInterrupt(p_enca), enc_interrupt, CHANGE);
    attachInterrupt(digitalPinToInterrupt(p_encb), enc_interrupt, CHANGE);
    attachInterrupt(digitalPinToInterrupt(p_push), check_button, FALLING);
    
    servo.attach(p_servo);

    //initial sbus data either loaded from eeprom or default
    for(int i = 0; i < 18; i++){
        sbus.Servo(i,sbus_value[i]);
    }
    sbus.Update();
}

void loop(void){
    if (Serial3.available() > 0){
        read_lidar();
        lidar_conn = true;
    } else {
        lidar_conn = false;
    }

    servo_drive();

    if (lock){
        lock_screen();
    } else {
        if (curr_menu == 7 && edit == true){
            line[1] = servo.readMicroseconds();
            update_disp();
        }
    }
    
}
