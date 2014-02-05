/*
 * DSP Radio (KT0915) control program on Arduino IDE
 *
 *  Created on: Feb. 5 , 2014
 *      Author: kodera2t
 */
/* Different version: Try if the previous one does not work well*/

#include <I2CLiquidCrystal.h>
#include <Wire.h>
I2CLiquidCrystal lcd(20, false);
#define RADIO 0x35
int read_byte,raw_upper,upper,lower,mode,read_byte2;
    int rssi,stereo;
float freq;
unsigned int channel_num,s_upper,s_lower;
unsigned char s_upper2, s_lower2, s_upper3;
unsigned int initial_num;
volatile int encorder_val;
volatile int mode_set=0; /// mode_set=0:AM, mode_set=1:FM
volatile int band_mode = LOW;
float listen_freq;
int terminal_1  = 2;
int terminal_2  = 4;
volatile char old_state = 0;
int ct,pt,event,event2;
int rssi_count;
int rssi_count2=0;


void i2c_write(int device_address, int memory_address, int value, int value2)
{
  Wire.beginTransmission(device_address);
  Wire.write(memory_address);
    delay(5);
  Wire.write(value);
    delay(5);
  Wire.write(value2);
    delay(5);
  Wire.endTransmission();
}

void i2c_read(int device_address, int memory_address)
{
Wire.beginTransmission(device_address);
Wire.write(memory_address);
Wire.endTransmission(false);
Wire.requestFrom(device_address, 2);
read_byte = Wire.read();
//Wire.requestFrom(device_address, 1);
read_byte2 = Wire.read();
Wire.endTransmission(true);
//delay(30);
}



void setup()
{
    unsigned int upper,lower,raw_upper;
    unsigned int mask,mode,mode_set;
    Wire.begin() ;
  attachInterrupt(0,Rotary_encorder,CHANGE);
  attachInterrupt(1,mode_setting,CHANGE);
   delay(100) ;  
    lcd.begin(8,2);
  pinMode(3, INPUT);
  pinMode(terminal_1, INPUT);
  pinMode(terminal_2, INPUT);
  digitalWrite(terminal_1, HIGH);
  digitalWrite(terminal_2, HIGH);
  digitalWrite(3,HIGH);
  
  int temp;
    temp=0;
//    listen_freq=83.3*1000; ///frequency in MHz
//    channel_num=listen_freq/50;
//    s_upper2=(channel_num>>8 | 0b10000000);
//    s_upper3=channel_num>>8;
//    s_lower2=channel_num&0b11111111;
//    i2c_write(RADIO,0x02,0b00000000,0b00000111);
//    ///tune FM 0x03
//// 83.3 MHz :                 0b110   10000010
////    i2c_write(RADIO,0x03,0b10000110,0b10000010);
//    i2c_write(RADIO,0x03,s_upper2,s_lower2);
//    i2c_write(RADIO,0x03,s_upper3,s_lower2);
//    i2c_write(RADIO,0x03,0b00000110,0b10000010);
    i2c_write(RADIO,0x04,0b01100000,0b10110000);
    i2c_write(RADIO,0x05,0b00010000,0b00100000);   
    i2c_write(RADIO,0x0F,0b10001000,0b00001111);
    i2c_write(RADIO,0x2E,0b00101000,0b10001100);
/// RSSI reading
//    i2c_read(RADIO,0x12);
//    rssi=-100+(read_byte2>>3)*3;
    

//lcd.setCursor(0,0);
//  lcd.print("FM");
////lcd.setCursor(5,0);
////  lcd.print(rssi);
//  freq=channel_num/20.0;
//lcd.setCursor(0,1);
//lcd.print(freq);
//  lcd.setCursor(4,1);
//lcd.print("MHz");
encorder_val=0;


//  i2c_write(RADIO,0x16,0b00000000,0b00000010);
//  listen_freq=88.6;  
//  initial_num=listen_freq*20.0;
//  channel_num=initial_num+encorder_val*2;
//  s_upper2=(channel_num>>8 | 0b10000000);
//  s_lower2=channel_num&0b11111111;
//  i2c_write(RADIO,0x02,0b00000000,0b00000111);
//  i2c_write(RADIO,0x03,s_upper2,s_lower2);
//  i2c_write(RADIO,0x0F,0b10001000,0b00010000);
//  lcd.setCursor(0,0);
//  lcd.print("FM");
//  freq=channel_num/20.0;
//  lcd.setCursor(0,1);
//  lcd.print(freq);
//  lcd.setCursor(4,1);
//  lcd.print("MHz ");

}

void loop()
{
//  rssi_display();
if(event==1){
  if(mode_set==0){
  i2c_write(RADIO,0x16,0b00000000,0b00000010);
//        i2c_write(RADIO,0x0F,0b10001000,0b00001111);
  listen_freq=83.3;  
  initial_num=listen_freq*20.0;
  channel_num=initial_num+encorder_val*2;
  s_upper2=(channel_num>>8 | 0b10000000);
  s_lower2=channel_num&0b11111111;
  i2c_write(RADIO,0x02,0b00000000,0b00000111);
  i2c_write(RADIO,0x03,s_upper2,s_lower2);
  lcd.setCursor(0,0);
  lcd.print("FM");
  freq=channel_num/20.0;
  lcd.setCursor(0,1);
//    lcd.print(encorder_val);
  lcd.print(freq);
  lcd.setCursor(4,1);
  lcd.print("MHz ");
  event=0;  
  }
  else if(mode_set==1){
  i2c_write(RADIO,0x16,0b10000000,0b11000010);
  i2c_write(RADIO,0x22,0b01010100,0b00000000);
//        i2c_write(RADIO,0x0F,0b10001000,0b00001111);
  listen_freq=1008;  
  initial_num=listen_freq;
  channel_num=initial_num+encorder_val*9;
  s_upper2=(channel_num>>8 | 0b10000000);
  s_lower2=channel_num&0b11111111;
  i2c_write(RADIO,0x02,0b00000000,0b00000111);
  i2c_write(RADIO,0x17,s_upper2,s_lower2);
  lcd.setCursor(0,0);
  lcd.print("AM      ");
  freq=channel_num;
  lcd.setCursor(0,1);
  lcd.print(freq);
    lcd.setCursor(4,1);
  lcd.print("kHz");
  event=0;  
  } else{
  i2c_write(RADIO,0x16,0b10000000,0b00000010);
  i2c_write(RADIO,0x22,0b01010100,0b00000000);
//        i2c_write(RADIO,0x0F,0b10001000,0b00001111);
  listen_freq=3000+(mode_set-2)*1000;  
  initial_num=listen_freq;
  channel_num=initial_num+encorder_val*5;
  s_upper2=(channel_num>>8 | 0b10000000);
  s_lower2=channel_num&0b11111111;
  i2c_write(RADIO,0x02,0b00000000,0b00000111);
  i2c_write(RADIO,0x17,s_upper2,s_lower2);
  lcd.setCursor(0,0);
  lcd.print("SW      ");
  freq=channel_num;
  lcd.setCursor(0,1);
  lcd.print(freq);
    lcd.setCursor(5,1);
  lcd.print("kHz");
  event=0;  
  }
}
}

void mode_setting(){
int sw,k;
ct=millis();
delay(1);
sw=digitalRead(3);
if(sw==LOW && (ct-pt)>50){
band_mode=HIGH;
mode_set=mode_set+1;
}
pt=ct;
if(mode_set>11){
  mode_set=0;
}
event=1;
encorder_val=0;
k=0;
}
    
void Rotary_encorder(void)
{
  if(!digitalRead(terminal_1)){
    if(digitalRead(terminal_2)){
      old_state = 'R';
    } else {
      old_state = 'L';
    }
  } else {
    if(digitalRead(terminal_2)){
      if(old_state == 'L'){ 
        encorder_val--;
      }
    } else {
      if(old_state == 'R'){
        encorder_val++;
      }
    }
    old_state = 0;
    event=1;
  }

}


void rssi_display(){
  if(mode_set==0){
//// display RSSI  every seconds. 
  rssi_count=millis()-1000*rssi_count2; 
  if(rssi_count>1000){
    rssi_count2++;
  i2c_read(RADIO,0x12);
  rssi=-100+(read_byte2>>3)*3;
  stereo=read_byte&0b00000011;
  if(stereo==0b11){
  lcd.setCursor(2,0);
  lcd.print("stereo");
  }else{
  lcd.setCursor(2,0);
  lcd.print("      ");
  }
//    lcd.setCursor(5,0);
//  lcd.print(rssi);
    rssi_count=0;
  }
}
}
