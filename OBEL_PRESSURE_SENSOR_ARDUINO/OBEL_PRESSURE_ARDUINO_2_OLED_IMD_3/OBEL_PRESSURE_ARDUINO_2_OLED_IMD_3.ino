
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Fonts/FreeSerif9pt7b.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

const unsigned char myBitmap [] PROGMEM = {
 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x0e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x38, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x03, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x01, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0xfc, 0x00, 0x00, 0x00, 0x00, 0x1f, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x3f, 0xf8, 0x00, 0x00, 0x07, 0xfe, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x07, 0xff, 0xe0, 0x03, 0xff, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xf0, 0x07, 0xff, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x0f, 0xfc, 0x1f, 0xf0, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x7f, 0xff, 0x7f, 0xff, 0x7f, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x03, 0xff, 0xf0, 0xcf, 0xf8, 0x07, 0xff, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x0f, 0xff, 0xcf, 0xfb, 0xef, 0xf9, 0xff, 0xf8, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x1f, 0xff, 0x9f, 0xfd, 0xdf, 0xfc, 0xff, 0xfe, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x3f, 0xff, 0x1f, 0xfe, 0x7f, 0xfc, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x7f, 0xff, 0x8f, 0xff, 0xff, 0xf8, 0xff, 0xff, 0xe0, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x0f, 0xf1, 0xff, 0xc3, 0xf7, 0xf7, 0xf1, 0xff, 0xff, 0xff, 0x80, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x78, 0x00, 0x07, 0xe0, 0x1f, 0xfc, 0x07, 0xff, 0x80, 0x00, 0x78, 0x00, 0x00, 0x00, 
	0x00, 0x01, 0xc0, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0xfc, 0x00, 0x00, 0x1e, 0x00, 0x00, 0x00, 
	0x00, 0x03, 0x80, 0x00, 0x00, 0x7f, 0xe0, 0x00, 0xff, 0xf0, 0x00, 0x00, 0x03, 0x80, 0x00, 0x00, 
	0x00, 0x06, 0x00, 0x00, 0x00, 0x1f, 0x00, 0x00, 0x1f, 0xc0, 0x00, 0x00, 0x01, 0xc0, 0x00, 0x00, 
	0x00, 0x0e, 0x00, 0x00, 0x00, 0x0c, 0x00, 0x00, 0x03, 0x80, 0x00, 0x00, 0x00, 0xe0, 0x00, 0x00, 
	0x00, 0x0c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x70, 0x00, 0x00, 
	0x00, 0x1c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30, 0x00, 0x00, 
	0x00, 0x0c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0x00, 0x00, 
	0x00, 0x0c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0x00, 0x00, 
	0x00, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30, 0x00, 0x00, 
	0x00, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30, 0x00, 0x00, 
	0x00, 0x03, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 0x00, 0x00, 
	0x00, 0x00, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xc0, 0x00, 0x00, 
	0x00, 0x00, 0x3e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xc0, 0x00, 0x00, 
	0x00, 0x00, 0x03, 0xfc, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x0c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3c, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x0c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xe0, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x0c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1e, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x38, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x01, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xc0, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x70, 0x00, 0x00, 0x00, 0x7c, 0x00, 0x0f, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x1e, 0x00, 0x00, 0x01, 0xc7, 0xe0, 0xf8, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x03, 0xe0, 0x00, 0x1f, 0x00, 0x3f, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x3f, 0xff, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x03, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x03, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1c, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x1f, 0x83, 0xcf, 0xf8, 0x3b, 0x7c, 0xfc, 0x3f, 0x7f, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x7f, 0xe3, 0xfe, 0x7c, 0x7b, 0xc7, 0x8e, 0xe1, 0xdc, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x7f, 0x83, 0xfc, 0x3c, 0xf3, 0x07, 0x07, 0xc0, 0xfc, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x3f, 0xf3, 0xfc, 0x1f, 0xf3, 0x07, 0x07, 0xff, 0xfc, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x79, 0xf3, 0xdf, 0x0f, 0xe3, 0x07, 0x07, 0xc0, 0xfc, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x7f, 0xe3, 0xcf, 0x87, 0xc3, 0x07, 0x07, 0xe1, 0xdc, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0x83, 0x03, 0x07, 0x7f, 0x0f, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x29, 0x73, 0xbf, 0x19, 0x92, 0x57, 0x18, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x3f, 0xff, 0xff, 0xff, 0x1f, 0xff, 0xfe, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x3f, 0xff, 0xfe, 0xff, 0x1f, 0xbf, 0xfc, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
  
  };

void display_init()
{
  Serial.println("Display Init");
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C))                                    // addr_cntr 0x3C for 128x32
  {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;)
      ;
  }
  else
    Serial.println("Display Init success!!");
  display.display();
  display.clearDisplay();
  //display.drawPixel(10, 10, SSD1306_WHITE);
  display.display();

  display.clearDisplay();

  display.setTextSize(1);                                                              // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE);                                                 // Draw white text
  display.setCursor(0, 0);                                                             // Start at top-left corner
  //display.println(F("Hello, world!!!.."));

  display.display();
  display.clearDisplay();
  display.display();
  delay(100);
}

char _Pressure[50] = {'\0'};
char _Pressure1[50] = {'\0'};
char Data_Buffer[50] = {'\0'};
void setup() {

 display_init();

 pinMode(13, OUTPUT);
 //Serial.begin(115200);
 Serial1.begin(1200,SERIAL_7E2);

  display.clearDisplay();
  display.drawBitmap(0, 0, myBitmap, 128, 64, WHITE);
  display.display();
  delay(5000); 
}

void loop() { 
 delay(40); //3000
 digitalWrite(13, HIGH);
 delay(160); //250
 Serial1.print("PD0!");
 delay(70); //100//110
 digitalWrite(13, LOW);
 delay(300); //330//320
 //display.clearDisplay();
 //display.display();
if(Serial1.available() > 0)
{
  memset(_Pressure,0,sizeof(_Pressure));
  memset(Data_Buffer,0,sizeof(Data_Buffer));
  Serial1.readBytesUntil('\0',Data_Buffer,sizeof(Data_Buffer));
  //Serial.print(Data_Buffer);

  strcpy(_Pressure, strtok(Data_Buffer,"+"));
  strcpy(_Pressure1, strtok(NULL,"\n") );
  Serial.print(_Pressure1);
  Serial.println("mBar");
  //display.clearDisplay();
  //display.display();
}
  display.clearDisplay();
  display.setTextSize(2);                    
  display.setCursor(17,0);
  display.print("PRESSURE");
  //display.setTextSize(1);
  display.setCursor(30, 20);
  display.print(_Pressure1);
  display.setCursor(30, 45);
  display.print(" mBar"); 
  display.display();

 //memset(_Pressure,0,sizeof(_Pressure));
 //memset(_Pressure1,0,sizeof(_Pressure1));
 memset(Data_Buffer,0,sizeof(Data_Buffer));
}
