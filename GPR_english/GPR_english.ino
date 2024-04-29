#include <SD.h>

//////////////////////////////////
//     <c> Mirel Paun 2020      //
//////////////////////////////////


const uint16_t  Display_Color_Black        = 0x0000;
const uint16_t  Display_Color_Blue         = 0x001F;
const uint16_t  Display_Color_Red          = 0xF800;
const uint16_t  Display_Color_Green        = 0x07E0;
const uint16_t  Display_Color_Cyan         = 0x07FF;
const uint16_t  Display_Color_Magenta      = 0xF81F;
const uint16_t  Display_Color_Yellow       = 0xFFE0;
const uint16_t  Display_Color_White        = 0xFFFF;

//sampling period is 224.38 us
#define Adresa_MCP4725 96             //DAC adress
#define pin_ADC A15                   //ADC input pin
#define pin_BAT A13                   //Battery voltage input pin
#define WHITE     0x0000              //color CHANGE ME BACK - 0xFFFF
#define BLACK     0x0000              //color
#define RED       0xF800              //color
#define GREEN     0x07E0              //color
#define DKBLUE    0x000D              //color
#define buton_sus 7
#define buton_OK 6
#define buton_jos 5
#define senzor_pas 3
#define SDC_CS 53                     //SD card control pin
#include <Wire.h>
#include <arduinoFFT.h>
#include "Free_Fonts.h"
#include <SD.h>
#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7735.h> // Hardware-specific library for ST7735

#define TFT_CS        A5
#define TFT_RST        A4 // Or set to -1 and connect to Arduino RESET pin
#define TFT_DC         A3
#define TFT_MOSI A2  // Data out
#define TFT_SCLK A1  // Clock out
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST);
arduinoFFT FFT = arduinoFFT();

File fisier;

const word battery_voltage_thresholds[3] = {384, 392, 401};   //battery voltage threshold levels: low - med1 - med2 (V_bat/5V*1023*0.4)

//GPR global parameters
const double GPR_horizontal_step = 0.24;    //GPR horizontal step [m]
const double bandwidth = 587000000;           //GPR bandwidth [Hz]; 910 MHz - 323 MHz
const double max_amplitude = 30000; // original value is 30000, changed for testing  //maximum FFT module value (for scaling)
const word sample_size_no = 256;                //sample no.
const double tftFontHeight = 0.1;

//menus
const char *materials_to_penetrate[] = {"Air - 0.3 m/ns", "Ice - 0.16 m/ns", "Dry sand - 0.15 m/ns", "Dry soil (Granite) - 0.13 m/ns", "Limestone - 0.12 m/ns", "Asphalt - 0.11 m/ns", "Concrete - 0.1 m/ns", "Moist soil - 0.09 m/ns", "Wet soil (Silt) - 0.07 m/ns", "Saturated sand (Clay) - 0.06 m/ns", "Sweet water - 0.03 m/ns", "Salt water - 0.01 m/ns"};
const double wave_speed[] = {300000000, 160000000, 150000000, 130000000, 120000000, 110000000, 100000000, 90000000, 70000000, 60000000, 30000000, 10000000};
const word resolution_mults[] = {16, 32, 64, 128}; //resolution multiples
const double depth_steps[] = 		{10, 5, 2.5, 1, 0.5, 0.25, 0.1, 0.05, 0.025, 0.01};
const word decimal_depth_steps[] =  {0,  0, 1  , 0,   1,    2,   1,    2,     3,    2};
const word distance[] = {12, 24, 48, 96};

//graph constants
const word orig_x = 30, orig_y = 116, graph_width = 160, graph_height = 128; //graph origin coord. and dimensions [pixels] - 128x160 origy = 116, x = 30
//variabile globale afisare grafic
double max_dist, pas_dist, max_adanc, min_adanc, pas_adanc, resolution; // [m]
double c; //[m/s]
word num_cel_res_horizontal, num_cel_res_vertical, height_res, res_width, nr_zecimale_pasi_adanc, xpos, ypos, pas = 0;

//antenna coupling correction (anechoic chamber acquisition)
//const word corectie[nr_esant] = {497, 497, 477, 383, 251, 163, 125, 113, 146, 210, 305, 430, 550, 682, 801, 893, 947, 964, 922, 787, 654, 569, 521, 486, 455, 446, 451, 454, 439, 409, 377, 352, 337, 332, 323, 334, 342, 354, 371, 384, 397, 410, 420, 433, 449, 468, 496, 528, 560, 596, 637, 674, 705, 726, 733, 735, 735, 738, 749, 757, 760, 754, 731, 699, 657, 597, 520, 432, 342, 264, 213, 180, 164, 164, 173, 194, 222, 252, 288, 316, 350, 390, 425, 459, 491, 522, 548, 571, 590, 606, 624, 642, 660, 681, 694, 703, 706, 701, 692, 676, 651, 623, 590, 557, 528, 501, 477, 457, 443, 433, 429, 429, 431, 433, 439, 449, 462, 476, 492, 508, 525, 543, 566, 587, 604, 609, 603, 589, 570, 547, 519, 482, 434, 376, 326, 277, 233, 194, 159, 147, 167, 224, 306, 383, 449, 503, 545, 576, 601, 611, 615, 616, 617, 617, 616, 614, 613, 609, 602, 593, 584, 577, 571, 566, 559, 553, 545, 539, 533, 528, 524, 521, 518, 515, 510, 505, 500, 496, 493, 490, 485, 480, 477, 475, 474, 475, 476, 479, 484, 490, 496, 502, 508, 514, 522, 532, 538, 542, 541, 540, 538, 536, 536, 534, 531, 525, 520, 511, 503, 497, 491, 487, 483, 479, 473, 467, 468, 468, 466, 466, 466, 466, 467, 467, 470, 468, 467, 467, 466, 466, 465, 465, 467, 468, 467, 468, 467, 471, 473, 475, 477, 480, 482, 484, 486, 489, 491, 494, 495, 497, 497, 498, 498, 499, 498, 498};

//global variables
word i, scaled_amplitude, color, samples[sample_size_no];
double real[sample_size_no], imag[sample_size_no], corrected_amplitude_with_dist;
boolean cont, card;

//------------------------------------------------------------------------
void setup()
{
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);

  tft.initR(INITR_BLACKTAB);      // Init ST7735S chip, black tab
  tft.setFont();
  tft.setTextSize(1);
  tft.fillScreen(BLACK);
  tft.setRotation(1);
  tft.setTextColor(Display_Color_White, Display_Color_Black);
  xpos = tft.width() / 2; // Middle screen
  ypos = (tft.height() / 2) - tftFontHeight; //tft.fontHeight(GFXFF); // Middle screen

  pinMode(buton_sus, INPUT);
  pinMode(buton_OK, INPUT);
  pinMode(buton_jos, INPUT);
  pinMode(senzor_pas, INPUT);

  //tft.drawString("<c> Mirel Paun 2020", xpos, ypos, GFXFF);
  delay(1000);
  tft.fillScreen(BLACK);   

  card = false; //no SD card
  //tft.drawString("SD Card missing!!!", xpos, ypos, GFXFF);
  //Serial.print("heck. - SD card missing WArning");
  delay(1000);

  //DAC at 0
  Wire.beginTransmission(Adresa_MCP4725);
  Wire.write( 0 );        //MSB
  Wire.write( 0 );        //LSB
  Wire.endTransmission();
  delay(10);

  if (card)
  {
    //Verify file existance, if missing, create it and write speed index
    if (!SD.exists("Date.dat")) {
      fisier = SD.open("Date.dat", FILE_WRITE);
      fisier.write(highByte(i));
      fisier.write(lowByte(i));
      fisier.close();
    }
  }

  // TODO find actual tuning vals to run GPR - rn just using dummy wave_speed, depth, etc vals!

  // vel params
  resolution = wave_speed[0] / (2.0 * bandwidth); //depth resolution [m]
  min_adanc = -4 * resolution;  	 //offset cables + antennas (aprox. 4 * rezolution)

  // depth params
  num_cel_res_vertical = resolution_mults[1];
  max_adanc = num_cel_res_vertical * resolution + min_adanc;
  pas_adanc = depth_steps[4];
  nr_zecimale_pasi_adanc = decimal_depth_steps[4];

  // dist params
  max_dist = distance[0];
  pas_dist = max_dist / 6;

  //graph parameters
  height_res = graph_height / num_cel_res_vertical;  	  				  //[pixels]
  num_cel_res_horizontal = max_dist / GPR_horizontal_step;
  res_width = graph_width / num_cel_res_horizontal;                   //[pixels]
  // Draw grid
  tft.fillScreen(BLACK);
  Graph(tft, orig_x, orig_y, graph_width, graph_height, 0, max_dist, pas_dist, min_adanc, max_adanc, pas_adanc, "GPR", "Distance [m]", "Depth [m]");
  //afis_card();
  //masurare_afis_bat();
}

//------------------------------------------------------------------------
void loop()
{
  // look every second? TODO TUNE THIS
  delay(1000);
  
  // If screen is full, delete and start again
  if (((pas % num_cel_res_horizontal) == 0) && (pas != 0))
  {
    word sreen_nr = pas / num_cel_res_horizontal;
    tft.fillScreen(BLACK);
    Graph(tft, orig_x, orig_y, graph_width, graph_height, sreen_nr * max_dist, (sreen_nr + 1) * max_dist, pas_dist, min_adanc, max_adanc, pas_adanc, "GPR", "Distance [m]", "Depth [m]");
    // afis_card();
    // masurare_afis_bat();
  }
  // Generate modulation signal and sample radar echo
  for (i = 0; i <= 4080; i = i + 16) // DAC goes from 0 - 4.98V
  {
    // Write to DAC
    Wire.beginTransmission(Adresa_MCP4725);
    Wire.write( highByte(i) );        //MSB
    Wire.write( lowByte(i) );         //LSB
    Wire.endTransmission();
    // Read from ADC
    samples[i >> 4] = analogRead(pin_ADC); // >>4 means /16
  }
  //Bring DAC to 0
  Wire.beginTransmission(Adresa_MCP4725);
  Wire.write( 0 );        //MSB
  Wire.write( 0 );        //LSB
  Wire.endTransmission();

  //Store on SD Card
  if (card)
  {
    fisier = SD.open("Date.dat", FILE_WRITE);
    for (i = 0; i < sample_size_no; i++)
    {
      fisier.write(highByte(samples[i]));
      fisier.write(lowByte(samples[i]));
    }
    fisier.close();
  }

  // Prepare data for FFT
  for (i = 0; i < sample_size_no; i++)
  {
    //real[i] = (double)(esantioane[i]) - (double)(corectie[i]); // Load samples and correct antenna coupling
    real[i] = (double)(samples[i]) - 512.0;    // Load samples and remove d.c.
    imag[i] = 0.0;                                // Delete imaginary part
  }
  // Compute FFT
  FFT.Compute(real, imag, sample_size_no, FFT_FORWARD); //FFT
  FFT.ComplexToMagnitude(real, imag, sample_size_no);   //Compute FFT and store it in real
  //Draw one column
  for (i = 0; i < num_cel_res_vertical; i++)
  {
    if(i <= 40) corrected_amplitude_with_dist = real[i] * exp(i * 0.1);   // distance correction
    else corrected_amplitude_with_dist = real[i] * 54.598;                // distance correction
    scaled_amplitude = (word)(corrected_amplitude_with_dist * 255.0 / max_amplitude);
    Serial.println(scaled_amplitude);
    if (scaled_amplitude > 255) scaled_amplitude = 255;
    color = (((scaled_amplitude & 0b11111000) << 8) + ((scaled_amplitude & 0b11111100) << 3) + (scaled_amplitude >> 3));
    tft.fillRect(orig_x + 1 + pas % num_cel_res_horizontal * res_width, orig_y + 1 - graph_height + i * height_res, res_width, height_res, color);
  }
  pas++;
}

// Grid drawing routine
void Graph(Adafruit_ST7735 &tft, double gx, double gy, double w, double h,
           double xlo, double xhi, double xinc,
           double ylo, double yhi, double yinc,
           char *title, char *xlabel, char *ylabel)
{
  double ydiv, xdiv;
  double i;
  double temp;
  int rot, newrot;

  unsigned int gcolor = DKBLUE; //grid color
  unsigned int acolor = RED;    //axis color
  unsigned int tcolor = WHITE;  //text color
  unsigned int bcolor = BLACK;  //background color

  tft.setFont(); //------------------------
  //tft.setTextDatum(MR_DATUM);

  // draw x axis
  tft.drawLine(gx, gy - h, gx + w, gy - h, acolor);  //red axis
  tft.setTextColor(acolor, bcolor);
  //tft.drawString(xlabel, (int)(gx + w) , (int)(gy - h) - 5, 2);
  // draw origin
  tft.setTextColor(tcolor, bcolor);
  //tft.drawFloat(ylo, 3, gx - 4, gy - h, 1);

  for (i = 0; i <= yhi; i += yinc)
  {
    temp =  gy - h + (i - ylo) * h / (yhi - ylo);
    tft.drawLine(gx, temp, gx + w, temp, gcolor);
    // draw y axis labels
    tft.setTextColor(tcolor, bcolor);
    //tft.drawFloat(i, nr_zecimale_pasi_adanc, gx - 4, temp, 1);
  }
  tft.drawLine(gx, gy, gx + w, gy, gcolor);//graph bottom line

  // draw y axis
  for (i = xlo; i <= xhi; i += xinc)
  {
    temp =  (i - xlo) * w / (xhi - xlo) + gx;
    if (i == xlo) 									//red axis
    {
      tft.drawLine(temp, gy, temp, gy - h, acolor);
      tft.setTextColor(acolor, bcolor);
      //tft.setTextDatum(BC_DATUM);
      //tft.drawString(ylabel, (int)temp, (int)(gy - h - 8) , 2);
    }
    else
    {
      tft.drawLine(temp, gy, temp, gy - h + 1, gcolor);
    }
    //draw x axis labels
    tft.setTextColor(tcolor, bcolor);
    //tft.setTextDatum(TC_DATUM);
    //tft.drawNumber(i, temp, gy + 7, 1);
  }

  //draw graph label
  tft.setTextColor(tcolor, bcolor);
  //tft.drawString(title, (int)(gx + w / 2) , (int)(gy - h - 30), 4);
}

// void afis_card()
// {
//   if (card)
//   {
//     tft.setTextColor(GREEN, BLACK);
//     //tft.drawString("CARD OK", 370, 2, 2);
//   }
//   else
//   {
//     tft.setTextColor(RED, BLACK);
//     //tft.drawString("NO CARD", 370, 2, 2);
//   }
// }

// void masurare_afis_bat(void)
// {
//   //Measure and display battery state
//   word val = analogRead(pin_BAT);
//   if (val <= battery_voltage_thresholds[0]) // low
//   {
//     tft.drawRect(430, 5, 20, 12, RED);
//     tft.drawRect(431, 6, 18, 10, RED);
//     tft.fillRect(450, 7, 3, 8, RED);
//     tft.fillRect(432, 7, 16, 8, BLACK);
//   }
//   else if (val <= battery_voltage_thresholds[1]) // med1
//   {
//     tft.drawRect(430, 5, 20, 12, WHITE);
//     tft.fillRect(450, 7, 3, 8, WHITE);
//     tft.fillRect(432, 7, 5, 8, WHITE);
//     tft.fillRect(437, 7, 11, 8, BLACK);
//   }
//   else if (val <= battery_voltage_thresholds[2]) // med2
//   {
//     tft.drawRect(430, 5, 20, 12, WHITE);
//     tft.fillRect(450, 7, 3, 8, WHITE);
//     tft.fillRect(432, 7, 11, 8, WHITE);
//     tft.fillRect(443, 7, 5, 8, BLACK);
//   }
//   else // max
//   {
//     tft.fillRect(430, 5, 20, 12, WHITE);
//     tft.fillRect(450, 7, 3, 8, WHITE);
//   }
// }
