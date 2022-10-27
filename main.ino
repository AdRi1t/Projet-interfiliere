#include <SendOnlySoftwareSerial.h>

#define TAILLE 100
#define DELAY_BETWEEN_MEASURES 50

char temp;
char cas=0;

//const int freqOutputPin = 13;   // OC1A output pin for ATmega32u4 (Arduino Nano)
const int ocr1aval  = 0;
const int SEL = 10;
const int OEB_0 = 2;
const int OEB_1 = 3;
const int OEB_2 = 11;
const int OEB_3 = 12;
const int txPin = 13; 

int OEB_n[4]={OEB_0,OEB_1,OEB_2,OEB_3};

const int RST = 8;

const float K1 = 0.001 * 21 / 51.29;
const float K2 = 0.0001 * 90 / 570;
const float K3 = 0.001 * 21 / 51.29;
const float K4 = 0.0001 * 90 / 570;

const int Kv=12;

static unsigned char cpt = 0;

int pos_0;
int pos_1;
int pos_2;
int pos_3;

int inc_pos_0;
int inc_pos_1;
int inc_pos_2;
int inc_pos_3;

int consigne_vitesse[4] = {0,0,0,0};
int vitesse[4] = {0,0,0,0};
int anc_pos[4] = {0,0,0,0};
int pos[4] = {0,0,0,0};
int sortie[4] = {0,0,0,0};

unsigned char tour = 0;



int i;

int anc_pos_0 = 0;
int anc_pos_1 = 1;
int anc_pos_2 = 0;
int anc_pos_3 = 1;

double theta = 0.0;
float x = 0.0;
float y = 0.0;

float l = 0;
float dl;
float dtheta;

float dv;
int vit0;
int vit1;



int sortie0;
int sortie1;
int sortie2;
int sortie3;

unsigned long t_0;
unsigned long t_1;   
  
SendOnlySoftwareSerial Serial4(txPin);  // Tx pin

/*
  OCR1AVAL
  0 = 8MHz
  1 = 4MHz
  3 = 2MHZ
  7 = 1MHz
*/

void setup()
{

  TCCR1A = B00000000;// Set Timer/Counter1 to normal mode.
  TCNT1  = 0;// Set Timer/Counter1 to 0
  OCR1A = 1;// Set the Output Compare A for Timer/Counter1, 1 MHz
  //  TIMSK1 |= B00000010;// Set to enable Compare A Match Vector for ISR; leave other vectors unchanged
  TCCR1A = B01000000;// Set OC1A to toggle.
  TCCR1B = B00001010;// Set WGM12 for CTC to OCR1A. Set clock source to F_CPU/8 (2Mhz).

  /* Lecture des 8 bits de poids faible sur PC (6bits) et PD (2Bits) */
  DDRB |= B00011111;
  //Set RST_ as an output - PB0
  //Set OC1A as an Output - PB1
  //Set OEB_1 as an Output - PB2
  //Set SEL as an Output - PB2
  //Set OEB_0 as an Output - PB4


  DDRC &= B11110000;  // C0-C3 en entrée
  // Lecture des 4 bits de poids faible sur le port C

  DDRD &= B00001111;  // D4-D7 en entrée
  DDRD |= B00001111;  // D0-D3 en sortie
  // Lecture des 4 bits de poids fort sur le port D

  /* Initialisation port Série */
  digitalWrite(RST, 1);
  Serial.begin(115700);
  Serial4.begin(9600);
  sortie0 = 0;
  sortie1 = 0;
  sortie2 = 0;
  sortie3 = 0;


  

  consigne_vitesse[0] = 70;
  consigne_vitesse[1] = -70;
  consigne_vitesse[2] = -70;
  consigne_vitesse[3] = 70;
  

  consigne_vitesse[0] = -70;
  consigne_vitesse[1] = 70;
  consigne_vitesse[2] = -70;
  consigne_vitesse[3] = 70;
  

  consigne_vitesse[0] = 70;
  consigne_vitesse[1] = 70;
  consigne_vitesse[2] = 70;
  consigne_vitesse[3] = 70;

  
  delay(4000);
  /* droit */
  Serial4.write(170);
  delay(1);
  
  output(sortie0, 0);
  delay(1);
  output(sortie2, 2);
  delay(1);
  output(sortie1, 1);
  delay(1);
  output(sortie3, 3);

  // init_sabertooth();

  t_0 = millis();
}

void loop()
{
  static unsigned long previousMillis = 0;
  volatile unsigned long currentMillis = millis();   
    

/*   
  if(read(str_lect)){
      
    int n1;
    sscanf(str_lect,"%d %d",&consigne_vitesse,&n1);
  }
  

  /* Réalise une prise de mesure toutes les DELAY_BETWEEN_MEASURES millisecondes */
//    if (currentMillis - previousMillis >= DELAY_BETWEEN_MEASURES) {
//      previousMillis = currentMillis;
  t_1=millis();
  
  if (cas==1) 
  {
  consigne_vitesse[0] = 70;
  consigne_vitesse[1] = -70;
  consigne_vitesse[2] = -70;
  consigne_vitesse[3] = 70;
  if ((t_1-t_0)>4000)
  {
    t_0=t_1;
    cas=2;
  }
  }
  else if (cas==2)
  {
  consigne_vitesse[0] = -70;
  consigne_vitesse[1] = 70;
  consigne_vitesse[2] = -70;
  consigne_vitesse[3] = 70;
  if ((t_1-t_0)>4000)
  {
    t_0=t_1;
    cas=3;
  }
  }
  else if (cas==3)
  {
  consigne_vitesse[0] = 70;
  consigne_vitesse[1] = 70;
  consigne_vitesse[2] = 70;
  consigne_vitesse[3] = 70;
  if ((t_1-t_0)>4000)
  {
    t_0=t_1;
    cas=4;
  }
  }
  else if (cas==4)
  {
  consigne_vitesse[0] = 70;
  consigne_vitesse[1] = -70;
  consigne_vitesse[2] = 70;
  consigne_vitesse[3] = -70;
  if ((t_1-t_0)>4000)
  {
    t_0=t_1;
    cas=5;
  }
  }
  else if (cas==5)
  {
  consigne_vitesse[0] = -70;
  consigne_vitesse[1] = -70;
  consigne_vitesse[2] = -70;
  consigne_vitesse[3] = -70;
  if ((t_1-t_0)>4000)
  {
    t_0=t_1;
    cas=6;
  }
  }
  else
  {
  consigne_vitesse[0] = -70;
  consigne_vitesse[1] = 70;
  consigne_vitesse[2] = 70;
  consigne_vitesse[3] = -70;
  if ((t_1-t_0)>4000)
  {
    t_0=t_1;
    cas=1;
  }
  }
  
  
      asservissement();

//    char str_x[10];
//    char str_y[10];
//    char str_theta[10];
//    
//    dtostrf(x, 4, 2, str_x);
//    dtostrf(y, 4, 2, str_y);
//    dtostrf(theta* RAD_TO_DEG, 4, 2, str_theta);
//     
    //sprintf(str_emis,"%d %d %s %s %s\n",inc_pos_0,inc_pos_1,str_x,str_y,str_theta);
    //while(!print(str_emis));
      
//    }

  

}

void asservissement()
{
  int n=0;
for (int n=0;n<4;n++)
  {
   pos[n]=readHCTL_n(n);
   vitesse[n]=pos[n]-anc_pos[n];
   anc_pos[n]=pos[n];
   sortie[n]=(Kv*(consigne_vitesse[n]-vitesse[n]))/10+consigne_vitesse[n]/2;
  output(sortie[n],n);
  
//  Serial.print(n);
//  Serial.write(' ');
    Serial.print(vitesse[n]);
    Serial.write(' ');
//  Serial.print(sortie[n]);
//  Serial.write(' ');
    delay(5);  
  }
  
Serial.write('\n');

//  
//  anc_pos_0 = pos_0;
//  anc_pos_1 = pos_1;
//  pos_0 = readHCTL();
//  pos_1 = -readHCTL1();
//
//  inc_pos_0 = pos_0 - anc_pos_0;
//  inc_pos_1 = pos_1 - anc_pos_1;
//
//  dl = (inc_pos_1 + inc_pos_0) * K1; // le /2 est compris dans le K1, - pour respecter les sens moteurs
//  l += dl;
//
//  dtheta = ( inc_pos_0- inc_pos_1) * K2; // le /2 est compris dans le K1, - pour respecter les sens moteurs
//  theta += dtheta;
//  
//  x += dl * cos(theta);
//  y += dl * sin(theta);
//  //  Serial.println(l);
//
//    
//
//    vit0=consigne_vitesse-500*theta; // nb codeur apeu pres 600 pour 10 à 50 ms
//    vit1=consigne_vitesse+500*theta; // nb codeur apeu pres 600 pour 10 à 50 ms
//    
//    //sortie = (vit * 10) / 600 + (vit - inc_pos_1) / 30;
//    
//    sortie0 = ((vit0 * 15L  + (vit0 - inc_pos_0) * 25L)/600L);
//    sortie1 = ((vit1 * 15L  + (vit1 - inc_pos_1) * 25L)/600L);
//    
////    Serial.print(sortie0);
////    Serial.print(" ");
////    Serial.print(sortie1);
////    Serial.print(" ");
//
//
////    sortie0 = 0;
////   sortie1 = 0;
//
//    
//    output(128,sortie0, 0);
//    output(128,sortie0, 0);
//    output(129,sortie1, 0);
//    output(129,sortie1, 0);
//
////  Serial.print(inc_pos_0);
////  Serial.print(" ");
////  Serial.print(inc_pos_1);
////  Serial.print(" ");
////  Serial.print(x);
////  Serial.print(" ");
////  Serial.print(y);
////  Serial.print(" ");
////  Serial.println(theta * RAD_TO_DEG );  
//
//
//
}



int readHCTL_n(int n)
{
  unsigned char LSB;
  unsigned char MSB;
  int POS;
  /* selection du circuit HCTL */
  
  digitalWrite(SEL, 0);
  digitalWrite(OEB_n[n], 0);

  /* lire HCTL MSB */
  MSB = inputHCTL();
  /* lire HCTL LSB */
  digitalWrite(SEL, 1);
  //  SEL = 1;
  LSB = inputHCTL();
  digitalWrite(OEB_n[n], 1);

  POS = (MSB << 8) | LSB;
  return POS;
}



int inputHCTL()
{
  return ((PINC & B00001111) | (PIND & B11110000));
}


void commande(byte address,int val, byte mode)
{
  //fonction d'envoi des commande au pont en H
//  byte address = 128;       //adresse du pont en H 001111 sur les selecteurs

  Serial4.write(address);
  Serial4.write(mode);
  Serial4.write(val);
  Serial4.write((address + mode + val) &  0b01111111);

  /*
    mySerial4.write(address);
    mySerial4.write(mode);
    mySerial4.write(val);
    mySerial4.write((address + mode + val) &  0b01111111);
  */
}




void output(int val, byte mot)
{
  
  if (mot == 0)
  {
    if (val > 127)
      commande(128, 127, 0 );
    else if ( val > 0)
      commande(128, val, 0);
    else if (val > -127)
      commande(128, -val, 1);
    else
      commande(128, 127, 1);
  }
  else if (mot == 2)
  {
    /* moteur 0 et 1 ont un sens opposé */
    if (val > 127)
      commande(128, 127, 1 + 4 );
    else if ( val > 0)
      commande(128, val, 1 + 4 );
    else if (val > -127)
      commande(128, -val, 4);
    else
      commande(128, 127, 4);
  }
  else if (mot == 1)
  {
    if (val > 127)
      commande(129, 127, 0 );
    else if ( val > 0)
      commande(129, val, 0);
    else if (val > -127)
      commande(129, -val, 1);
    else
      commande(129, 127, 1);
  }
  else
  {
    /* moteur 0 et 1 ont un sens opposé */
    if (val > 127)
      commande(129, 127, 1 + 4 );
    else if ( val > 0)
      commande(129, val, 1 + 4 );
    else if (val > -127)
      commande(129, -val, 4);
    else
      commande(129, 127, 4);
  }
}


/* init */
/* Ramping au minimum soit valeur max 10 - commande 0b00010000*/
/* Baud Rate (decimal 15, binary 0b00001111, hex 0h0f)
  This value remains until it is changed and does persist through a power cycle. The values are:
  1: 2400 baud
  2: 9600 baud (default)
  3: 19700 baud
  4: 38400 baud
*/
/*
   14: Serial Timeout (decimal 14, binary 0b00001110, hex 0h0e)
*/

void init_sabertooth()
{
  byte address = 128;
  Serial4.write(address);
  Serial4.write(16);  // ramping
  Serial4.write(10);
  Serial4.write((address + 16 + 10) &  0b01111111);
  delay(10);
  //  Serial4.write(address);
  //  Serial4.write(15);  // baudrate
  //  Serial4.write(4);   // 2 - 9600 //4 -38400
  //  Serial4.write((address + 15 + 4) &  0b01111111);
  //  Serial4.begin(38400);
  //  delay(10);
  Serial4.write(address);
  Serial4.write(14);  // Time0ut
  Serial4.write(10);   // 10x100ms=1S
  Serial4.write((address + 14 + 10) &  0b01111111);
  delay(10);
}
