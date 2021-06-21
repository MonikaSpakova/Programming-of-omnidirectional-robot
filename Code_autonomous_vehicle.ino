// Pripojenie knižníc
#include "BluetoothSerial.h"
#include <SoftwareSerial.h>
#include <AccelStepper.h>
#include <ESP_FlexyStepper.h>
#include "Adafruit_VL53L0X.h"
#include <ESP32Servo.h>

// Enable pin na ESP32 pre spustenie Bluetooth
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

// Definovanie premenných
int m;
char dataIn;
char c;
String pokyn = "";
String slovo = "";
int counter = 0;
// Piny motory
const int MOTOR_STEP_PIN = 12;
const int MOTOR_DIRECTION_PIN = 28; 
const int MOTOR_ENABLE_PIN = 13;
const int SPEED_IN_STEPS_PER_SECOND = 200;
const int ACCELERATION_IN_STEPS_PER_SECOND = 3000;
const int DECELERATION_IN_STEPS_PER_SECOND = 3000;

const int MOTOR_LEFT_REAR = 32;
const int MOTOR_LEFT_FRONT = 33;
const int MOTOR_RIGHT_REAR = 14;
const int MOTOR_RIGHT_FRONT = 27;

// Piny ultrazvukove senzory
const int TrigPin_predny = 4;
const int TrigPin_pravy = 26;
const int TrigPin_zadny = 2;

const int EchoPin_predny = 5;
const int EchoPin_pravy = 25;
const int EchoPin_zadny = 34;

// Piny servo motor
const int servoPin = 17;

// Vytvorenie mien
ESP_FlexyStepper stepper;

Adafruit_VL53L0X lox = Adafruit_VL53L0X();

BluetoothSerial SerialBT;

Servo Servomotor;

VL53L0X_RangingMeasurementData_t vzdialenost_laser; 

// Začiatok komunikácie
void setup() {
  Serial.begin(115200);
  //SerialBT.begin(115200);
  Wire.begin (21, 22);   // sda= GPIO_21 /scl= GPIO_22

  Serial.println("zacinam setup");
  SerialBT.setTimeout(1);
  SerialBT.begin("ESP32test"); //Bluetooth device name
  Serial.println("The device started, now you can pair it with bluetooth!");

  // Nastavenie servo motoru
  Servomotor.setPeriodHertz(50); 
  Servomotor.attach(servoPin);

  // Nastavenie rýchlosti na motore
  stepper.connectToPins(MOTOR_STEP_PIN, MOTOR_DIRECTION_PIN);
  stepper.setSpeedInStepsPerSecond(SPEED_IN_STEPS_PER_SECOND); // rychlost otacania (mozes menit)
  stepper.setAccelerationInStepsPerSecondPerSecond(ACCELERATION_IN_STEPS_PER_SECOND); // zrychlenie - akcelerácia (tiez mozes menit)
  stepper.setDecelerationInStepsPerSecondPerSecond(DECELERATION_IN_STEPS_PER_SECOND); // spomalenie - deceleracia (tiez mozes menit)

if (!lox.begin()) {
   Serial.println(F("Failed to boot VL53L0X"));
   while(1);
 }
 // power 
 Serial.println(F("VL53L0X API Simple Ranging example\n\n"));

  //Nastavenie OUTPUTOV
  pinMode(MOTOR_ENABLE_PIN, OUTPUT);
  
  pinMode(MOTOR_LEFT_REAR, OUTPUT);
  pinMode(MOTOR_LEFT_FRONT, OUTPUT);
  pinMode(MOTOR_RIGHT_REAR, OUTPUT);
  pinMode(MOTOR_RIGHT_FRONT, OUTPUT);

  pinMode(TrigPin_predny, OUTPUT); 
  pinMode(TrigPin_pravy, OUTPUT);
  pinMode(TrigPin_zadny, OUTPUT);

  // Nastavenie INPUTOV
  pinMode(EchoPin_predny, INPUT);
  pinMode(EchoPin_pravy, INPUT);
  pinMode(EchoPin_zadny, INPUT);
  Serial.println("koncim setup");
}
// Skenovanie zariadení pre I2C 
void Scanner ()
{
  Serial.println ();
  Serial.println ("I2C scanner. Scanning ...");
  byte count = 0;

  Wire.begin();
  for (byte i = 8; i < 120; i++)
  {
    Wire.beginTransmission (i);          // Begin I2C transmission Address (i)
    if (Wire.endTransmission () == 0)  // Receive 0 = success (ACK response) 
    {
      Serial.print ("Found address: ");
      Serial.print (i, DEC);
      Serial.print (" (0x");
      Serial.print (i, HEX);     // PCF8574 7 bit address
      Serial.println (")");
      count++;
    }
  }
  Serial.print ("Found ");      
  Serial.print (count, DEC);        // numbers of devices
  Serial.println (" device(s).");
}

//-------------------------------------------------------------------------------------------------------------------------------------------
// Vytvorenie jednotlivých funkcií pre pohyby
void PohybDopredu() {
  digitalWrite(MOTOR_ENABLE_PIN, LOW);
  // pohyb motora
  digitalWrite(MOTOR_LEFT_REAR, LOW);                  //dopredu // PROTI SMRERU
  digitalWrite(MOTOR_LEFT_FRONT, LOW);                  // dopredu
  digitalWrite(MOTOR_RIGHT_REAR, HIGH);                  // dopredu  // V SMERE
  digitalWrite(MOTOR_RIGHT_FRONT, HIGH);                  // dopredu

}

void PohybDozadu() {
  digitalWrite(MOTOR_ENABLE_PIN, LOW);
  // pohyb motora
  digitalWrite(MOTOR_LEFT_REAR, HIGH);                  // dozadu  // V SMERE 
  digitalWrite(MOTOR_LEFT_FRONT, HIGH);                  // dozadu
  digitalWrite(MOTOR_RIGHT_REAR, LOW);                  // dozadu  // PROTI SMERU
  digitalWrite(MOTOR_RIGHT_FRONT, LOW);                  // dozadu
}

void PohybKolmoDoprava() {
  digitalWrite(MOTOR_ENABLE_PIN, LOW);
  // pohyb motora
  digitalWrite(MOTOR_LEFT_REAR, LOW);                  // dopredu
  digitalWrite(MOTOR_LEFT_FRONT, HIGH);                  // dozadu
  digitalWrite(MOTOR_RIGHT_REAR, LOW);                  // dozadu
  digitalWrite(MOTOR_RIGHT_FRONT, HIGH);                  // dopredu
}

void PohybKolmoDolava() {
  digitalWrite(MOTOR_ENABLE_PIN, LOW);
  // pohyb motora
  digitalWrite(MOTOR_LEFT_REAR, HIGH);                  // dozadu
  digitalWrite(MOTOR_LEFT_FRONT, LOW);                  // dopredu
  digitalWrite(MOTOR_RIGHT_REAR, HIGH);                  // dopredu
  digitalWrite(MOTOR_RIGHT_FRONT, LOW);                  // dozadu
}

void PohybTocenieDolava() {
  digitalWrite(MOTOR_ENABLE_PIN, LOW);
  // pohyb motora
  digitalWrite(MOTOR_LEFT_REAR, HIGH);                  // dozadu
  digitalWrite(MOTOR_LEFT_FRONT, HIGH);                  // dozadu
  digitalWrite(MOTOR_RIGHT_REAR, HIGH);                  // dopredu
  digitalWrite(MOTOR_RIGHT_FRONT, HIGH);                  // dopredu
}

void PohybTocenieDoprava() {
  digitalWrite(MOTOR_ENABLE_PIN, LOW);
  // pohyb motora
  digitalWrite(MOTOR_LEFT_REAR, LOW);                  // dopredu
  digitalWrite(MOTOR_LEFT_FRONT, LOW);                  // dopredu
  digitalWrite(MOTOR_RIGHT_REAR, LOW);                  // dozadu
  digitalWrite(MOTOR_RIGHT_FRONT, LOW);                  // dozadu
}

// Vytvorenie funkcií pre spustenie senzorov
int UltrazvukovyPredny() {
  digitalWrite(TrigPin_predny, LOW);
  delayMicroseconds(2);
  // nadstavenie pinu na 20 mikrosekúnd
  digitalWrite(TrigPin_predny, HIGH);
  delayMicroseconds(20);
  digitalWrite(TrigPin_predny, LOW);
  delayMicroseconds(2);
  long trvanie_vlny_predny = pulseIn(EchoPin_predny, HIGH);
  int vzdialenost_predny = trvanie_vlny_predny*0.034/2;
  return vzdialenost_predny;
}

int UltrazvukovyPravy() {
  // vyčistenie pinu po dobu 2 mikrosekúnd
  digitalWrite(TrigPin_pravy, LOW);
  delayMicroseconds(2);
  // nadstavenie pinu na 20 mikrosekúnd
  digitalWrite(TrigPin_pravy, HIGH);
  delayMicroseconds(20);
  digitalWrite(TrigPin_pravy, LOW);
  delayMicroseconds(2);
  long trvanie_vlny_pravy = pulseIn(EchoPin_pravy, HIGH);
  int vzdialenost_pravy = trvanie_vlny_pravy*0.034/2;
  return vzdialenost_pravy;
}
int UltrazvukovyZadny() {
  // vyčistenie pinu po dobu 2 mikrosekúnd
  digitalWrite(TrigPin_zadny, LOW);
  delayMicroseconds(2);
  // nadstavenie pinu na 20 mikrosekúnd
  digitalWrite(TrigPin_zadny, HIGH);
  delayMicroseconds(20);
  digitalWrite(TrigPin_zadny, LOW);
  delayMicroseconds(2);
  long trvanie_vlny_zadny = pulseIn(EchoPin_zadny, HIGH);
  int vzdialenost_zadny = trvanie_vlny_zadny*0.034/2;
  return vzdialenost_zadny;
}

int meranie_laserom() {
  //VL53L0X_RangingMeasurementData_t vzdialenost_laser;             // Spustenie laseroveho senzora
  lox.rangingTest(&vzdialenost_laser, false);
  long measure_laser = vzdialenost_laser.RangeMilliMeter/10;
  return measure_laser;
}

//-------------------------------------------------------------------------------------------------------------------------------------------
// Funkcie pre autonomne parkovanie
void ManeverPozdlzneParkovanie() {
  Servomotor.write(0);                                             // Otocenie serva napravo
        
  Serial.print("Nieco... ");                                                                 
   
  int premenna_laser = meranie_laserom();
  Serial.print("Vzdialenost objektu laser: "); 
  Serial.print(premenna_laser);
  Serial.println(" cm      ");

  while (premenna_laser < 25) {                                      // Vozidlo pojde dopredu, kym nedetekuje miesto
    PohybDopredu();
    stepper.moveRelativeInSteps(10); 
     
    delay(10);
    
    premenna_laser = meranie_laserom();

    Serial.print("Vzdialenost objektu laser: "); 
    Serial.print(premenna_laser);
    Serial.println(" cm      ");
  }

  PohybDopredu();
  stepper.moveRelativeInSteps(315);                                 // Pojde dopredu s tym, ze ak to prejde a stale bude vidiet miesto tak moze zaparkovat

  premenna_laser = meranie_laserom();

  Serial.print("Vzdialenost objektu laser: "); 
  Serial.print(premenna_laser);
  Serial.println(" cm      ");

  //int aktualna_vzdialenost = premenna_laser;                        // Ulozenie pozicie v mieste kde skoncil pohyb dopredu, ak bude hodnota vhodna pre parkovanie tak tento ukon zacne

  int aktualna_vzdialenost = UltrazvukovyPravy();
  
  if (aktualna_vzdialenost > 28) {
    PohybDozadu();
    stepper.moveRelativeInSteps(30);                               // Pokial presiel 900 a stale je miesto tak ked sa vrati o 300 mal by sa zmestit cely do toho miesta
    
    delay(10);
    
    int napravo_vzd = UltrazvukovyPravy();

    int ulozena_vzdialenost = napravo_vzd;                      // Ulozenie pozicie z ktorej ide parkovat

    Serial.print("Vzdialenost objektu napravo: ");
    Serial.print(napravo_vzd);
    Serial.println(" cm      ");

    while (napravo_vzd > 0) {                                 // zaparkuje pokial pravy senzor nebude blizko pri stene
      PohybKolmoDoprava();
      stepper.moveRelativeInSteps(1);
      
      delay(10);

      napravo_vzd = UltrazvukovyPravy();

      Serial.print("Vzdialenost objektu napravo: ");
      Serial.print(napravo_vzd);
      Serial.println(" cm      ");

      if (napravo_vzd > 4 and napravo_vzd < 7) {
        break;
      }
    }
    
    int vpredu_vzd = UltrazvukovyPredny();

    Serial.print("Vzdialenost objektu vpredu: ");
    Serial.print(vpredu_vzd);
    Serial.println(" cm      ");

    int vzadu_vzd = UltrazvukovyZadny();
  
    Serial.print("Vzdialenost objektu vzadu: ");
    Serial.print(vzadu_vzd);
    Serial.println(" cm      ");

    while (vpredu_vzd != vzadu_vzd) {                                    // Vyrovnanie na mieste aby bol v strede parkovacieho miesta
      while (vpredu_vzd > vzadu_vzd) {
        PohybDopredu();
        stepper.moveRelativeInSteps(1); 
         
        delay(10);

        vpredu_vzd = UltrazvukovyPredny();

        Serial.print("Vzdialenost objektu vpredu: ");
        Serial.print(vpredu_vzd);
        Serial.println(" cm      ");

        vzadu_vzd = UltrazvukovyZadny();

        Serial.print("Vzdialenost objektu vzadu: ");
        Serial.print(vzadu_vzd);
        Serial.println(" cm      ");
      }

      while (vpredu_vzd < vzadu_vzd) {
        PohybDozadu();
        stepper.moveRelativeInSteps(1); 
         
        delay(10);

        vpredu_vzd = UltrazvukovyPredny();

        Serial.print("Vzdialenost objektu vpredu: ");
        Serial.print(vpredu_vzd);
        Serial.println(" cm      ");

        vzadu_vzd = UltrazvukovyZadny();

        Serial.print("Vzdialenost objektu vzadu: ");
        Serial.print(vzadu_vzd);
        Serial.println(" cm      ");
      }
    }
    
    Serial.println("Cakam");
    delay(3000);                                                                         // Pocka 3 sekundy
    
    napravo_vzd = UltrazvukovyPravy();

    Serial.print("Vzdialenost objektu napravo: ");
    Serial.print(napravo_vzd);
    Serial.println(" cm      ");

    while (napravo_vzd != ulozena_vzdialenost) {                                  // Vyparkovanie z miesta na ulozenu poziciu
      
      PohybKolmoDolava();
      stepper.moveRelativeInSteps(1);
      
      delay(10);

      napravo_vzd = UltrazvukovyPravy();

      Serial.print("Vzdialenost objektu napravo: ");
      Serial.print(napravo_vzd);
      Serial.println(" cm      ");

      Serial.print("Pozadovana vzdialenost: ");
      Serial.print(ulozena_vzdialenost);
      Serial.println(" cm      ");
    }

    PohybDopredu();                                                   // Ukoncenie pohybom dopredu
    stepper.moveRelativeInSteps(200);       
  }

  else {
    PohybDopredu();                                               // Pokial sa nezmesti do miesta tak bude pokracovat
    stepper.moveRelativeInSteps(200); 
    }
  }
//-------------------------------------------------------------------------------------------------------------------------------------------
void ManeverKolmeParkovanie() {
  Servomotor.write(0);    // Otocenie serva napravo

  Serial.println("Zaciatok... "); 
  
  int premenna_laser = meranie_laserom();               // Spustenie laseroveho senzora

  Serial.print("Vzdialenost objektu laser: "); 
  Serial.print(premenna_laser);
  Serial.println(" cm      ");
  
  while (premenna_laser < 30) {                                      // Pojde dopredu kym nedetekuje miesto
    PohybDopredu();
    stepper.moveRelativeInSteps(10);  
    
    delay(10);

    premenna_laser = meranie_laserom(); 

    Serial.print("Vzdialenost objektu laser: "); 
    Serial.print(premenna_laser);
    Serial.println(" cm      ");
  }

  PohybDopredu();                                                         // Pojde dopredu s tym, ze ak to prejde a stale bude vidiet miesto tak moze zaparkovat
  stepper.moveRelativeInSteps(246);   

  premenna_laser = meranie_laserom(); 

  Serial.print("Vzdialenost objektu laser: "); 
  Serial.print(premenna_laser);
  Serial.println(" cm      ");

  int aktualna_vzdialenost = premenna_laser;                               // Ulozenie hodnoty po predchadzajucom pohybe

  if (aktualna_vzdialenost > 30) {
    PohybDozadu();
    stepper.moveRelativeInSteps(10);                               // Vrati sa na zaciatok miesta

    PohybTocenieDoprava();                                          // Natocenie do parkovacieho miesta
    stepper.moveRelativeInSteps(250);

    PohybKolmoDolava();                                          // Natocenie do parkovacieho miesta
    stepper.moveRelativeInSteps(20);

    Servomotor.write(40);                                          // Uprava aby nenarazil kolesami do steny, otocenie servo motorom nad kolesa
    Serial.println("Po otoceni");

    premenna_laser = meranie_laserom(); 
    
    Serial.print("Vzdialenost objektu laser: "); 
    Serial.print(premenna_laser-10);
    Serial.println(" cm      ");
        
    while (premenna_laser-10 < 15) {                                      // Nad pravym kolesom
      PohybKolmoDolava();
      stepper.moveRelativeInSteps(20); 

      delay(10);

      premenna_laser = meranie_laserom();

      Serial.print("Idem Kolmo Dolava");
      Serial.print("Vzdialenost napravo: "); 
      Serial.print(premenna_laser-10);
      Serial.println(" cm      ");
    }

    Servomotor.write(140);  

    premenna_laser = meranie_laserom();
     
    while (premenna_laser-10 < 15) {                                        // Nad lavym kolesom
      PohybKolmoDoprava();
      stepper.moveRelativeInSteps(20); 

      delay(10);

      premenna_laser = meranie_laserom();

      Serial.print("Idem Kolmo Doprava");
      Serial.print("Vzdialenost nalavo: "); 
      Serial.print(premenna_laser-10);
      Serial.println(" cm      ");
    } 

    Servomotor.write(40); 
    premenna_laser = meranie_laserom();
    Serial.print("Konecne napravo: ");
    Serial.print(premenna_laser-10);
    Serial.println(" cm ");

    Servomotor.write(140); 
    premenna_laser = meranie_laserom();
    Serial.print("Konecne nalavo: ");
    Serial.print(premenna_laser-10);
    Serial.println(" cm ");
    
    int vpredu_vzd = UltrazvukovyPredny();
    Serial.print("Vzdialenost ultra vpredu: ");
    Serial.print(vpredu_vzd);
    Serial.println(" cm ");

    int ulozenie_vzdialenosti = vpredu_vzd;                       // Ulozenie miesta z ktoreho ide parkovat
    
    while (vpredu_vzd > 0) {                                  // pohyb dopredu do parkovacieho, pokial nebude blizko steny
      PohybDopredu();
      stepper.moveRelativeInSteps(1);  
         
      delay(10); 

      vpredu_vzd = UltrazvukovyPredny();
      Serial.print("Vzdialenost ultra vpredu: ");
      Serial.print(vpredu_vzd);
      Serial.println(" cm ");

      if (vpredu_vzd > 0 and vpredu_vzd < 4) {
        break;
      }
    }

    delay(3000);                                                               // Pocka 3 sekundy  

    PohybDozadu();
    stepper.moveRelativeInSteps(300);
 
    PohybTocenieDolava();                                                     // Otocenie dolava
    stepper.moveRelativeInSteps(250);

    PohybDopredu();
    stepper.moveRelativeInSteps(200);                                          // Po vyparkovani pokracuje smerom dopredu
  }
  
  else {
    PohybDopredu();                                                             // Pokial sa nevojde do parkovacieho tak bude pokracovat dopredu
    stepper.moveRelativeInSteps(200); 
  } 
}
//-------------------------------------------------------------------------------------------------------------------------------------------
// Zaciatok sluciek
void loop() {
  if(counter >= 10){
    Serial.println("zacinam loop");
    counter = 0;
  }
  
  while (SerialBT.available()) {
    c = SerialBT.read();
    if (c != '.') {
      slovo += c;
    }
    else {
      Serial.println(slovo);
      pokyn = slovo;
      slovo = "";
    }
   }
    
// Po prijati zvoleneho pokynu z aplikacie sa prislusny pohyb vykona       
    if (pokyn == "dopredu") {
      PohybDopredu();
      stepper.moveRelativeInSteps(25);    
    }
    if (pokyn == "dozadu") {
      PohybDozadu();
      stepper.moveRelativeInSteps(25);
    }
    if (pokyn == "doprava") {
      PohybKolmoDoprava();
      stepper.moveRelativeInSteps(25);
    }
    if (pokyn == "dolava") {
      PohybKolmoDolava();
      stepper.moveRelativeInSteps(25);
    }
    if (pokyn == "toceniedoprava") {
      PohybTocenieDoprava();
      stepper.moveRelativeInSteps(1);
    }
    if (pokyn == "toceniedolava") {
      PohybTocenieDolava();
      stepper.moveRelativeInSteps(1);
    }
    if (pokyn == "kolmeparkovanie") {
      ManeverKolmeParkovanie();
    }
    if (pokyn == "pozdlzneparkovanie") {
      ManeverPozdlzneParkovanie();
    }
    if (pokyn == "stop") {                                          // ukoncenie pohybu
      digitalWrite(MOTOR_ENABLE_PIN, HIGH);
    }
    if (pokyn == "adresa") {
      Scanner ();
    }
    counter++;
  //delay(500);
}
