//--IMU 6DoF I2C
//--Accelerometre = ADXL345
//--Gyroscope = ITG3200
//--Fusion de l'accel et du gyro avec Kalman
//--Integration des servos en X et Y
//--valeur servo (Datasheet Hitech HS-485B): 
// -90° = 900us
//   0° = 1500us
// +90° = 2100us

#include <Wire.h> // communication I2C
#include <Servo.h> // commande servos

// declaration des servos (X,Y)
//-----------------------------

Servo servo_X;
Servo servo_Y;
Servo servo_Z;

// declaration des positions des servos (X,Y)
//-------------------------------------------

int pos_X = 0;
int pos_Y = 0;
int pos_Z = 90;

// vecteur 3 axes par capteurs
//----------------------------

int Gyro_out[3],Accel_out[3];

// base de temps pour l'acquisition des signaux
//---------------------------------------------

float dt = 0.02; 

// declaration des variables
//--------------------------

float Gyro_cal_x,Gyro_cal_y,Gyro_cal_z,Accel_cal_x,Accel_cal_y,Accel_cal_z;

// valeur d'initialisation axe X
//------------------------------

float Gyro_X = 0;
float Accel_X = 0;
float Predicted_X = 0; // sortie du filtre de Kalman X

// valeur d'initialisation axe Y
//------------------------------

float Gyro_Y = 0;
float Accel_Y = 0;
float Predicted_Y = 0; // sortie du filtre de Kalman Y

// prediction des etats initiaux (a modifier si besoin)
//-----------------------------------------------------

float Q = 2.5; // bruit de covariance
float R = 5; // bruit de mesure gaussien

// matrice de covariance des erreurs (erreurs estimees non nulles)
//----------------------------------------------------------------

float P00 = 0.1; 
float P11 = 0.1; 
float P01 = 0.1; 

// declaration des gains de Kalman
//--------------------------------

float Kk0, Kk1; 

// declaration des unites de temps
//--------------------------------

unsigned long timer;
unsigned long time;

// fonction ecriture I2C librairie "wire.h"
//-----------------------------------------

void writeTo(byte device, byte toAddress, byte val) 
{
  Wire.beginTransmission(device);
  Wire.write(toAddress);
  Wire.write(val);
  Wire.endTransmission();
}

// fonction lecture I2C librairie "wire.h"
//----------------------------------------

void readFrom(byte device, byte fromAddress, int num, byte result[]) 
{
  Wire.beginTransmission(device);
  Wire.write(fromAddress);
  Wire.endTransmission();
  Wire.requestFrom((int)device, num);
  
  // condition de test pour la lecture I2C
  //--------------------------------------

  int i = 0;
  
  while(Wire.available())
  {
    result[i] = Wire.read();
    i++;
  }
}

//datasheet lecture gyroscope I2C ITG3200
//---------------------------------------

void getGyroscopeReadings(int Gyro_out[]) 
{
  byte buffer[6];
  readFrom(0x68,0x1D,6,buffer); 
  
  Gyro_out[0]=(((int)buffer[0]) << 8 ) | buffer[1];
  Gyro_out[1]=(((int)buffer[2]) << 8 ) | buffer[3];
  Gyro_out[2]=(((int)buffer[4]) << 8 ) | buffer[5];
}

//datasheet lecture accelerometre I2C ADXL345
//-------------------------------------------

void getAccelerometerReadings(int Accel_out[]) 
{
  byte buffer[6];
  readFrom(0x53,0x32,6,buffer); 
  
  Accel_out[0]=(((int)buffer[1]) << 8 ) | buffer[0];
  Accel_out[1]=(((int)buffer[3]) << 8 ) | buffer[2];
  Accel_out[2]=(((int)buffer[5]) << 8 ) | buffer[4];
}

// routine de calibration
//-----------------------

void setup()
{
  
  servo_X.attach(12); // plug le servo_X sur pin 10 (a changer si besoin)
  servo_Y.attach(11); // plug le servo_Y sur pin 11 (a changer si besoin)
  servo_Z.attach(10);

  int Gyro_cal_x_sample = 0;
  int Gyro_cal_y_sample = 0;
  int Gyro_cal_z_sample = 0;
  
  int Accel_cal_x_sample = 0;
  int Accel_cal_y_sample = 0;
  int Accel_cal_z_sample = 0;
  
  int i;

  delay(5);

  // configuration du port serie
  //----------------------------

  Wire.begin();
  Serial.begin(115200); // vitesse a 115200 bds
 
  // calibration des capteurs selon les datasheets constructeurs
  //------------------------------------------------------------

  writeTo(0x53,0x31,0x09); //accel 11 bits - +/-4g
  writeTo(0x53,0x2D,0x08); //accel en mode mesure
  writeTo(0x68,0x16,0x1A); //gyro +/-2000 deg/s + passe-bas a 100Hz
  writeTo(0x68,0x15,0x09); //gyro echantillonage a 100Hz
  
  delay(100);

  for(i = 0;i < 100;i += 1)
  {

    // lecture des sorties capteurs
    //-----------------------------

    getGyroscopeReadings(Gyro_out);
    getAccelerometerReadings(Accel_out);

    // acquisition du gyro sur les 3 axes
    //-----------------------------------
    
    Gyro_cal_x_sample += Gyro_out[0]; 
    Gyro_cal_y_sample += Gyro_out[1]; 
    Gyro_cal_z_sample += Gyro_out[2];  
    
    // acquisition de l'accel sur les 3 axes
    //--------------------------------------

    Accel_cal_x_sample += Accel_out[0]; 
    Accel_cal_y_sample += Accel_out[1]; 
    Accel_cal_z_sample += Accel_out[2]; 
    
    delay(50);
  }
  
  // lecture du gyro sur les 3 axes 
  //-------------------------------

  Gyro_cal_x = Gyro_cal_x_sample / 100;
  Gyro_cal_y = Gyro_cal_y_sample / 100;
  Gyro_cal_z = Gyro_cal_z_sample / 100;

  // lecture de l'accel sur les 3 axes
  //----------------------------------
  
  Accel_cal_x = Accel_cal_x_sample / 100;
  Accel_cal_y = Accel_cal_y_sample / 100;
  Accel_cal_z = (Accel_cal_z_sample / 100) - 256; //code sur 8 bits (256LSB) pour annuler la gravite => 0 = -256
}

// programme principal
//--------------------

void loop()
{
  timer = millis();

  // lecture discrete (loop) des sorties capteurs
  //---------------------------------------------

  getGyroscopeReadings(Gyro_out);
  getAccelerometerReadings(Accel_out);
  
  // equation du mouvement selon l'axe des X
  //----------------------------------------

  Accel_X = atan2((Accel_out[1] - Accel_cal_y) / 256,(Accel_out[2] - Accel_cal_z)/256) * 180 / PI; 
  Gyro_X = Gyro_X + ((Gyro_out[0] - Gyro_cal_x)/ 14.375) * dt; 

  // lecture de -90°/+90° sur X
  //---------------------------

  if(Gyro_X < 180) Gyro_X += 360; 
  if(Gyro_X >= 180) Gyro_X -= 360; 

  // mise a jour de la matrice de prediction sur X (time update phase 1)
  //------------------------------------------------------------------------

  Predicted_X = Predicted_X + ((Gyro_out[0] - Gyro_cal_x)/14.375) * dt; 

  // equation du mouvement selon l'axe des Y
  //----------------------------------------

  Accel_Y = atan2((Accel_out[0] - Accel_cal_x) / 256,(Accel_out[2] - Accel_cal_z)/256) * 180 / PI; 
  Gyro_Y = Gyro_Y + ((Gyro_out[1] - Gyro_cal_y)/ 14.375) * dt; 
   
  // lecture de -90°/+90° sur Y
  //---------------------------

  if(Gyro_Y < 180) Gyro_Y += 360; 
  if(Gyro_Y >= 180) Gyro_Y -= 360; 

  // mise a jour de la matrice de prediction sur Y (time update phase 1)
  //------------------------------------------------------------------------

  Predicted_Y = Predicted_Y - ((Gyro_out[1] - Gyro_cal_y)/14.375) * dt; 
  
  // erreur de covariance donne par les resultats de derivation (time update phase 2)
  //---------------------------------------------------------------------------------

  P00 += dt * (2 * P01 + dt * P11); 
  P01 += dt * P11; 
  P00 += dt * Q; 
  P11 += dt * Q; 
  
  // mise a jour des gains de Kalman (measure update phase 1)
  //---------------------------------------------------------

  Kk0 = P00 / (P00 + R); 
  Kk1 = P01 / (P01 + R); 

  // mise a jour du filtre de Kalman (measure update phase 2)
  //---------------------------------------------------------

  Predicted_X += (Accel_X - Predicted_X) * Kk0; 
  Predicted_Y += (Accel_Y - Predicted_Y) * Kk0; 

  // mise a jour de la matrice de covariance des erreurs (measure update phase 3)
  //-----------------------------------------------------------------------------

  P00 *= (1 - Kk0);
  P01 *= (1 - Kk1);
  P11 -= Kk1 * P01;

  float angle_z = Gyro_out[2];

  time = millis();

   // conversion et mise a l'echelle angle/temps pour les servos
   //-----------------------------------------------------------

   pos_X = map(Predicted_X, -90, 90, 900, 2200); // servo_X
   servo_X.write(pos_X);

   pos_Y = map(Predicted_Y, 90, -90, 900, 2200); // servo_Y
   servo_Y.write(pos_Y);
  
  /*
  // affichage des resultats sur port serie (115200 bds)
  //----------------------------------------------------

  Serial.print("Predicted_X: "); //affichage X en angle d'Euler
  Serial.print(Predicted_X);
  Serial.print(" ");
  Serial.print("Predicted_Y: "); //affichage Y en angle d'Euler
  Serial.print(Predicted_Y);
  Serial.println(" ");
  Serial.print("Pos_X: "); // affichage valeur servo X 
  Serial.print(pos_X);
  Serial.print(" ");
  Serial.print("Pos_Y: "); //affichage valeur servo Y
  Serial.print(pos_Y);
  Serial.print(" ");
  */
  
  timer = millis() - timer; //base de temps en millisecondes
  timer = (dt * 1000) - timer; //mise a jour de la base de temps
  
  delay(timer);
}
