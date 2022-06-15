#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <Carro.h>
#include <Encoder.h>
#include <analogWrite.h>
#include <VL53L0X.h>
#include <Mux.h>
//----------------------------------------------------------------------------------------
long randomGiro;
long randomIzqDer;
//----------------------------------------------------------------------------------------
//Variables para accionamiento de motor y encoder.
Encoder Rizq(18,19); //(18, 19)
Encoder Rder(5,23);  //(5, 23)
Encoder RizqAux(33,15);
Encoder RderAux(34,35);
//Variables de driver, A --> Motor derecho, B --> Motor izquierdo.
#define AIN1 16 // 16
#define AIN2 27 // 27
#define PWM_A 17 //17
#define BIN1 13 // 13
#define BIN2 12 // 12
#define PWM_B 14 //14
#define RESOLUCION 1023 //Rango de valores de 0 a 1023.
//Variables de distancia:
#define Res_encoder 2800 // La resolución real es de 2800 pulsos por una vuelta. Hay un cambio aquí por las interrupciones y el modo de recibir la Interrupción
#define radio_rueda 2.15
//#define radio_chasis 5.95 // Esto puede variar por cada robot (ruedas  grades).
//#define radio_chasis 5.85//5.75//5.95
#define radio_chasis 5.75//5.85 //6
float res;
//Variables para movimiento
#define STBY 9
const int offsetA = 1;
const int offsetB = 1;
int nuevaDer = 0, nuevaIzq = 0, pul_der = 0, nuevaIzqAux = 0, nuevaDerAux = 0;

//Variables para controlador PI
double dif_vel =0 ;
//Variables de lectura de pulsos del encoder.
volatile long pulsos_der = 0;
volatile long pulsos_izq = 0;
//Variables de tiempo y tiempo de muestreo.
unsigned long LastTime = 0;
unsigned long InitTime;
//Variables controlador PI velocidad lineal.
double vel_lineal = 0.0, vel_lineal_d = 0.0;
double I_vel = 0.0, Last_vel = 0.0;
double kpVel = 0.0, kiVel = 0.0, kdVel = 0.0;
double PWMmin = 0.0, PWMmax = 0.0; 
double errorVel = 0.0 , P_vel = 0.0;
double D_vel = 0.0;
//Variables controlador PI velocidad angular.
double vel_angular = 0.0, vel_ang_d = 0.0;
double I_ang = 0.0, Last_ang = 0.0;
double kpAng = 0.0, kiAng = 0.0, kdAng = 0.0;
double errorAng = 0.0, P_ang = 0.0;
double D_ang = 0.0;
//Variables globales
double Volt_R = 0.0, Volt_L = 0.0;
//byte PWM2 = 0, PWM1 = 0;

double uR = 0.0, uL = 0.0; //Velocidades lineales de cada llanta.
double PID_vel = 0.0, PID_ang = 0.0; //Señales de control para cada Vel.
const double alfa = 0.9;
const double alfaw = 0.1;
const double Const = 0.048245, distRuedas = 0.119;//0.119;
//Variables de distancias
//int nuevaDer = 0, nuevaIzq = 0, pul_der = 0, nuevaIzqAux = 0, nuevaDerAux = 0;
int suma = 0, resul = 0;
float Dist_rec = 0, dist_m = 0, Dist = 0, Dist_recAux = 0;
float tiempo = 0;
unsigned long TTime;
float ProPulsos = 0;
int datoCal = round((2*PI)*radio_chasis*1000); // 37384.95258 --> 37385
int datoGiro = round((PI/2)*radio_chasis*1000); //9346.238144 --> 9346

// oofsetX no se utiliza
Motor motor1 = Motor(AIN1, AIN2, PWM_A, offsetA, STBY); // Motor derecha
Motor motor2 = Motor(BIN1, BIN2, PWM_B, offsetB, STBY); // Motor izquierda

//----------------------------------------------------------------------------------------
uint8_t broadcastAddress[] = {0x24, 0x6F, 0x28, 0x24, 0xD0, 0x20};

//uint8_t broadcastAddress[] = {0x24, 0x6F, 0x28, 0x24, 0xD0, 0x20};
//24:6F:28:24:D0:20
String varR2;
String Exito = "Éxito 2";
String Error = "Error 2";
String Vacio = " ";
//String Encontrado = "Blanco encontrado";
// Variables de recepción de datos
String incomingPlaca2; // Mensaje de la placa 1
String incomingColorFront2, incomingColorTras2, incomingEstado2;//, incomingEncontrado2;
String incomingEncontrado2;
float incomingSenDist2;
int incomingconect2 = 0; 
int incomingR2 = 0;
int ContNOW2 = 0;
int contador2 = 0; // Valor de i
String success2;

typedef struct struct_message {
    char Placa2[20];
    float SenDist2;
    int conect2, R2, L2;
    String ColorFront2, ColorTras2, Estado2, EO2, EO3;
} struct_message;

struct_message myData;

struct_message incomingReadings;

esp_now_peer_info_t peerInfo;
//----------------------------------------------------------------------------------------
//Variables de dirección en controlador PI.
int Frente  = 1;
int Atras   = 2;
int GiroIzq = 3;
int GiroDer = 4;
//----------------------------------------------------------------------------------------
//Variables función Colores
String Color_front;
String Color_tras;
String Blanco = "Blanco";
String Gris   = "Gris";
String Negro  = "Negro";
String EncontradoB = "encontrado";
int lectura_color_front = 0,lectura_color_trase = 0, lectura_Calibracion = 0;
int lectura_color_front_C = 0, lectura_color_trase_C = 0;
//int Max_Calibracion = 0;
//int Min_Calibracion = 1000;
//---------------------------- Agregado
int Min_Calibracion_fron = 1000;
int Max_Calibracion_fron = 0;
int Min_Calibracion_tras = 1000;
int Max_Calibracion_tras = 0;
//----------------------------
String Encontrado2, Encontrado3; 
String frontalTres[9] = {}, trasTres[9] = {}; 
String frontalCuatro[16] = {}, trasCuatro[16] = {};
int count, sum, j2, i;
//----------------------------------------------------------------------------------------
// Variables Multiplexor.
using namespace admux;

Mux mux(Pin(39, INPUT, PinType::Analog), Pinset(25, 26, 2, 4));
//int PWMBlancoMax = 0,PWMBlancoMin = 0, PWMNegroMax = 0, PWMNegroMin = 0, PWMGrisMax = 0, PWMGrisMin = 0;
//int PWMBlancoMaxG = 0,PWMBlancoMinG = 0, PWMNegroMaxG = 0, PWMNegroMinG = 0, PWMGrisMaxG = 0, PWMGrisMinG = 0;
int PWMBlancoMaxG_T = 0,PWMBlancoMinG_T = 0,PWMNegroMaxG_T = 0, PWMNegroMinG_T = 0, PWMGrisMaxG_T = 0, PWMGrisMinG_T = 0;
int PWMBlancoMaxG_F = 0,PWMBlancoMinG_F = 0,PWMNegroMaxG_F = 0, PWMNegroMinG_F = 0, PWMGrisMaxG_F = 0, PWMGrisMinG_F = 0;
//----------------------------------------------------------------------------------------
// Variables sensor de distancia
VL53L0X sensor; //Se define segun la libreria para inicializar el sensor
float distanciaMili = 0;
//----------------------------------------------------------------------------------------
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nEstado de envío del último paquete:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Entrega correcta" : "Entrega incorrecta");
  if (status ==0){
    success2 = Exito;
  }
  else{
    success2 = Error;
  }
}
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&incomingReadings, incomingData, sizeof(incomingReadings));
  Serial.print("Bytes received: ");
  Serial.println(len);
  incomingPlaca2 = incomingReadings.Placa2;
  incomingSenDist2 = incomingReadings.SenDist2;
  incomingconect2 = incomingReadings.conect2;
  incomingR2 = incomingReadings.R2; // j //contador
  incomingColorFront2 = incomingReadings.ColorFront2;
  incomingColorTras2 = incomingReadings.ColorTras2;
  incomingEstado2 = incomingReadings.Estado2;
  incomingEncontrado2 = incomingReadings.EO2;
  //incomingEncontrado2 = incomingReadings.EO3;
}
void Datos(){
  strcpy(myData.Placa2 , "Placa Emisora 2");
  myData.SenDist2 = distanciaMili;//ContNOW2;//distanciaMili;//88.32;
  myData.conect2 = contador2;//ContNOW;//random(2,9); // Este valor sera el ConNOW para comparar en el otro carro
  myData.R2 = j2;//i;//contador;//16;//contador
  myData.ColorFront2 = Color_front;//"Gris";
  myData.ColorTras2 = Color_tras;//"Cafe";
  myData.Estado2 = success2;
  myData.EO2 = Encontrado2; // Blanco
  myData.EO3 = Encontrado3; // Encontrado

  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
   
  if (result == ESP_OK) {
    varR2 = Exito;//"Éxito en la entrega desde R1";
  }
  else {
    varR2 = Error;//"Error en la entrega desde R1";
  }
  delay(50); //60
}
//xftxftjcxftjct
//----------------------------------------------------------------------------------------
void SETUP_SENSOR_DISTANCIA(){
  Wire.begin();

  sensor.setTimeout(500);
  if (!sensor.init())
  {
    Serial.println("¡Falla en la inicializacion del sensor Distancia!");
    while (1) {}
  }

  sensor.startContinuous(); //Entrega datos continuos en milisegundos (100)
  }
void SENSOR_DISTANCIA(){
    distanciaMili = sensor.readRangeContinuousMillimeters();//0.1*sensor.readRangeContinuousMillimeters() - 3.0;
  if (sensor.timeoutOccurred()) {  //Si el sensor falla arroja a pantalla TIMEOUT
    Serial.print(" TIMEOUT"); 
  }
}
void ControlPI(int pulsos_izq, int pulsos_der, double vel_lineal_d, double vel_ang_d, int direccion)
{
  if(millis() - LastTime >= InitTime)
    {
      if(direccion == 2 || direccion == 3 || direccion == 4){
        pulsos_izq = abs(nuevaIzq);
        pulsos_der = abs(nuevaDer);
      }
      else if(direccion == 1){
        pulsos_izq = nuevaIzq;
        pulsos_der = nuevaDer;
      }
      //Obtenemos las velocidades lineales de cada llanta
      uR = (0.048245*pulsos_der)/(millis() - LastTime); //Se multiplica x 4 por la resolución real. 700*4 = 2800 pulsos en una vuelta después del motorreductor.
      uL = (0.048245*pulsos_izq)/(millis() - LastTime);
      LastTime = millis();
      //Reset a variables para el cambio en cada 100 milisegundos.
      pulsos_der =0;
      pulsos_izq =0;
      Rder.write(0);
      Rizq.write(0);
      
      //Utilizamos las ecuaciones cinemáticas de nuestro robot diferencial.
      vel_lineal = alfa*((uR + uL)/2.0) + (1.0 - alfa)*Last_vel;
      vel_angular = alfaw*((uR - uL)/distRuedas)+(1.0 - alfaw)*Last_ang;
      //Calculo de los errores:
      errorVel = vel_lineal_d - vel_lineal;
      errorAng = vel_ang_d - vel_angular;
      //Calculo proporcional:
      P_vel = kpVel*errorVel; 
      P_ang = kpAng*errorAng;
      //Calculo integral:
      I_vel += kiVel*P_vel; //I_vel = I_vel + kiVel*P_vel;
      I_ang += kiAng*P_ang; //I_ang = I_ang + kiAng*P_ang;
      //Acotamos el error integral para eliminar el "efecto windup".
      if(I_vel > PWMmax) I_vel = PWMmax; else if(I_vel < PWMmin) I_vel = PWMmin;
      if(I_ang > PWMmax) I_ang = PWMmax; else if(I_ang < PWMmin) I_ang = PWMmin;
      //Calculo derivativo:
      double vInput = (vel_lineal - Last_vel);
      double aInput = (vel_angular- Last_ang);
      D_vel = kdVel*vInput;
      D_ang = kdAng*aInput;
      //Obtenemos la señal de control u = kp*e + kd*dedt + ki*eintegral
      PID_vel = P_vel + I_vel - D_vel;
      PID_ang = P_ang + I_ang - D_ang;

      Volt_R = PID_vel + PID_ang;
      Volt_L = PID_vel - PID_ang;
      
      //Acotamos las salidas para que el PWM este dentro de los valores max. y min.
      if(Volt_R > PWMmax) Volt_R = PWMmax; else if(Volt_R < PWMmin) Volt_R = PWMmin;
      if(Volt_L > PWMmax) Volt_L = PWMmax; else if(Volt_L < PWMmin) Volt_L = PWMmin;
      // Guardamos los valores
      Last_vel = vel_lineal;
      Last_ang = vel_angular;
      //Llamamos a la función motor para mover los motores a un valor PWM dado por Volt_R y Volt_L.
      //motor(Volt_L, Volt_R);
      dif_vel = abs(Volt_R - Volt_L) ;
    }
    if(direccion == 1){ // 1 -- > Avanza hacia el frente , 2 --> Avanza hacia atras
          /* Con la dirección "1": Podemos aplicar una velocidad angular, con las demás opciones no es posible.
           * Para girar a la derecha con velocidad angular, esta debe ser negativa. vel_and_d = - x.x
           * Para girar a la izquierda con velocidad angular, esta debe ser positiva. vel_ang_d = x.x
           */
          motor1.delante(Volt_R); motor2.delante(Volt_L);
        }
        if(direccion == 2){
          /* Solo con velocidad lineal y velocidad angular 0.0
           *  Al igual que las direcciones 3 y 4.
           * ControlPI(abs(nuevaIzq), abs(nuevaDer), 0.1, 0.0, 2); // De esta forma funciona la dirección hacia atrás. 
           */
          motor1.atras(Volt_R); motor2.atras(Volt_L);
        }
        if(direccion == 3){ // Izquierda
          //ControlPI(abs(nuevaIzq), abs(nuevaDer), 0.1, 0.0, 3); // De esta forma funciona la dirección hacia izquierda.
        //Izquierda(motorIzquierda, motorDerecha, Vel. Derecha, Vel. izquierda);
          //izquierda(motor2,motor1, Volt_R, Volt_L);
          motor1.delante(Volt_R);motor2.atras(Volt_L);
        }
        if(direccion == 4){ //Derecha
          //ControlPI(abs(nuevaIzq), abs(nuevaDer), 0.1, 0.0, 4);
          motor1.atras(Volt_R);motor2.delante(Volt_L);
        }
        if (direccion == 5) //Freno
        {
          Volt_R = 0;
          Volt_L = 0;
          motor1.delante(Volt_R); motor2.delante(Volt_L);
        }
        // if (direccion == 8) // ----> CAMBIO 
        // {
        //   motor1.delante(Volt_R);motor2.atras(Volt_L);
        // }
        // if (direccion == 9) // ----> CAMBIO 
        // {
        //   motor1.atras(Volt_R);motor2.delante(Volt_L);
        // }
        
   //Serial.print(uR,3);Serial.print('\t');Serial.print(uL,3);Serial.print('\t');Serial.print(dif_vel,3);Serial.print('\t');Serial.println(vel_lineal,3);  
}
void CalibracionInalambrica(){ 
  int canal_0=mux.read(0); // Canal 1
  int canal_1=mux.read(1); // Canal 2
  int canal_2=mux.read(2); // Canal 3
  int canal_3=mux.read(3); // Canal 4
  int canal_4=mux.read(4); // Canal 5
  int canal_5=mux.read(5); // canal 6
  int canal_6=mux.read(6); // canal 7
  int canal_7=mux.read(7); // Canal 8    
  int canal_8=mux.read(8); // Canal 9
  int canal_9=mux.read(9); // Canal 10
  int canal_10=mux.read(10); // Canal 11
  int canal_11=mux.read(11); // Canal 12
  int canal_12=mux.read(12); // Canal 13
  int canal_13=mux.read(13); // Canal 14
  int canal_14=mux.read(14); // Canal 15
  int canal_15=mux.read(15); // Canal 16
  //int canales[16] = {canal_0,canal_1,canal_2,canal_3,canal_4,canal_5,canal_6,canal_7,canal_8,canal_9,canal_10,canal_11,canal_12,canal_13,canal_14,canal_15};
  int sumaFront = 0,sumaTrase = 0, promedioFront = 0, promedioTrase = 0;
  //suma = canal_0+canal_1+canal_2+canal_3+canal_4+canal_5+canal_6+canal_7+canal_8+canal_9+canal_10+canal_11+canal_12+canal_13+canal_14+canal_15;
  //promedio = suma/16;
  sumaFront = canal_0+canal_1+canal_2+canal_3+canal_4+canal_5+canal_6+canal_7;
  sumaTrase = canal_8+canal_9+canal_10+canal_11+canal_12+canal_13+canal_14+canal_15;
  promedioFront = sumaFront/8;
  promedioTrase = sumaTrase/8;
  float Const = 0.24421; //0.244200311
  lectura_color_front_C = Const*promedioFront;
  lectura_color_trase_C = Const*promedioTrase;
  if(lectura_color_front_C > Max_Calibracion_fron){ // Valor inicial 0
    Max_Calibracion_fron = lectura_color_front_C;
    }
  if(lectura_color_front_C < Min_Calibracion_fron){ // Valor inicial 1000
    Min_Calibracion_fron = lectura_color_front_C;
    }
  if(lectura_color_trase_C > Max_Calibracion_tras){ // Valor inicial 0
    Max_Calibracion_tras = lectura_color_trase_C;
    }
  if(lectura_color_trase_C < Min_Calibracion_tras){ // Valor inicial 1000
    Min_Calibracion_tras = lectura_color_trase_C;
    } 
  /*lectura_Calibracion = Const*promedio;
  if(lectura_Calibracion > Max_Calibracion){ // Valor inicial 0
    Max_Calibracion = lectura_Calibracion;
    }
  if(lectura_Calibracion < Min_Calibracion){ // Valor inicial 1000
    Min_Calibracion = lectura_Calibracion;
    } */
}
void Colores(int PWMBlancoMax_F,int PWMBlancoMax_T, int PWMNegroMin_F, int PWMNegroMin_T, int PWMGrisMin_F, int PWMGrisMax_F,int PWMGrisMin_T, int PWMGrisMax_T)
{
    int canal_0=mux.read(0); // Canal 1
    int canal_1=mux.read(1); // Canal 2
    int canal_2=mux.read(2); // Canal 3
    int canal_3=mux.read(3); // Canal 4
    int canal_4=mux.read(4); // Canal 5
    int canal_5=mux.read(5); // canal 6
    int canal_6=mux.read(6); // canal 7
    int canal_7=mux.read(7); // Canal 8    
    int canal_8=mux.read(8); // Canal 9
    int canal_9=mux.read(9); // Canal 10
    int canal_10=mux.read(10); // Canal 11
    int canal_11=mux.read(11); // Canal 12
    int canal_12=mux.read(12); // Canal 13
    int canal_13=mux.read(13); // Canal 14
    int canal_14=mux.read(14); // Canal 15
    int canal_15=mux.read(15); // Canal 16

    int sumaFront = 0,sumaTrase = 0, promedioFront = 0, promedioTrase = 0;
  
    sumaFront = canal_0+canal_1+canal_2+canal_3+canal_4+canal_5+canal_6+canal_7;
    sumaTrase = canal_8+canal_9+canal_10+canal_11+canal_12+canal_13+canal_14+canal_15;
    promedioFront = sumaFront/8;
    promedioTrase = sumaTrase/8;
    float Const = 0.24421; // 1000/4095
    lectura_color_front = Const*promedioFront;
    lectura_color_trase = Const*promedioTrase;

    //Delanteros ---------------------------------------------------------------------
    if(lectura_color_front > PWMGrisMin_F){
      if (lectura_color_front < PWMGrisMax_F) {
        Color_front = Gris;
        Color_tras = Gris;
      }
    }  //POSIBLEMENTE DEBA EDITAR !!
    if(lectura_color_front < PWMBlancoMax_F){
      Color_front = Blanco;
    }
    if(lectura_color_front > PWMNegroMin_F){
      Color_front = Negro;
    }
    //Trasero ------------------------------------------------------------------------
    // if(lectura_color_trase >PWMGrisMin_F){
    //   if(lectura_color_trase < PWMGrisMax_T){
    //    Color_tras = Gris;
    //   }
    // } 
    if(lectura_color_trase < PWMBlancoMax_T){
      Color_tras = Blanco;
    }
    if(lectura_color_trase > PWMNegroMin_T){
      Color_tras = Negro;
    }     
  //Serial.println();
  delay(30);
}
void giro(int direccion){
  nuevaDer = Rder.read();
  nuevaIzq = Rizq.read();
  ControlPI(nuevaIzq, nuevaDer, 0.05, 0.0, direccion); //Velocidad solo para el giro.
  ProPulsos = (abs(RderAux.read()) + abs(RizqAux.read()))/2;
  Dist_rec = (2*PI*ProPulsos*radio_rueda)/2800;
  Dist_recAux = round(Dist_rec*1000);
}
//----------------------------------------------------------------------------------------
void Avance1(){
  SENSOR_DISTANCIA();
  RizqAux.write(0);
  RderAux.write(0);
  while (Dist_recAux < 27000 && sensor.readRangeContinuousMillimeters() > 120/*27.00 - 0.16*/)
  {
    nuevaDer = Rder.read();
    nuevaIzq = Rizq.read();
    ControlPI(nuevaIzq, nuevaDer, 0.05, 0.0, Frente);
    ProPulsos = (abs(RderAux.read()) + abs(RizqAux.read()))/2;
    Dist_rec = (2*PI*radio_rueda*ProPulsos)/2800;
    Dist_recAux = round(Dist_rec*1000);
    Colores(PWMBlancoMaxG_F, PWMBlancoMaxG_T, PWMNegroMinG_F, PWMNegroMinG_T,PWMGrisMinG_F, PWMGrisMaxG_F, PWMGrisMinG_T, PWMGrisMinG_T); 
    SENSOR_DISTANCIA();
    Datos();
    //EnvioDatos(i);
  }
    //Preguntar por el color
  //dist_m = dist_m + Dist_rec;
  Dist_rec = 0;
  Dist_recAux = 0;
  randomGiro = random(3,5); // -1 a la derecha, 1 a la izquierda
  while (Dist_recAux < datoGiro - 40/*&& distanciaMili  > 120*/) //1937 - 40 = 1897pulsos
  {
    giro(randomGiro);
    Colores(PWMBlancoMaxG_F, PWMBlancoMaxG_T, PWMNegroMinG_F, PWMNegroMinG_T,PWMGrisMinG_F, PWMGrisMaxG_F, PWMGrisMinG_T, PWMGrisMinG_T); 
    SENSOR_DISTANCIA();         
    Datos();
    //EnvioDatos(i);
  }  
  //nuevaDer = Rder.read();
  //nuevaIzq = Rizq.read();
  //ControlPI(nuevaIzq, nuevaDer, 0.0, 0.0, 5);
  motor1.freno();motor2.freno();
  delay(500);
}
void Avance2(float DistRequerida){
  j2 = j2 +1;
  Datos();//Datos(i); 
  SENSOR_DISTANCIA();
  RizqAux.write(0);
  RderAux.write(0);
  // DistRequerida == 27.00
  while (Dist_recAux <= DistRequerida /*&& distanciaMili  > 120 -27.00 - 0.16*/)
  {
    nuevaDer = Rder.read();
    nuevaIzq = Rizq.read();
    ControlPI(nuevaIzq, nuevaDer, 0.05, 0.0, Frente);
    ProPulsos = (abs(RderAux.read()) + abs(RizqAux.read()))/2;
    Dist_rec = (2*PI*radio_rueda*ProPulsos)/2800;
    Dist_recAux = round(Dist_rec*1000);
    Colores(PWMBlancoMaxG_F, PWMBlancoMaxG_T, PWMNegroMinG_F, PWMNegroMinG_T,PWMGrisMinG_F, PWMGrisMaxG_F, PWMGrisMinG_T, PWMGrisMinG_T); 
    SENSOR_DISTANCIA();
    Datos();Datos();  // Envio de datos al robot 1
    //EnvioDatos(i);    
  }
  Dist = Dist + Dist_recAux;//Dist_rec;
  Dist_rec = 0;
  Dist_recAux = 0;
  Datos();Datos(); 
  motor1.freno();motor2.freno();
  delay(2000);
}
void Avance2giro(int DireccionGiro/*, int DistMayoroMenor*/){
  //Mayor == 1. Menor == 2
  //Dist_rec = 0;
  Datos();Datos(); 
  RizqAux.write(0);
  RderAux.write(0);
  while (Dist_recAux < datoGiro - 10) // carro 1 co -40
    {
    giro(DireccionGiro); // Salida Dist_rec
    Colores(PWMBlancoMaxG_F, PWMBlancoMaxG_T, PWMNegroMinG_F, PWMNegroMinG_T,PWMGrisMinG_F, PWMGrisMaxG_F, PWMGrisMinG_T, PWMGrisMinG_T); 
    SENSOR_DISTANCIA();         
    Datos();Datos(); 
    //EnvioDatos(i);
    } 
  motor1.freno();motor2.freno();
  delay(1000);
  SENSOR_DISTANCIA();          
  Datos();Datos(); 
  //EnvioDatos(i);
}
void Escenario(int arena){ //3x3 o 4x4
  if (arena == 3) // Un Robot
  {
    count = 9;
    for (int i = 0; i < count; i++) // para 3x3 count == 8 (9)
    {
      Datos();
      delay(500);
      if ( (Dist <= 27600*i) && (Dist >= 27000*i))
      {
        //Datos(i);
        SENSOR_DISTANCIA();
        Colores(PWMBlancoMaxG_F, PWMBlancoMaxG_T, PWMNegroMinG_F, PWMNegroMinG_T,PWMGrisMinG_F, PWMGrisMaxG_F, PWMGrisMinG_T, PWMGrisMinG_T);
        //frontalTres[i] = Color_front;
        //trasTres[i] = Color_tras;
        j2 = j2+1; // i=8, j sera 9
        Datos();
        delay(1000);
        while (i == 8 && count == 9)
        {
          Datos();
          motor1.freno(); motor2.freno();
        }       
      }
        if (i == 2 || i == 3)
        {
          //EnvioDatos3x3();
          Avance2giro(4);
          Datos();
        }
        else if(i == 5 || i == 6)
        {
          //EnvioDatos3x3();
          Avance2giro(3);
          Datos();
        }     
      // EnvioDatos3x3();//EnvioDatosPos3x3();//EnvioDatos();
      // EnvioDatos();
      // Datos();    
      Avance2(27000); // Output: Dist = Dist + Dist_recAux
      Datos(); 
      delay(500);   
    }
    //EnvioDatos3x3();
    //EnvioDatos();
  }
  else if (arena == 4) // Dos Robot's
  {
    /* El primer dato que envía después es reemplazado por el segundo valor de colores.
    Solucionar el envío de datos !!
    - [] También solucionar el desvio del carro pesado
    - [] Depurar j en el servidor web para saber si está llegando bien
    */
    count = 16;
    j2=0;
    Colores(PWMBlancoMaxG_F, PWMBlancoMaxG_T, PWMNegroMinG_F, PWMNegroMinG_T,PWMGrisMinG_F, PWMGrisMaxG_F, PWMGrisMinG_T, PWMGrisMinG_T);
    Datos();
    for (int i = 0; i < count; i++) // para 3x3 count == 8 (9)
    {
      //j = j +1;
      Datos();Datos(); 
      delay(1500);
      if ( (Dist <= 27600*i) && (Dist >= 27000*i))
      { 
        SENSOR_DISTANCIA();
        Colores(PWMBlancoMaxG_F, PWMBlancoMaxG_T, PWMNegroMinG_F, PWMNegroMinG_T,PWMGrisMinG_F, PWMGrisMaxG_F, PWMGrisMinG_T, PWMGrisMinG_T);        
        contador2 = contador2 + 1; // ULTIMOOOOO
        ContNOW2 = contador2 + incomingR2;        
        Datos();Datos();
        delay(1000);
        while (ContNOW2 == 16)
        {
          // Enviar mensaje "Mapeo completo"
          Datos();Datos(); 
          motor1.freno(); motor2.freno();
          Datos();
        }         
      }
        while (i == 15 && count == 16)
        {
          Datos(); 
          motor1.freno(); motor2.freno();
          Datos();
        }            
        if (i == 3 || i == 4 || i == 11 || i == 12)
        {
          Datos();
          Avance2giro(4); // derecha
          Datos(); 
        }
        else if(i == 7 || i == 8 /*|| i == 11 || i == 12*/)
        {
          Datos();
          Avance2giro(3); // izquierda
          Datos();
        }        
      Datos();Datos();
      delay(500); 
      //Aquí avanza y ya debe cambiar el valor de i o j o contador a uno más
      Avance2(27000); // Output: Dist = Dist + Dist_recAux
      Datos();
      delay(500);   
    }
    Datos();Datos();
  }  
}
void reset(){
  /*Rder.write(0);
  Rizq.write(0);
  distanciaMili = 0;
  vel_lineal = 0;
  PID_vel = 0;
  Volt_L = 0;
  Volt_R =0;
  dif_vel = 0;
  */
  Min_Calibracion_fron = 0; // Para poder sensar nuevamente.
  Max_Calibracion_tras = 0;
  Min_Calibracion_tras = 0; // Para poder sensar nuevamente.
  Max_Calibracion_fron = 0;
  lectura_Calibracion = 0;
  Dist_rec = 0;
  //conectar;
  Datos();
  //EnvioDatos(i);
}
//----------------------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);
  //Configuramos pines del Driver
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);

  //Límites máximos y mínimos del controlador PI
  PWMmax = 1023.0;
  PWMmin = -PWMmax;
  //Tiempo de muestreo
  InitTime = 50; // Cada 100 milisegundos realizara una muestra. PROBAR CON 50ms
  //Valores controlador velocidad lineal.
  kpVel = 4.4;//5;//3.6;//1000; //280
  kiVel = 40.3;//45.3;//65.3;//1.3; //0.2
  kdVel = 0;//540.0;//0.0;
  //Valores controlador velocidad angular.
  kpAng = 0.4;//0.8;//200;//60;
  kiAng = 35;//26.3;//1.3;//0.2;
  kdAng = 0;//100;//0.0;
  //Valores deseados de Velocidad lineal y velocidad angular.
  //vel_lineal_d = 0.0; // m/s ***La velocidad debe ser positiva**
  //vel_ang_d = 0.8;  // rad/s
  //res = 2*PI*radio_chasis;
  delay(1000); 

  randomSeed(analogRead(39)); // 32

  SETUP_SENSOR_DISTANCIA();// inicio del sensor de distancia
  
  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);
  
  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
  // Register for a callback function that will be called when data is received
  esp_now_register_recv_cb(OnDataRecv);
}

void loop() {
    Datos();
    Serial.println(i);
    Serial.println(incomingPlaca2);
    Serial.println(incomingSenDist2);
    Serial.println(incomingconect2);
    Serial.println(incomingR2);
    Serial.println(incomingColorFront2);
    Serial.println(incomingColorTras2);
    Serial.println(incomingEstado2);
    Serial.println(contador2);
    Serial.println(ContNOW2);
    delay(50);
  if(incomingconect2 == 1){
    RizqAux.write(0);
    RderAux.write(0);    
    Min_Calibracion_fron = 1000;//4095; // Para poder sensar nuevamente.
    Max_Calibracion_fron = 0;  
    Min_Calibracion_tras = 1000;//4095; // Para poder sensar nuevamente.
    Max_Calibracion_tras = 0;      
    while (Dist_recAux < datoCal) // res = 37.38495258 (-500 carro 2)
    {
      CalibracionInalambrica();
      PWMBlancoMinG_F = Min_Calibracion_fron;
      PWMBlancoMaxG_F = Max_Calibracion_fron + 10; // Necesario
      PWMBlancoMinG_T = Min_Calibracion_tras;
      PWMBlancoMaxG_T = Max_Calibracion_tras + 10; // Necesario  
      giro(3); //Out = Dist_rec*1000 --> Dist_recAux
      Datos();
      //EnvioDatos(i);
    }
    //nuevaDer = Rder.read();
    //nuevaIzq = Rizq.read();
    //ControlPI(nuevaIzq, nuevaDer, 0.0, 0.0, 5);
    motor1.freno(); motor2.freno();   
  }
  else if(incomingconect2 == 2){
    RizqAux.write(0);
    RderAux.write(0);        
    Min_Calibracion_fron = 1000;//4095; // Para poder sensar nuevamente.
    Max_Calibracion_fron = 0;  
    Min_Calibracion_tras = 1000;//4095; // Para poder sensar nuevamente.
    Max_Calibracion_tras = 0;  
    while (Dist_recAux < datoCal) //ProPulsos = 7748 -40 = 7708 pulsos
    {
      CalibracionInalambrica();    
      PWMGrisMinG_F = Min_Calibracion_fron - 15; // Necesario
      PWMGrisMaxG_F = Max_Calibracion_fron + 10; // Necesario
      PWMGrisMinG_T = Min_Calibracion_tras - 15; // Necesario
      PWMGrisMaxG_T = Max_Calibracion_tras + 10;    // Necesario
      giro(3);
      Datos();   
      //EnvioDatos(i);
    }
    //nuevaDer = Rder.read();
    //nuevaIzq = Rizq.read();
    //ControlPI(nuevaIzq, nuevaDer, 0.0, 0.0, 5);
    motor1.freno(); motor2.freno();
  }
  else if(incomingconect2 == 3){
    RizqAux.write(0);
    RderAux.write(0);    
    Min_Calibracion_fron = 1000;//4095; // Para poder sensar nuevamente.
    Max_Calibracion_fron = 0;  
    Min_Calibracion_tras = 1000;//4095; // Para poder sensar nuevamente.
    Max_Calibracion_tras = 0;         
    while (Dist_recAux < datoCal)
    {
      CalibracionInalambrica();
      PWMNegroMinG_F = Min_Calibracion_fron - 10; // - 10 Necesario
      PWMNegroMaxG_F = Max_Calibracion_fron;
      PWMNegroMinG_T = Min_Calibracion_tras - 10; // - 10 Necesario
      PWMNegroMaxG_T = Max_Calibracion_tras;
      giro(3);
      Datos();
      //EnvioDatos(i);
    }
    //nuevaDer = Rder.read();
    //nuevaIzq = Rizq.read();
    //ControlPI(nuevaIzq, nuevaDer, 0.0, 0.0, 5);
    motor1.freno(); motor2.freno();
  }
  else if(incomingconect2 == 4){ //Reset calibración
    //Solo reinicia los valores para lectura en la calibración.
    // Reset de calibración. Se utiliza para calibrar los otros colores y no borrar el valor min y max del color anterior.
    Min_Calibracion_fron = 0; // Para poder sensar nuevamente.
    Max_Calibracion_tras = 0;
    //lectura_Calibracion = 0;
    lectura_color_front_C = 0;
    lectura_color_trase_C = 0;
    Dist_rec = 0;
    Dist_recAux = 0;
    Datos();
    //EnvioDatos(i);
  }
  else if(incomingconect2 == 5){
    //Realiza un reinicio de todos los valores de la calibración.
    // Min_Calibracion_fron = 1000;//4095; // Para poder sensar nuevamente.
    // Max_Calibracion_fron = 0;  
    // Min_Calibracion_tras = 1000;//4095; // Para poder sensar nuevamente.
    // Max_Calibracion_tras = 0; 
    // lectura_Calibracion = 0;
    // Dist_rec = 0;
    // Dist_rec = 0;
    // PWMBlancoMinG = 0;
    // PWMBlancoMaxG = 0;
    // PWMGrisMinG = 0;
    // PWMGrisMaxG = 0;
    // PWMNegroMinG = 0;
    // PWMNegroMaxG = 0;
    PWMBlancoMinG_F = 0;
    PWMBlancoMaxG_F = 0; // Necesario
    PWMBlancoMinG_T = 0;
    PWMBlancoMaxG_T = 0; // Necesario
    Min_Calibracion_fron = 0; // Para poder sensar nuevamente.
    Max_Calibracion_tras = 0;
    lectura_color_front_C = 0;
    lectura_color_trase_C = 0;
    Dist_rec = 0;
    Dist_recAux = 0;
    Datos();
    //EnvioDatos(i);
  }
  else if(incomingconect2 == 6){ // Guardamos los valores y hacemos el seguimiento de caminos:
    /*PWMBlancoMin = PWMBlancoMinG;
    PWMBlancoMax = PWMBlancoMaxG;
    PWMGrisMin = PWMGrisMinG;
    PWMGrisMax = PWMGrisMaxG;
    PWMNegroMin = PWMNegroMinG;
    PWMNegroMax = PWMNegroMaxG; */
    //ControlPI(nuevaIzq, nuevaDer, 0.1, 0.0, Frente);
    //SENSOR_DISTANCIA;
    //Colores(PWMBlancoMaxG_F, PWMBlancoMaxG_T, PWMNegroMinG_F, PWMNegroMinG_T,PWMGrisMinG_F, PWMGrisMaxG_F, PWMGrisMinG_T, PWMGrisMinG_T);
    PWMGrisMinG_F = 0; // Necesario
    PWMGrisMaxG_F = 0; // Necesario
    PWMGrisMinG_T = 0; // Necesario
    PWMGrisMaxG_T = 0;    // Necesario
    Min_Calibracion_fron = 0; // Para poder sensar nuevamente.
    Max_Calibracion_tras = 0;
    lectura_color_front_C = 0;
    lectura_color_trase_C = 0;
    Dist_rec = 0;
    Dist_recAux = 0;
    Datos();
    //EnvioDatos(i); 
  }
  else if(incomingconect2 == 7){ // A prueba
    // nuevaDer = Rder.read();
    // nuevaIzq = Rizq.read();
    // ControlPI(nuevaIzq, nuevaDer, 0.06, 0.0, Frente);
    // Serial.print(uR);Serial.print(",");Serial.println(uL);
    PWMNegroMinG_F = 0; // - 10 Necesario
    PWMNegroMaxG_F = 0;
    PWMNegroMinG_T = 0; // - 10 Necesario
    PWMNegroMaxG_T = 0;
    Min_Calibracion_fron = 0; // Para poder sensar nuevamente.
    Max_Calibracion_tras = 0;
    lectura_color_front_C = 0;
    lectura_color_trase_C = 0;
    Dist_rec = 0;
    Dist_recAux = 0;
    Datos();
  }
  else if(incomingconect2 == 8)
  {
    Datos();
    motor1.freno();motor2.freno();
  }  
  /*else if(incomingconect2 == 9){  //Escenario 1: Un agente busca el blanco
    SENSOR_DISTANCIA();
    Datos();
    //EnvioDatos(i);
    Colores(PWMBlancoMaxG_F, PWMBlancoMaxG_T, PWMNegroMinG_F, PWMNegroMinG_T,PWMGrisMinG_F, PWMGrisMaxG_F, PWMGrisMinG_T, PWMGrisMinG_T);
    while(Color_front == Blanco && Color_tras == Blanco){
      //motor1.freno();motor2.freno();
      SENSOR_DISTANCIA();
      Datos();
      //EnvioDatos(i);
      delay(200);
      motor1.standby();motor2.standby();
    }
    if(sensor.readRangeContinuousMillimeters() < 120){
      RizqAux.write(0);
      RderAux.write(0);
      Dist_rec = 0;
      Dist_recAux = 0;
      randomGiro = random(3,5); // -1 a la derecha, 1 a la izquierda
      while (Dist_recAux < datoGiro)
      {
        giro(randomGiro); // Giro a la izquierda
        SENSOR_DISTANCIA();
        Datos();
        //EnvioDatos(i); 
      }
      //nuevaDer = Rder.read();
      //nuevaIzq = Rizq.read();
      //ControlPI(nuevaIzq, nuevaDer, 0.0, 0.0, 5);
      motor1.freno();motor2.freno();
      delay(500);
      Datos();
      //EnvioDatos(i);      
    }
    else if(sensor.readRangeContinuousMillimeters() > 120){
      //avance 27 centimetros
      delay(500);
      Avance1(); // Sale con Dist_rec = 0;
      Colores(PWMBlancoMaxG_F, PWMBlancoMaxG_T, PWMNegroMinG_F, PWMNegroMinG_T,PWMGrisMinG_F, PWMGrisMaxG_F, PWMGrisMinG_T, PWMGrisMinG_T); 
      SENSOR_DISTANCIA();
      delay(500);
      //motor1.freno();motor2.freno();
      Datos();
      //EnvioDatos(i);
    }
  } */
 /* else if(incomingconect2== 10){ // Escenario 2: Comienza en A1 y mapea
    Escenario(3);    
  } */
  else if (incomingconect2 == 11){ //Escenario 3: Mapeo 4x4 de dos robot's
    Escenario(4);
  }
  else if (incomingconect2 == 12){ //Escenario 4: Busqueda de un blanco con dos agentes.
    /* Cada robot debe saber la posición y el color del otro para ir mapeando el escenario.
    - Escenario 1 en este apartado. Que cada uno busque un blanco
    - Asignamos variables "Posición".
    - Variables ColorFront y ColorTras
    - Habilitar el sensor de distancia
    */
   SENSOR_DISTANCIA();
    Datos();
    Colores(PWMBlancoMaxG_F, PWMBlancoMaxG_T, PWMNegroMinG_F, PWMNegroMinG_T,PWMGrisMinG_F, PWMGrisMaxG_F, PWMGrisMinG_T, PWMGrisMinG_T);
    while(Color_front == Blanco && Color_tras == Blanco){
      // Blanco encontrado !!
      Encontrado2 = Blanco;//EncontradoBlanco;
      Encontrado3 = EncontradoB;
      SENSOR_DISTANCIA();
      Datos();
      Datos();
      delay(200);
      motor1.standby();motor2.standby();
    }
    if(sensor.readRangeContinuousMillimeters() < 120){
      RizqAux.write(0);
      RderAux.write(0);
      Dist_rec = 0;
      Dist_recAux = 0;
      randomGiro = random(3,5);  // ----> CAMBIO 1.0
      while (Dist_recAux < datoGiro)
      {
        giro(randomGiro);
        SENSOR_DISTANCIA();
        Datos();
      }
      motor1.freno();motor2.freno();
      delay(500);
      Datos();     
    }
    else if(sensor.readRangeContinuousMillimeters() > 120){
      //avance 27 centimetros
      delay(500);
      Avance1(); // Sale con Dist_rec = 0;
      Colores(PWMBlancoMaxG_F, PWMBlancoMaxG_T, PWMNegroMinG_F, PWMNegroMinG_T,PWMGrisMinG_F, PWMGrisMaxG_F, PWMGrisMinG_T, PWMGrisMinG_T); 
      SENSOR_DISTANCIA();
      delay(500);
      Datos();
    }
  }
}