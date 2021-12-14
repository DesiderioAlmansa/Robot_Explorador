//creo que esta es la que mejor está

/*sensores de luz*/
const int Luz_left = A0;
const int Luz_right = A1;
int Luz_Izquierda = 0;
int Luz_Derecha = 0;

/*--------------------------------------*/

#define TRI_Pin 4 // Definimos el pin donde se conecta la señal TRI
#define ECH_Pin 5 // Definimos el pin donde se conecta la señal ECH

/*------------------------------------*/

/* servos */
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVOMIN  200 // Valor de la longitud mínima del pulso PWM para velocidad máxima en sentido antihorario (valor de 0 a 4096). antes = 200
#define SERVOSTOP 340 // Valor de la longitud del pulso para dentener el servo (valor de 0 a 4096).
#define SERVOMAX  4000 // Valor de la longitud máxima del pulso PWM para velocidad máxima en sentido horario (valor de 0 a 4096). antes  4000


int servo_izquierdo=0;
int servo_derecho=1;

/*-------------------------------*/

/*sensores inflarrojos*/
const int IR_left = 2; // Sensor IR izquierdo conectado al pin digital 2
const int IR_right = 3; // Sensor IR derecho conectado al pin digital 3
int Valor_IR_left=0; // Inicializamos a 0 la variable donde guardamos el estado del sensor IR izquierdo
int Valor_IR_right=0; // Inicializamos a 0 la variable donde guardamos el estado del sensor IR derecho
int NEGRO=0;

/*--------------------------------*/

/*buzzer*/
#define SONIDO 300 // señal acústica de 3 kHz 
#define SONIDO_2 500
int Buzzpin = 6; // Definimos el pin al que conectamos el zumbador

//MELODIA
#define Zumb 6 // definimos el pin donde se conecta la señal del zumbador
const float c = 261.63; // Do (Octava 0)
const float d = 293.66; // Re (Octava 0)
const float e = 329.63; // Mi (Octava 0)
const float f = 349.23; // Fa (Octava 0)
const float g = 392.00; // Sol (Octava 0)
const float gS = 415.30;  // Sol# (Octava 0)
const float a = 440.00; // La (Octava 0)
const float b = 493.98; // Si (Octava 0)
const float aS = 466.16; // La# (Octava 0)
const float cH = 523.25;   // Do (Octava 1)
const float cSH = 554.37;  // Do# (Octava 1)
const float dH = 587.33; // Re (Octava 1)
const float dSH = 622.25; // Re# (Octava 1)
const float eH = 659.26; // Mi (Octava 1)
const float fH = 698.46; // Fa (Octava 1)
const float fS = 369.99; // Fa# (Octava 0)
const float gH = 783.99;  // Sol (Octava 1)
const float gSH = 830.61; // Sol# (Octava 1)
const float aH = 880.00; // La (Octava 1)
const long PAU=30000; // Pausa

int d1=650, d2=500, d3=350, d4=150; // definimos los distintos tiempos para las notas
float melodia[] ={d,b,fS,d,b,fS,e,cH,g,e,cH,g,d,b,fS,d,b,fS,e,d,f,g,g,gS,a,a,aS,b,cH,PAU}; // matriz con todas las notas de la melodía 1
int duracionNota[] = {d4,d4,d4,d4,d4,d4,d4,d4,d4,d4,d4,d4,d4,d4,d4,d4,d4,d4,d4,d4,d4,d4,d4,d4,d4,d4,d4,d4,d4,d4}; // matriz con las duraciones de cada nota para la melodía 1
float melodia2[] ={c,f,f,f,e,d,e,PAU}; // matriz con todas las notas de la melodía 2
int duracionNota2[] = {d4,d4,d4,d4,d4,d4,d4,d4}; // matriz con las duraciones de cada nota para la melodía 2

void setup() { 
  /*sensores de luz*/
  pinMode(Luz_left,INPUT);
  pinMode(Luz_right,INPUT);
  Serial.begin(9600); // Inicializamos la comunicación serie a 9600 baudios
  
  /*sevos*/
  pwm.begin();  
  pwm.setPWMFreq(60); // Frecuencia PWM fijada en 60 Hz  

  /*inflarrojos*/
  pinMode(IR_left,INPUT);  // Configuramos el pin 2 donde se conectan los sensor IR izquierdo como INPUT
  pinMode(IR_right,INPUT);  // Configuramos el pin 3 donde se conectan los sensor IR izquierdo como INPUT
 
  //ultrasonidos
  pinMode(TRI_Pin, OUTPUT); // Definimos el pin 4 como salida (TRI)
  pinMode(ECH_Pin, INPUT); // Definimos el pin 5 como entrada (ECH)

   /*zumbador*/
  pinMode( Zumb, OUTPUT); // definimos el pin 6 como salida (TRI)
  reproducirMelodia(); // se reproduce la melodía 1 cuando se enciende el robo
  
  
}

void loop() {

  if(tiempoTotal>180000){
  
  parar();
  //reproducirMelodia();
  
 }else{
  Luz_Izquierda = digitalRead(Luz_left);
  Luz_Derecha = digitalRead(Luz_right);

  if(Luz_Izquierda>50 && Luz_Derecha>50){ 


 /*rota en sentido horario*/
 
 Valor_IR_left = digitalRead(IR_left);//los inflarrojos recogen la informacion
 Valor_IR_right = digitalRead(IR_right);
 
  trayectoria();
 
 if( Valor_IR_left == NEGRO || Valor_IR_right == NEGRO){//si uno de los dos inflarrojos detecta el color negro
   
    parar();// los servos se paran
   tone(Buzzpin,SONIDO);//se emite el sonido
   delay(5000);
   noTone(Buzzpin);//para de emitir el sonido
   Valor_IR_left = 1;//igualo a 1 para que no se quede en la misma mancha todo el rato y cuando salga del if y repita el loop() vuelva a leer de nuevo el dato y asi sucesivamente
   Valor_IR_right = 1;
   trayectoria();
   delay(1000);
  
 }
delay(50);

  //ULTRASONIDO
 /*distancia de 5 cm y ultrasonido*/
  long duracion, distancia;
  digitalWrite(TRI_Pin, LOW); // Mandamos un pulso positivo de 10 microsegundos
  delayMicroseconds(10);
  digitalWrite(TRI_Pin, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRI_Pin, LOW);
  duracion = pulseIn(ECH_Pin, HIGH);// Medimos la duración del pulso positivo en la senal ECH
  distancia = duracion/58; // Calculamos las distancia en cm dividiendo la duración del pulso por 58 según documentación del fabricante
 
  Serial.println("Distancia:");
  Serial.print(distancia);
  
  if (distancia <= 5){
    parar();// los servos se paran
    tone(Buzzpin,SONIDO_2);//se empieza a emitir el sonido
    delay(3000);//hace una pausa de 3 segundos
    noTone(Buzzpin);//para de emitir el sonido
    pwm.setPWM(servo_derecho, 0, SERVOMAX);//inicia la marcha de nuevo girando a la derecha para evitar el obstaculo
    pwm.setPWM(servo_izquierdo, 0, SERVOMAX);
    delay(1000);
  }
  }
  else{
    parar();
    reproducirMelodia2();
  }
}
  }
 
}
void reproducirMelodia(){ // método que reproduce la melodía 1
  for (int notaActual = 0; notaActual<30; notaActual++) { // bucle que ejecutará la melodía 1
    tone(Zumb, melodia[notaActual]); // da el tono a la frecuencia de la nota en ese momento
    delay(duracionNota[notaActual]);// se mantiene con la nota el tiempo definido para esa nota
    noTone(Zumb); // finaliza la nota
  }
}

void reproducirMelodia2(){ // método que reproduce la melodía 2
  for (int notaActual = 0; notaActual<8; notaActual++) { //bucle que ejecutará la melodía 2
    tone(Zumb, melodia2[notaActual]); // da el tono a la frecuencia de la nota en ese momento
    delay(duracionNota2[notaActual]); // se mantiene con la nota el tiempo definido para esa nota
    noTone(Zumb); // finaliza la nota
  }
}

void parar(){ //Metodo que detiene el movimiento del robot
  pwm.setPWM(servo_derecho, 0, SERVOSTOP);//los servos se paran
  pwm.setPWM(servo_izquierdo, 0, SERVOSTOP);
}

void trayectoria(){
  pwm.setPWM(servo_derecho, (0), (SERVOMAX-decremento));
  pwm.setPWM(servo_izquierdo, (0), (SERVOMIN+decremento));
  delay(500);
  if(decremento==100){
    decremento=10;
  }
}
