// PROJETO I

// Controlo de um corpo sobre uma Plataforma movel 

//Keyboard Controls:
//
// 1 -Plataforma movel Forward
// 3 -Platforma movel  Stop
// 2 -Platforma movel Reverse


// Motor 1
int dir1PinA = 4; // IN1 L298N Driver
int dir2PinA = 5; // IN2 L298N Driver
int speedPinA = 6; //ENA L298N Driver(PWM1)

// Motor 2
int dir1PinB = 7; // IN3 L298N Driver
int dir2PinB = 8; // IN4 L298N Driver
int speedPinB = 9; //  ENA L298N Driver(PWM2)

void forward(double); // Movimento para frente
void reverse(double); // Movimento para atras


// Sensor de Rotaçao da Haste
unsigned int counter_h = 0;  //This variable will increase or decrease depending on the rotation of encoder
#define haste_high 451
#define haste_low 449


// Sensor de Proximidade

// defines pins numbers
const int trigPin_1 = 10;
const int echoPin_1 = 11;
const int trigPin_2 = 12;
const int echoPin_2 = 13;
// defines variables
long duration_1;
int distance_1;
long duration_2;
int distance_2;

//define prototypes 
int calc_distance1(); 
int calc_distance2();
void distance();

// Sensor de Rotação do Motor
volatile unsigned int counter_m = 0;  //This variable will increase on the rotation of encoder


// TIMER

unsigned long lastTime = 0;         // will store last time value was sendt
const long interval = 500;         // interval at which to send result in milli second.  1000 ms = 1 second

//PID 

void calc_pid(int); // função do cálculo do pid tem como parametro de entrada a "variável a ser controlada" e retorna o valor da "valor da variável de comando"
volatile int sum_e = 0;   // valores do somatório dos erros
volatile int e_ant = 0;  // valor do erro anterior
volatile int u_d_ant = 0; // Ganho derivativo anterior
int erro; // valor do erro
double kp = 0.5 ;// valor do ganho proporcional
double kd = 0 ;// valor do ganho derivativo
double ki = 0 ;// valor do ganho integral
#define yr 450 // variável de referência
#define U_sat_a 255 // valor de saturação superior da variável de comando(PWM máximo que o arduino fornece)
#define U_sat_b 125 // valor de saturação inferior da variável de comando(PWM mínimo para a rotação dos motores)

//SEM CONTROLO
void prob_reg(double);


void setup() {  // Setup runs once per reset
// initialize serial communication @ 9600 baud:
 Serial.begin(9600);

 
/*1- MOTORES*/ 
 
//Define L298N Dual H-Bridge Motor Controller Pins

pinMode(dir1PinA,OUTPUT); // Motor 1 output1 Pin 4
pinMode(dir2PinA,OUTPUT); // Motor 2 output2 Pin 5
pinMode(speedPinA,OUTPUT); //PWM 1 Pin 9
pinMode(dir1PinB,OUTPUT); // Motor 2 Output 1 Pin 6
pinMode(dir2PinB,OUTPUT); // Motor 2 Output 2 Pin 7
pinMode(speedPinB,OUTPUT); // PWM 2 Pin 10


/*2- SENSOR DE ROTAÇÃO DA HASTE(SR1)*/

// Encoder connect to digitalpin 2 and 3 on the Arduino.

pinMode(2, INPUT);           // set pin to input
pinMode(3, INPUT);           // set pin to input
digitalWrite(2, HIGH);       // turn on pullup resistors
digitalWrite(3, HIGH);       // turn on pullup resistors
 
//Setting up interrupt
//A rising pulse from encodenren activated ai0(). AttachInterrupt 0 is DigitalPin nr 2 on moust Arduino.
attachInterrupt(0, ai0, RISING);
  
//B rising pulse from encodenren activated ai1(). AttachInterrupt 1 is DigitalPin nr 3 on moust Arduino.
attachInterrupt(1, ai1, RISING);

/*3 - ENCODER DE ROTAÇÃO DO MOTOR(SR2M)*/

// Encoder connect to digitalpin 21 on the Arduino.

  pinMode(18, INPUT);           // set pin to input
  digitalWrite(18, HIGH);       // turn on pullup resistors

  //Setting up interrupt
  //A rising pulse from encodenren activated ai0(). AttachInterrupt 0 is DigitalPin nr 21 on moust Arduino.
  attachInterrupt(5, ai5, RISING);



/*4 - SENSOR DE PROXIMIDADE(SP1) e (SP2)*/
pinMode(trigPin_1, OUTPUT); // Sets the trigPin as an Output 1
pinMode(echoPin_1, INPUT); // Sets the echoPin as an Input 1
pinMode(trigPin_2, OUTPUT); // Sets the trigPin as an Output 2
pinMode(echoPin_2, INPUT); // Sets the echoPin as an Input 2
}


/*1- MOTORES*/ 

void forward(double dc)
{
// Motors  Forward	
analogWrite(speedPinA, dc);//Sets speed variable via PWM 
analogWrite(speedPinB, dc);
digitalWrite(dir1PinA, LOW);
digitalWrite(dir1PinB, LOW);
digitalWrite(dir2PinA, HIGH);
digitalWrite(dir2PinB, HIGH);
Serial.println("Motor Forward\n"); // Prints out “Motor 1 Forward” on the serial monitor
//Serial.println("Motor 2 Forward\n");
Serial.println("   "); // Creates a blank line printed on the serial monitor

	
}

void stop_()
{
// Motor 1 Stop (Freespin)
analogWrite(speedPinA, 0);
analogWrite(speedPinB, 0);
Serial.println("Motor Stop\n");
Serial.println("   ");
}

void reverse(double dc)
{
// Motor 1 Reverse
analogWrite(speedPinA, dc); 
analogWrite(speedPinB, dc);
digitalWrite(dir1PinA, HIGH);
digitalWrite(dir1PinB, HIGH);
digitalWrite(dir2PinA, LOW);
digitalWrite(dir2PinB, LOW);
Serial.println("Motor Reverse\n");
Serial.println("   ");

	
}

/*2- SENSOR DE ROTAÇÃO DA HASTE*/

void ai0() {
  // ai0 is activated if DigitalPin nr 2 is going from LOW to HIGH
  // Check pin 3 to determine the direction
  if(digitalRead(3)==LOW) {
    counter_h++;
  }else{
    counter_h--;
  }
}


void ai1() {
  // ai0 is activated if DigitalPin nr 3 is going from LOW to HIGH
  // Check with pin 2 to determine the direction
  if(digitalRead(2)==LOW) {
    counter_h--;
  }else{
    counter_h++;
  }
}


/*3 - SENSOR DE ROTAÇÃO DO MOTOR(SRM)*/
void ai5() {
  // ai2 is activated if DigitalPin nr 19 is going from LOW to HIGH
 // if(digitalRead(21)==LOW)
  counter_m++;
}




/*4 - SENSOR DE PROXIMIDADE(SP1) e (SP2) */

int calc_distance1()
{
	// Clears the trigPin
digitalWrite(trigPin_1, LOW);
delayMicroseconds(2);
// Sets the trigPin on HIGH state for 10 micro seconds
digitalWrite(trigPin_1, HIGH);
delayMicroseconds(10);
digitalWrite(trigPin_1, LOW);
// Reads the echoPin, returns the sound wave travel time in microseconds
duration_1 = pulseIn(echoPin_1, HIGH);
// Calculating the distance
distance_1 = duration_1*0.034/2;

return distance_1;
}

int calc_distance2()
{
// Clears the trigPin
digitalWrite(trigPin_2, LOW);
delayMicroseconds(2);
// Sets the trigPin on HIGH state for 10 micro seconds
digitalWrite(trigPin_2, HIGH);
delayMicroseconds(10);  
digitalWrite(trigPin_2, LOW);
// Reads the echoPin, returns the sound wave travel time in microseconds
duration_2 = pulseIn(echoPin_2, HIGH);
// Calculating the distance
distance_2 = duration_2*0.034/2;

return distance_2;

}

/*5 - SEM LEI DE CONTROLO*/
void prob_reg(double dc)
{

 
   if(counter_h >= haste_high)
       {
             reverse(dc);
               if(calc_distance2() <= 5)
                {
                Serial.print("\t Distance back = ");
                Serial.println (calc_distance2()); 
                stop_();
                }
       }
       
   else if(counter_h <= haste_low) 
       {
            forward(dc);
             if(calc_distance1() <= 5)
             {
                Serial.print("\t Distance front = ");
                Serial.println (calc_distance1()); 
                stop_();
             
             }
        }
       
   else
      stop_();
}


/*6 - CONTROLADOR PID */

void calc_pid(int y)
{
   double u_d; 
   double u; 
   erro = yr - y; //Valor do erro
   sum_e = sum_e + erro + e_ant; //  somatório dos erros
   u_d = kd*(erro - e_ant) + kd*u_d_ant; // Ganho derivativo
   u = kp*erro + ki*sum_e + u_d ; // Calculo do PID variável de comando
   e_ant = erro;
   u_d_ant= u_d;
   
  
   if(erro < 0) // Se erro for negativo então
     {
        u = - u;
        Serial.print("Erro: ");
        Serial.println(erro);
        Serial.print("Comando: ");
        Serial.println(u);

         if(u > U_sat_a) //Comando acima da saturação superior 
          {
            u = U_sat_a;
            sum_e = sum_e - erro - e_ant; //Somatório dos erros “congelado”, 
            //”regressa” ao valor anterior 
           }
         if(u < U_sat_b) //Comando abaixo da saturação inferior 
            {
              u = U_sat_b;
              sum_e = sum_e - erro - e_ant;  //Somatório dos erros “congelado”, 
             //”regressa” ao valor anterior 
            }
       reverse(u); // A plataforma move-se para atras
          if(calc_distance2() <= 5)   // Se a distância for menor que 5 cm 
                { 
                  Serial.print("\t Distance back = ");
                  Serial.println (calc_distance2()); 
                  forward(u); // Então a plataforma move-se para frente
                
                }
     }
     
   else if(erro > 0) // Se o erro for positivo então
    {
       Serial.print("Erro: ");
       Serial.println(erro);
       Serial.print("Comando: ");
       Serial.println(u);
      if(u > U_sat_a) //Comando acima da saturação superior 
        {
          u = U_sat_a;
          sum_e = sum_e - erro - e_ant; //Somatório dos erros “congelado”, 
           //”regressa” ao valor anterior 
         }
       if(u < U_sat_b) //Comando abaixo da saturação inferior 
         {
           u = U_sat_b;
           sum_e = sum_e - erro - e_ant;  //Somatório dos erros “congelado”, 
          //”regressa” ao valor anterior 
         }
      
   forward(u); // A plataforma move-se para frente
        if(calc_distance1() <= 5)    // Se a distância for menor que 5 cm 
           {
             Serial.print("\t Distance front = ");
             Serial.println (calc_distance1()); 
             reverse(u); // Então a plataforma move-se para atrás
             }
         }
  
   else   // Se não existir o erro então
     stop_(); // A plataforma mantem-se parada


  // return u; //Comando não saturado não é alterado 
}


// 7- TIMER 
void sendData() {
  
  //Velicadade dos Motores
  Serial.print("Pulse pr second = ");
  Serial.print(counter_m);      // Sending cout / time

  float speedOut = map(counter_m, 0, 400, 0, 100);      // change from 400 pulse pr / second to 100 m/h

  Serial.print("\t Speed = ");
  Serial.println(speedOut);
  
  /*Posição da Haste
  Serial.print("\t Posicao da haste = ");
  Serial.println (counter_h);  // Send the value of counter*/
  
  
  // Controlo PID
 calc_pid(counter_h); // A posição da haste é a nossa variável de referência
     
}



void resetSampling() {
  counter_m = 0;

}

boolean timeIntervall() {
  unsigned long currentTime = millis();

  // Chedk if it's time to make an interupt
  if (currentTime - lastTime >= interval) {
    lastTime = lastTime + interval;

    return true;
  } else if (currentTime < lastTime) {
    // After 50 day millis() will start 0 again
    lastTime = 0;

  } else {
    return false;
  }

}


// MAIN 

void loop() {
 
  
  
     Serial.print("Distancia: ");
     Serial.println(calc_distance2());
   
   /*  
     Serial.print("\t Distance back = ");
     Serial.println (calc_distance2()); 
     Serial.print("\t Distance front = ");
     Serial.println (calc_distance1()); */
     delay(1000);
  
  
  
  
  }
     
