//***************************************************************************************
//  EE40LX
//  Sketch V200
//
//  Description; robot with Ultrasonic sensor
//
//  Alejandro Lopez, Tom Zajdel
//  201508
//
//***************************************************************************************

int UStrig = P1_1;  // set UltraSonic triguer as P1.1 alias
int BUZZER = P1_3;  // set BUZZER as P1.3 alias
int LMOTOR = P2_1;  // set LMOTOR as P2.1 alias
int RMOTOR = P2_5;  // set RMOTOR as P2.5 alias
int RUSecho = P2_0; // set LeftUltraSonic as P2.0 alias
int LUSecho = P1_7; // set RightUltrasonic as P1.7 alias
int MICINP = A5;    // set MICINP as A5   alias

int MPOW50 = 128;   // set motors to use 50% PWM (possible values 0-255)
int MPOW75 = 255;   // set motors to use 75% PWM
int MPOW100 = 255;  // set motors to use 100% PWM
int MICTHRESH = 600;// set microphone trigger threshold (possible values 0-1023)

int i, val, maxval;
int Ltime, Rtime, Ldist, Rdist;

void setup()
{  
  Serial.begin(9600);
  Serial.println("Log on");
  // set outputs
  pinMode(LMOTOR, OUTPUT);
  pinMode(RMOTOR, OUTPUT);
  pinMode(BUZZER, OUTPUT);
  pinMode(UStrig, OUTPUT);
  
  // set inputs
  pinMode(LUSecho, INPUT);
  pinMode(RUSecho, INPUT);

}

void loop()
{
  
  analogWrite(LMOTOR,0);        // turn off both motors 
  analogWrite(RMOTOR,0);
  
  // Step 1: beep a bunch of times!
  //###############################
  
  for (i=1; i<5; i++)
  {
    beep(BUZZER, 1000, 100*i);
    delay(100*i);
  }

  // Step 2: listen to the microphone for ~100 ms
  //*********************************************
  
  maxval = 0;
  for (i=1; i<100; i++)
  {
    val = analogRead(MICINP);
    if (val > maxval)
      maxval = val;
    delay(1);
  }

  // If the largest voltage detected is above 1.94 V (3.3*600/1023),
  // commence the "beep dance" response
  if (maxval > MICTHRESH)
  {
    // Make the "siren" noise by alternating 1200 Hz and 800 Hz tones
    for (i=0; i<5; i++)
    {
        beep(BUZZER, 1200, 100);
        beep(BUZZER, 800,  100);
    }
    // Shake motors back and forth rapidly
    for (i=0; i<3; i++)
    {
        analogWrite(RMOTOR, MPOW50);
        delay(200);
        analogWrite(RMOTOR, 0);
        analogWrite(LMOTOR, MPOW50);
        delay(200);
        analogWrite(LMOTOR, 0);
    }
    // Make a series of tones with increasing frequency from 300-100 Hz
    // then come back down
    for (i=30; i<100; i+=1)
      beep(BUZZER, 10*i, 10);
    for (i=100; i>30; i-=1)
      beep(BUZZER, 10*i, 10);
  } 
  
  // Step 3: read the distance US sensors.
  //**************************************
  digitalWrite(UStrig, HIGH);      // supply 3.3V to the trigger
  delayMicroseconds(1000);         // delay 1ms to triguer the ultrasonic sensor
  digitalWrite(UStrig, LOW);
  Ltime=pulseIn(LUSecho, HIGH);    // time that the echo signal pulse is high
  
  sleep (60);         // new pulse
  digitalWrite(UStrig, HIGH);      // supply 3.3V to the US trigger
  delayMicroseconds(1000);         // delay 1ms to triguer the ultrasonic sensor
  digitalWrite(UStrig, LOW);
  Rtime=pulseIn(RUSecho, HIGH);
  
  Ldist= int(0.017*Ltime);         //calcular la distancia obteniendo un valor entero en cm
  Rdist= int(0.017*Rtime);         //get the Right distance in cm 
  digitalWrite(UStrig, LOW);          // turn the power-blocked rail off
 
  Serial.println("Distancia R ");
  Serial.println(Rdist);
  Serial.println(" cm");
  Serial.println("Distancia L ");
  Serial.println(Ldist);
  Serial.println(" cm");
  
  
    //motor actions according the sensors data 
  if (Ldist > 20)                     // check each photocell/circuit output and determine
    analogWrite(RMOTOR, MPOW75);     // whether to run on the left motor...
  if (Rdist > 20)                     // ...or right motor
    analogWrite(LMOTOR, MPOW75);     
  sleep(500);                      // wait 500 ms
  analogWrite(LMOTOR,0);        // turn off both motors 
  analogWrite(RMOTOR,0);
  
  sleep(500);                      // wait 500 ms
  
}
 
void beep(int pin, int freq, long ms)  	//generate a square wave at a given frequency for ms miliseconds
{
	int k;
        long semiper = (long) (1000000/(freq*2));
        long loops = (long)((ms*1000)/(semiper*2));
	for (k=0;k<loops;k++)
	{
            digitalWrite(pin, HIGH);  //set buzzer pin high
	    delayMicroseconds(semiper);  //for half of the period
	    digitalWrite(pin, LOW);   //set buzzer pin low
            delayMicroseconds(semiper);  //for the other half of the period
	}
}

