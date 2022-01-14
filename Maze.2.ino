int c = 10;
int lf = 5;
int lb = 9;
int rf = 11;
int rb = 10;

volatile int rl =0;
volatile int rr =0;
int distanceRight =0;
int distanceLeft =0;
int distanceCenter =0;
int lastdistanceRight =0;
int lastdistanceLeft =0;
int lastdistanceCenter =0;
volatile int x = 0;
volatile int y = 0;
volatile int xm = 0;
volatile int ym = 0;

int mode = 0, wheelSpeed=100, LwheelSpeed=90, LWS = 108, RwheelSpeed=90, correction = 0;


#include <NewPing.h>

#define SONAR_NUM 3      // Number of sensors.
#define MAX_DISTANCE 100 // Maximum distance (in cm) to ping.

NewPing sonar[SONAR_NUM] = {   // Sensor object array.
  NewPing(A4, A5, MAX_DISTANCE), // Each sensor's trigger pin, echo pin, and max distance to ping. 
  NewPing(A2, A3, MAX_DISTANCE), 
  NewPing(A0, A1, MAX_DISTANCE)
  
};

void setup() {
  
pinMode(rb, OUTPUT);
pinMode(lb, OUTPUT);
pinMode(lf, OUTPUT);
pinMode(rf, OUTPUT);


  pinMode(2,INPUT_PULLUP);

  pinMode(3,INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(3), stopLeft, CHANGE); 
  attachInterrupt(digitalPinToInterrupt(2), stopRight, CHANGE);
  
  pinMode(13,OUTPUT);
  digitalWrite(13,HIGH);
  
  Serial.begin(9600); // Open serial monitor at 115200 baud to see ping results.
  delay (1000);
}

void loop() {


  
//Aforward ();



distanceLeft = sonar[2].ping();delay (10); 
distanceRight = sonar[0].ping();delay (10);
distanceCenter = sonar[1].ping();delay (10);

Serial.println(distanceCenter);

lastdistanceLeft = distanceLeft; //запоминаются предыдущие значения дистанций
lastdistanceRight = distanceRight;
lastdistanceCenter = distanceCenter;   
distanceLeft = sonar[2].ping();delay (10); //сканируются дистанции в микросекундах по трем направлениям 
distanceRight = sonar[0].ping();delay (10);
distanceCenter = sonar[1].ping();delay (10);
  switch (mode) {
    case 0: 
  
  if ( (distanceRight > 800 || distanceRight == 0) && (lastdistanceRight > 800 || lastdistanceRight == 0)) { // если справа дыра и предыдущее сканирование тоже была справа дыра
    correction = 0;
    forward12();    // вперед
    while (rl == 0 || rr == 0){}; delay (500);
    right90();   //вправо 90 град
 
    while (rl == 0 || rr == 0); delay (500);
    forward18();  //вперед
    while (rl == 0 || rr == 0); delay (500);
    mode = 1; //change to fixed mode переход в режим фиксированной позиции
    }
    else if ( (distanceCenter > 550 || distanceCenter == 0) || (lastdistanceCenter > 550 || lastdistanceCenter == 0)){ //если впереди нет препятствия
            //Serial.println("Aforward "); 
      Aforward (); 
    //  if (!correction) {x = 0; y = 0; correction = 1;} //режим коррекции движения по прямой
     // LWS = LwheelSpeed + (y-x); // подстройка скорости левого колеса
      }
    else {correction = 0; // справа стенка и впереди стенка - переход в режим фиксированной позиции

 
    mode = 1; //change to fixed mode
    }
break;
    
    case 1: //fixed mode режим фиксированной позиции
    //Serial.println("fixed mode ");
    stoP();  //Serial.println("stoP(); ");
delay (500);
if ( (distanceRight > 800 || distanceRight == 0) ) { // справа пусто
    right90();  // поворот
    Serial.println("right90() ");
    while (rl == 0 || rr == 0); delay (500);
    forward18();  //вперед
    Serial.println("forward18() ");
    while (rl == 0 || rr == 0); delay (500);
}
else if ( (distanceCenter > 550 || distanceCenter == 0) ) // справа стенка, впереди пусто
mode = 0; //переход в режим неизвестного положения
else if ( (distanceLeft > 800 || distanceLeft == 0) ) { // справа и впереди стенка , слева пусто
   left90();  //поворот
    Serial.println("left90() ");
    while (rl == 0 || rr == 0); delay (500);
    forward18();  //вперед
    Serial.println("forward18() ");
    while (rl == 0 || rr == 0); delay (500);
}
else {
  left180(); //стенки справа, впереди, слева - разворот
Serial.println("left180() ");
    while (rl == 0 || rr == 0); delay (500);
    }
    break;
}
}
void stopLeft(void)
{
  x++;
  if (correction); 
  else if(x>xm&&rl==0){
  digitalWrite (lb,LOW);
  digitalWrite (lf,LOW);
  rl =1;
  }
}
void stopRight(void)
{
  y++;
  if (correction) ;
  else if(y>ym&&rr==0){
  digitalWrite (rf,LOW);
  digitalWrite (rb,LOW); 
  rr=1;
  }
}

  void Aback (int k)
{
  analogWrite (rf,0);
    analogWrite (rb,k);     
      analogWrite (lb,k);
        analogWrite (lf,0);
        
  }
  void stoP (void)
{
  digitalWrite (rf,LOW);
    digitalWrite (rb,LOW);     
      digitalWrite (lb,LOW);
        digitalWrite (lf,LOW);
  }
   void Aforward ()
{
  analogWrite (rf,110);
    analogWrite (rb,0);     
      analogWrite (lb,0);
        analogWrite (lf,110);
  }

void left90(void)
{  
  rl=0;
  rr=0;
   ym =13;
   xm = 14;
   x=0;
   y=0;
   analogWrite (rf,RwheelSpeed);
   analogWrite (rb,0);     
   analogWrite (lb,LwheelSpeed);
   analogWrite (lf,0);
  }
  void left180(void)
{  
  rl=0;
  rr=0;
   ym =29;
   xm = 29;
   x=0;
   y=0;
   analogWrite (rf,110);
   analogWrite (rb,0);     
   analogWrite (lb,110);
   analogWrite (lf,0);
  }
void right90(void)
{   
   rl=0;
  rr=0;  
   ym =13;
   xm = 14;
   x=0;
   y=0;
  analogWrite (rf,0);
  analogWrite (rb,RwheelSpeed);     
  analogWrite (lb,0);
  analogWrite (lf,LwheelSpeed);
  }
void forward18(void)
{   
   rl=0;
  rr=0;
  ym = 27;
   xm = 27;
   
   x=0;
   y=0;
  analogWrite (rf,RwheelSpeed);
  analogWrite (rb,0);     
  analogWrite (lb,0);
  analogWrite (lf,LwheelSpeed);
  }
 
  void forward12(void)
{     
   rl=0;
  rr=0; 
  ym =11;
   xm = 11;
   x=0;
   y=0;
  analogWrite (rf,110);
  analogWrite (rb,0);     
  analogWrite (lb,0);
  analogWrite (lf,110);
  }

  void forward3(void)
{  
   
   rl=0;
  rr=0; 
  ym =8;
   xm = 8;
   x=0;
   y=0;
  analogWrite (rf,RwheelSpeed);
  analogWrite (rb,0);     
  analogWrite (lb,0);
  analogWrite (lf,LwheelSpeed);
  }
