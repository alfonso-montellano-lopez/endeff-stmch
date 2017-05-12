//End Effector FSM
//#######################INCLUDES##########################
//Ethernet includes:
#include <SPI.h>
#include <Ethernet2.h>
//Re-orientation (pan and tilt) includes:
#include <Servo.h>
#include <Math.h>
//#######################VARIABLES#########################
//Re-orientation variables:
Servo tiltservo, panservo;
const int tilt = 3, pan = 4;
float FIXED_PAN_POS = 95.0, MAX_PAN = 100.0, MIN_PAN = 90.0;
float FIXED_TILT_POS = 125.0, MAX_TILT = 140.0, MIN_TILT = 125.0;//130.0
float theta_pan = 0, theta_tilt = 0, prev_theta_pan = 0, prev_theta_tilt = 0;
const float D_L2R = 418.0, D_T2B = 317.0;//mm//16.0;//cm //mm//9.0;//cm
long dLEFT, dRIGHT, dLEFTprev, dRIGHTprev, dTOP, dBOTTOM, dTOPprev, dBOTTOMprev, dTOL = 20.0;//dTOL 60.0mm
long abs_d_diff_PAN, abs_d_diff_TILT;
const int pwPinTOP = 11, pwPinBOTTOM = 8, pwPinLEFT = 9, pwPinRIGHT = 10; //Set the pin to receive the signal.
const int arraysize = 15;//15//201;//31;// more samples increase reaction time
int rangevalueLEFT[arraysize], rangevalueRIGHT[arraysize], rangevalueTOP[arraysize], rangevalueBOTTOM[arraysize];
int newArrayFILOTOP[arraysize],newArrayFILOBOTTOM[arraysize], newArrayFILOLEFT[arraysize], newArrayFILORIGHT[arraysize];
int newArrayForSortingTOP[arraysize], newArrayForSortingBOTTOM[arraysize], newArrayForSortingLEFT[arraysize], newArrayForSortingRIGHT[arraysize];
int modETOP,  modETOPprev, modEBOTTOM, modEBOTTOMprev, modELEFT,  modELEFTprev, modERIGHT, modERIGHTprev;
int modETOL = 10.0;//10.0//TILT
int modETOL_FACTOR= 1000.0;
int modETOL_PAN = 25.0;//10.0
boolean modETOP_has_changed = false, modEBOTTOM_has_changed = false;
long newSampleTOP, newSampleBOTTOM, newSampleLEFT, newSampleRIGHT;
long pulseLEFT, pulseRIGHT, pulseTOP, pulseBOTTOM;
float tilt_pos = FIXED_TILT_POS, pan_pos = FIXED_PAN_POS;
float PT_MIN = 0.5, PT_MIN_PAN = 0.15;
float t0=0.0,tn=0.0;
// State machine variable:
char GScommand = 'b';
//Ethernet variables:
byte mac[] = {0x90, 0xA2, 0xDA, 0x10, 0xEC, 0x19};
IPAddress ip(192, 168, 137, 160);
IPAddress server(192,168, 137, 20);//IP address of the server you're connecting to.
EthernetClient client;
int port = 8888;
//Digital Flow Switch (DFS) variables:
int blackWire_DFS1 = 0, blackWire_DFS2 = 0, blackWire_DFS3 = 0;//Analogue input 0
float blackValue_DFS1 = 0.0, blackValue_DFS2 = 0.0, blackValue_DFS3 = 0.0;
float DFS_on_threshold = 3.5, DFS_off_threshold = 0.5;//minimum voltage indicating activation of suction cups attached to DFS.
float max_blackWire_Volt = 4.0;
int pin_relay_DFS1, pin_relay_DFS2, pin_relay_DFS3;
//#############################FUNCTIONS#############################
void setup()
{ 
  //Serial:
  Serial.begin(9600);
  //Ethernet:
  Ethernet.begin(mac, ip);
  delay(1000);// give the Ethernet shield a second to initialize:
  Serial.println("EE: Ethernet connecting...");
  if (client.connect(server, port)) { // if you get a connection, report back via serial:
    Serial.println("EE: Ethernet connected.");
  }
  else {
    // if you didn't get a connection to the server:
    Serial.println("EE: Ethernet connection failed.");
  }
  Serial.print("EE: Set-up completed.");//client.write("EE ready.");
  //Re-orientation:
  tiltservo.attach(tilt);
  panservo.attach(pan);
  panservo.write(pan_pos);
  tiltservo.write(tilt_pos);
  pinMode(pwPinLEFT, INPUT);
  pinMode(pwPinRIGHT, INPUT);
  pinMode(pwPinTOP, INPUT);
  pinMode(pwPinBOTTOM, INPUT);//the sensor at the bottom is sticking out 4mm with respect to the one on the top
  delay(1000);
  for (int i = 0; i < arraysize; i++)
  {
    pulseTOP = pulseIn(pwPinTOP, HIGH);
    rangevalueTOP[i] = pulseTOP / 5.82;//mm
    
    pulseBOTTOM = pulseIn(pwPinBOTTOM, HIGH);
    rangevalueBOTTOM[i] = pulseBOTTOM / 5.82;//mm

    pulseLEFT = pulseIn(pwPinLEFT, HIGH);
    rangevalueLEFT[i] = pulseLEFT / 5.82;//mm
    
    pulseRIGHT = pulseIn(pwPinRIGHT, HIGH);
    rangevalueRIGHT[i] = pulseRIGHT / 5.82;//mm
  }
  copy_array(rangevalueTOP, newArrayForSortingTOP, arraysize);
  isort(newArrayForSortingTOP, arraysize);
  modETOP = mode(newArrayForSortingTOP, arraysize);
  modETOPprev = modETOP;
  copy_array(rangevalueBOTTOM, newArrayForSortingBOTTOM, arraysize);
  isort(newArrayForSortingBOTTOM, arraysize);
  modEBOTTOM = mode(newArrayForSortingBOTTOM, arraysize);
  modEBOTTOMprev = modEBOTTOM;
  copy_array(rangevalueLEFT, newArrayForSortingLEFT, arraysize);
  isort(newArrayForSortingLEFT, arraysize);
  modELEFT = mode(newArrayForSortingLEFT, arraysize);
  modELEFTprev = modELEFT;
  copy_array(rangevalueRIGHT, newArrayForSortingRIGHT, arraysize);
  isort(newArrayForSortingRIGHT, arraysize);
  modERIGHT = mode(newArrayForSortingRIGHT, arraysize);
  modERIGHTprev = modERIGHT;

  pinMode(pin_relay_DFS1, OUTPUT); 
  pinMode(pin_relay_DFS2, OUTPUT); 
  pinMode(pin_relay_DFS3, OUTPUT);
  
  delay(1000);
}

void loop()
{
  bool st_status = false;
  delay(2000);
  reconnect_GS();// if the server's disconnected, reconnect the client.
  GScommand = readNprint_GSEE_mssgs();//if there are incoming bytes available from the server, read them and print them.
  
  switch (GScommand){
    case 's':
      Serial.println("EE: Re-orienting end effector.");
      st_status = reorient();//the actual task
      if (st_status == true){
        Serial.println("EE: End effector re-oriented.");
        send_char_srv(GScommand);  
      }
      else{
        GScommand = 'e';
        send_char_srv(GScommand);
      }
    break;

    case 'c':
      Serial.println("EE: Activating suction cups.");
      st_status = activate();
      if (st_status == true){
        Serial.println("EE: Suction cups activated.");
        send_char_srv(GScommand);  
      }
      else{
        GScommand = 'e';
        send_char_srv(GScommand);    
      }
    break;

    case 'i':
      Serial.println("EE: De-activating suction cups.");
      st_status = deactivate();
      if (st_status == true){
        Serial.println("EE: Suction cups deactivated.");
        send_char_srv(GScommand);  
      }
      else{
        GScommand = 'e';
        send_char_srv(GScommand);
      }
    break;

    case 'e':
      Serial.println("EE: Something went wrong, task couldn't be completed.");
    break;
    
    default:
      Serial.println("EE: Waiting...");
    break;
  }

}

//Ethernet functions:

char readNprint_GSEE_mssgs(){
  char inbyte, outbyte;
  if (client.available()) {
    inbyte = client.read();
    //Serial.print("GS: ");
    //Serial.println(inbyte);
  }
  return inbyte;
}

void send_char_srv(char m)
{
  client.write(m);
}


void reconnect_GS(){
  if (!client.connected()) {
    Serial.println();
    Serial.println("EE: GS disconnected....");
    client.stop();
    delay(1000);
    if (client.connect(server, port)) {
      Serial.println("EE: GS connected.");
    }
  }
  return;
}

bool reorient(){
  
  bool tilt_ok = false, pan_ok = false;

  copy_array(rangevalueTOP, newArrayForSortingTOP, arraysize);
  copy_array(rangevalueBOTTOM, newArrayForSortingBOTTOM, arraysize);
  copy_array(rangevalueLEFT, newArrayForSortingLEFT, arraysize);
  copy_array(rangevalueRIGHT, newArrayForSortingRIGHT, arraysize);
  isort(newArrayForSortingTOP, arraysize);
  isort(newArrayForSortingBOTTOM, arraysize);
  isort(newArrayForSortingLEFT, arraysize);
  isort(newArrayForSortingRIGHT, arraysize);
  modEBOTTOM = mode(newArrayForSortingBOTTOM, arraysize);
  modETOP = mode(newArrayForSortingTOP, arraysize);
  modELEFT = mode(newArrayForSortingLEFT, arraysize);
  modERIGHT = mode(newArrayForSortingRIGHT, arraysize);
  
  Serial.print(",");
  Serial.print(modELEFT);
  Serial.print(",");
  Serial.println(modERIGHT);
    
  newSampleTOP = pulseIn(pwPinTOP, HIGH) / 5.82;
  newSampleBOTTOM = pulseIn(pwPinBOTTOM, HIGH) / 5.82;
  newSampleLEFT = pulseIn(pwPinLEFT, HIGH) / 5.82;
  newSampleRIGHT = pulseIn(pwPinRIGHT, HIGH) / 5.82;
  
  add_sample_filo(rangevalueTOP, newArrayFILOTOP, arraysize, newSampleTOP);
  add_sample_filo(rangevalueBOTTOM, newArrayFILOBOTTOM, arraysize, newSampleBOTTOM);
  add_sample_filo(rangevalueLEFT, newArrayFILOLEFT, arraysize, newSampleLEFT);
  add_sample_filo(rangevalueRIGHT, newArrayFILORIGHT, arraysize, newSampleRIGHT);
  
  copy_array(newArrayFILOTOP, rangevalueTOP, arraysize);
  copy_array(newArrayFILOBOTTOM, rangevalueBOTTOM, arraysize);
  copy_array(newArrayFILOLEFT, rangevalueLEFT, arraysize);
  copy_array(newArrayFILORIGHT, rangevalueRIGHT, arraysize);
  
  //tilt:
  //if ( (-modETOL < (modETOP - modEBOTTOM) ) && ( (modETOP - modEBOTTOM) < modETOL)  )
  abs_d_diff_TILT = abs( modETOP - modEBOTTOM );
  //Serial.println(abs_d_diff_TILT);
  if ( ( abs_d_diff_TILT < modETOL ) || ( abs_d_diff_TILT > ( modETOL_FACTOR * modETOL ) ) )
  {
   tilt_pos = tilt_pos;
   tilt_ok = true;
  }
  else{
    if(modETOP > modEBOTTOM)// && tilt_pos <= MAX_TILT)
    {
      tilt_pos = tilt_pos + PT_MIN;
      Serial.println("in plus");
    }
    else //(modETOP < modEBOTTOM && tilt_pos >= MIN_TILT)
    {
      tilt_pos = tilt_pos - PT_MIN;
      Serial.println("in minus");
    }
    if (tilt_pos >= MAX_TILT)
    {
      tilt_pos = MAX_TILT;
      Serial.println("over upper limit");
    }
    if (tilt_pos <= MIN_TILT)
    {
      tilt_pos = MIN_TILT;
      Serial.println("over lower limit");
    }
    Serial.println("moving");
  }
//pan:
  //if ( (-modETOL_PAN < (modELEFT - modERIGHT) ) && ( (modELEFT - modERIGHT) < modETOL_PAN)  )
  abs_d_diff_PAN = abs( modELEFT - modERIGHT );
  //Serial.println(abs_d_diff_PAN);
  if ( ( abs_d_diff_PAN < modETOL_PAN ) || ( abs_d_diff_PAN > ( 2 * modETOL_PAN ) ) )
  {
   pan_pos = pan_pos;
   pan_ok = true;
  }
  else{ 
    if(modELEFT > modERIGHT && pan_pos < MAX_PAN)
    {
      pan_pos = pan_pos + PT_MIN_PAN;
    }
    if (modELEFT < modERIGHT && pan_pos > MIN_PAN)
    {
      pan_pos = pan_pos - PT_MIN_PAN;
    }
  }
  
  tiltservo.write(tilt_pos);//set pan position
  //tiltservo.write(FIXED_TILT_POS);//WHILE TESTING PAN ONLY
  //panservo.write(pan_pos);//set pan position
  panservo.write(FIXED_PAN_POS);
 
  if (tilt_ok == true && pan_ok == true)
  {
    return true;  
  }
  else
    return false; 
}

bool activate(){

  turn_DFS_on(pin_relay_DFS1);
  turn_DFS_on(pin_relay_DFS2);
  turn_DFS_on(pin_relay_DFS3);

  delay(2000);

  if ((check_attachment(blackWire_DFS1, blackValue_DFS1) == true) && (check_attachment(blackWire_DFS2, blackValue_DFS2) == true) && (check_attachment(blackWire_DFS3, blackValue_DFS3) == true))
  {
    return true;  
  }
  else
    return false; 
}

bool deactivate(){
  
  turn_DFS_off(pin_relay_DFS1);
  turn_DFS_off(pin_relay_DFS2);
  turn_DFS_off(pin_relay_DFS3);

  delay(2000);
 
  if ((check_detachment(blackWire_DFS1, blackValue_DFS1) == true) && (check_detachment(blackWire_DFS2, blackValue_DFS2) == true) && (check_detachment(blackWire_DFS3, blackValue_DFS3) == true))
  {
    return true;  
  }
  else
    return false;
}

//Additional functions:

void add_sample_filo(int *a, int *b, int n, long nS) {
  for (int i = 0; i < n-1; ++i){
    b[i]=a[i+1];
  }
  b[n-1]=nS;
}

void copy_array(int *from, int *into, int n)
{
  for (int i = 0; i < n; ++i)
  {
    into[i] = from[i];
    }
}

//Sorting function
// sort function (Author: Bill Gentles, Nov. 12, 2010)
void isort(int *a, int n) {
  //  *a is an array pointer function

  for (int i = 1; i < n; ++i)
  {
    int j = a[i];
    int k;
    for (k = i - 1; (k >= 0) && (j < a[k]); k--)
    {
      a[k + 1] = a[k];
    }
    a[k + 1] = j;
  }
}

//Mode function, returning the mode or median.
//Author: Jason Lessels
//This work has been compiled using many sources mainly posts/wiki posts from Allen, Bruce (2009/July/23) and Gentles, Bill (2010/Nov/12)
int mode(int *x, int n) {
  int i = 0;
  int count = 0;
  int maxCount = 0;
  int mode = 0;
  int bimodal;
  int prevCount = 0;

  while (i < (n - 1)) {
    prevCount = count;
    count = 0;

    while (x[i] == x[i + 1]) {
      count++;
      i++;
    }

    if (count > prevCount & count > maxCount) {
      mode = x[i];
      maxCount = count;
      bimodal = 0;
    }
    if (count == 0) {
      i++;
    }
    if (count == maxCount) { //If the dataset has 2 or more modes.
      bimodal = 1;
    }
    if (mode == 0 || bimodal == 1) { //Return the median if there is no mode.
      mode = x[(n / 2)];
    }
    return mode;
  }
}

//Function to print the arrays.
void printArray(int *a, int n)
{
  for (int i = 0; i < n; i++)
  {
    Serial.print(a[i], DEC);
    Serial.print(' ');
  }

  Serial.println();
}

//DFS:

void turn_DFS_on(int relay_DFS){
    digitalWrite(relay_DFS,HIGH);
    Serial.println("EE: DFS on.");
}
void turn_DFS_off(int relay_DFS){
    digitalWrite(relay_DFS,LOW);
    Serial.println("EE: DFS off.");
}

bool check_attachment(int blackWire_DFS, float blackValue_DFS){
  blackValue_DFS = analogRead(blackWire_DFS);
  blackValue_DFS = map(blackValue_DFS,0,1023,0.0,max_blackWire_Volt);
  if (blackValue_DFS >= DFS_on_threshold){
    Serial.println("EE: Suction cups on DFS ON.");
    return true;
  }
  else{
    Serial.println("EE: Suction cups on DFS are not ON yet. Try again.");
    return false;
  }
}

bool check_detachment(int blackWire_DFS, float blackValue_DFS){
  blackValue_DFS = analogRead(blackWire_DFS);
  blackValue_DFS = map(blackValue_DFS,0,1023,0.0,max_blackWire_Volt);
  if (blackValue_DFS <= DFS_off_threshold){
    Serial.println("EE: Suction cups on DFS are OFF.");
    return true;
  }
  else{
    Serial.println("EE: Suction cups on DFS are not OFF yet. Try again.");
    return false;
  }
}

