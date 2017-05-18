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
const int tilt = 6, pan = 5;
float FIXED_PAN_POS = 95.0, MAX_PAN = 100.0, MIN_PAN = 90.0;
float FIXED_TILT_POS = 135.0, MAX_TILT = 140.0, MIN_TILT = 125.0;//130.0
float theta_pan = 0, theta_tilt = 0, prev_theta_pan = 0, prev_theta_tilt = 0;
const float D_L2R = 418.0, D_T2B = 317.0;//mm//16.0;//cm //mm//9.0;//cm
long dLEFT, dRIGHT, dLEFTprev, dRIGHTprev, dTOP, dBOTTOM, dTOPprev, dBOTTOMprev, dTOL = 20.0;//dTOL 60.0mm
long abs_d_diff_PAN, abs_d_diff_TILT;
const int pwPinTOP = 4, pwPinBOTTOM = 11, pwPinLEFT = 3, pwPinRIGHT = 9;//pin 10 crushes ethernet communication!!! //Set the pin to receive the signal.
const int arraysize = 31;//15//201;//31;// more samples increase reaction time
int rangevalueLEFT[arraysize], rangevalueRIGHT[arraysize], rangevalueTOP[arraysize], rangevalueBOTTOM[arraysize];
int newArrayFILOTOP[arraysize],newArrayFILOBOTTOM[arraysize], newArrayFILOLEFT[arraysize], newArrayFILORIGHT[arraysize];
int newArrayForSortingTOP[arraysize], newArrayForSortingBOTTOM[arraysize], newArrayForSortingLEFT[arraysize], newArrayForSortingRIGHT[arraysize];
int modETOP,  modETOPprev, modEBOTTOM, modEBOTTOMprev, modELEFT,  modELEFTprev, modERIGHT, modERIGHTprev;
int modETOL = 20.0;//10.0//TILT
int modETOL_FACTOR= 1000.0;
int modETOL_PAN = 20.0;//10.0
boolean modETOP_has_changed = false, modEBOTTOM_has_changed = false;
long newSampleTOP, newSampleBOTTOM, newSampleLEFT, newSampleRIGHT;
long pulseLEFT, pulseRIGHT, pulseTOP, pulseBOTTOM;
float tilt_pos = FIXED_TILT_POS, pan_pos = FIXED_PAN_POS;
//float PT_MIN = 0.2, PT_MIN_PAN = 0.05;
float PT_MIN = 0.4, PT_MIN_PAN = 1.0;
float t0=0.0,tn=0.0;
// State machine variable:
char GScommand = 'a', GScommand_old = 'a', newGScommand = 'a';
//Ethernet variables:
byte mac[] = {0x90, 0xA2, 0xDA, 0x10, 0xEC, 0x19};
IPAddress ip(192, 168, 137, 160);
IPAddress server(192,168, 137, 20);//IP address of the server you're connecting to.
EthernetClient client;
int port = 8888;
//Digital Flow Switch (DFS) variables:
int blackWire_DFS1 = 0, blackWire_DFS2 = 0, blackWire_DFS3 = 0;//Analogue input 0
int blackValue_DFS1 = 0, blackValue_DFS2 = 0, blackValue_DFS3 = 0;
float DFS_on_threshold = 2.5, DFS_off_threshold = 0.5;//minimum voltage indicating activation of suction cups attached to DFS.
float max_blackWire_Volt = 4.0;
//int pin_relay_DFS1 = 0, pin_relay_DFS2 = 1, pin_relay_DFS3 = 2;
//Relays: // these pins don't interfere with Ethernet Shield (pin 10 does!!)
#define pin_relay_PNT 8
#define pin_relay_DFS1 2
//Micro-switches:
int uswitch1 = 1; //Analogue input 1
float uswitchValue;
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
  
//  //Re-orientation:
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
  
  //Relay:
  pinMode(pin_relay_PNT, OUTPUT); //careful when assigning these with the Ethernet shield!
  //pinMode(pin_relay_DFS2, OUTPUT); 
  //pinMode(pin_relay_DFS3, OUTPUT);
  pinMode(pin_relay_DFS1,OUTPUT);
  digitalWrite(pin_relay_PNT,HIGH);//on NO
  digitalWrite(pin_relay_DFS1,HIGH);
  
  //GScommand = 'a';
  Serial.print("EE: Set-up completed.");
  //client.write("EE ready.");
  delay(1000);
}

void loop()
{ 
  bool st_status = false;
  reconnect_GS();// if the server's disconnected, reconnect the client.
  delay(1500);//1000
  client.write('x');// tell server there is connection, otherwise it spends a long time waiting for command.
    switch (GScommand){
      case 's':
        digitalWrite(pin_relay_PNT, LOW);
        Serial.println("EE: Re-orienting end effector.");
        st_status = reorient();//the actual task
        newGScommand = readNprint_GSEE_mssgs();
        if ((newGScommand == 'c') || (newGScommand == 'i') || (newGScommand == 'a')){
          GScommand = newGScommand;
        }
        if (st_status == true){
          Serial.println("EE: End effector re-oriented.");
          send_char_srv(GScommand);  
        }
      break;
  
      case 'c':
        Serial.println("EE: Activating suction cups.");
        digitalWrite(pin_relay_DFS1,LOW);
        st_status = activate(blackWire_DFS1, blackValue_DFS1);
        newGScommand = readNprint_GSEE_mssgs();
        if ((newGScommand == 's') || (newGScommand == 'i') || (newGScommand == 'a')){
          GScommand = newGScommand;
        }
        if (st_status == true){
          Serial.println("EE: Suction cups activated.");
          send_char_srv(GScommand);  
        }
        check_uswitch_contact(uswitch1);
        
      break;
  
      case 'i':
        Serial.println("EE: De-activating suction cups.");
        digitalWrite(pin_relay_DFS1,HIGH);
        st_status = deactivate(blackWire_DFS1, blackValue_DFS1);//after 20s voltage drops, pressure disappears completely after 160s
        newGScommand = readNprint_GSEE_mssgs();
        if ((newGScommand == 's') || (newGScommand == 'c') || (newGScommand == 'a')){
          GScommand = newGScommand;
        }
        if (st_status == true){
          Serial.println("EE: Suction cups deactivated.");
          send_char_srv(GScommand);  
        }
      break;
  
      case 'a':
        newGScommand = readNprint_GSEE_mssgs();
        if ((newGScommand == 's') || (newGScommand == 'c') || (newGScommand == 'i')){
            GScommand = newGScommand;
          }
        digitalWrite(pin_relay_PNT, HIGH);
        Serial.println("EE: Idle state.");
      break;
      
      default:
        newGScommand = readNprint_GSEE_mssgs();
        if ((newGScommand == 's') || (newGScommand == 'c') || (newGScommand == 'i') || (newGScommand == 'a')){
          GScommand = newGScommand;
        }
        Serial.println("EE: Waiting in state machine...");
      break;
    }
}

//Ethernet functions:

char readNprint_GSEE_mssgs(){
  char inbyte, outbyte;
  if (client.available()) {
    inbyte = client.read();
    Serial.print("GS: ");
    Serial.println(inbyte);
  }
  return inbyte;
}

void send_char_srv(char m)
{
  client.write(m);
}


void reconnect_GS(){
  if (!client.connected()) {
    Serial.println("EE: GS disconnected....");
    client.stop();
    delay(2000);
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
  
  Serial.println("EE: L,R,T,B distance:");
  Serial.print(modELEFT);
  Serial.print(",");
  Serial.print(modERIGHT);
  Serial.print(",");
  Serial.print(modETOP);
  Serial.print(",");
  Serial.println(modEBOTTOM);
    
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
      //Serial.println("in plus");
    }
    else //(modETOP < modEBOTTOM && tilt_pos >= MIN_TILT)
    {
      tilt_pos = tilt_pos - PT_MIN;
      //Serial.println("in minus");
    }
    if (tilt_pos >= MAX_TILT)
    {
      tilt_pos = MAX_TILT;
      //Serial.println("over upper limit");
    }
    if (tilt_pos <= MIN_TILT)
    {
      tilt_pos = MIN_TILT;
      //Serial.println("over lower limit");
    }
    //Serial.println("moving");
  }
//pan:
  //if ( (-modETOL_PAN < (modELEFT - modERIGHT) ) && ( (modELEFT - modERIGHT) < modETOL_PAN)  )
  abs_d_diff_PAN = abs( modELEFT - modERIGHT );
  //Serial.println(abs_d_diff_PAN);
  if ( ( abs_d_diff_PAN < modETOL_PAN ) || ( abs_d_diff_PAN > ( modETOL_FACTOR * modETOL_PAN ) ) )
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
  panservo.write(pan_pos);//set pan position
  //panservo.write(FIXED_PAN_POS);
 
  if (tilt_ok == true && pan_ok == true)
  {
    return true;  
  }
  else
    return false; 
}

bool activate(int blackWire, float blackValue){

  //turn_DFS_on(pin_relay_DFS1);//relay not yet connnected
  //turn_DFS_on(pin_relay_DFS2);
  //turn_DFS_on(pin_relay_DFS3);

  blackValue = analogRead(blackWire);
  blackValue = map(blackValue,0,1023,0.0,max_blackWire_Volt);
//  Serial.print(DFS_on_threshold);
//  Serial.print(",");
//  Serial.println(blackValue_DFS);
  if (blackValue >= DFS_on_threshold){
    Serial.println("EE: Suction cups on DFS ON.");
    return true;
  }
  else{
    Serial.println("EE: Suction cups on DFS are not ON yet.");
    return false;
  }

//  if ((check_attachment(blackWire_DFS1, blackValue_DFS1) == true))// && (check_attachment(blackWire_DFS2, blackValue_DFS2) == true) && (check_attachment(blackWire_DFS3, blackValue_DFS3) == true))
//  {
//      
//  }
//  else
     
}

bool deactivate(int blackWire, float blackValue){
  
  //turn_DFS_off(pin_relay_DFS1);
  //turn_DFS_off(pin_relay_DFS2);
  //turn_DFS_off(pin_relay_DFS3);

  blackValue = analogRead(blackWire);
  blackValue = map(blackValue,0,1023,0.0,max_blackWire_Volt);
//  Serial.print(DFS_on_threshold);
//  Serial.print(",");
//  Serial.println(blackValue_DFS);
  if (blackValue <= DFS_off_threshold){
    Serial.println("EE: Suction cups on DFS OFF. Wait until there is no pressure in the circuit");
    Serial.println("EE: Please send 'waiting(a)' command from interface.");
    delay(180000);
    Serial.println("EE: Suction cups DETACHED.");
    return true;
  }
  else{
    Serial.println("EE: Suction cups on DFS are not OFF yet.");
    return false;
  }
 
//  if ((check_detachment(blackWire_DFS1, blackValue_DFS1) == true))// && (check_detachment(blackWire_DFS2, blackValue_DFS2) == true) && (check_detachment(blackWire_DFS3, blackValue_DFS3) == true))
//  {
//    return true;  
//  }
//  else
//    return false;
}

bool check_uswitch_contact(int uswitch)
{
  uswitchValue = analogRead(uswitch);
  uswitchValue = map(uswitchValue,0,1023,0.0,max_blackWire_Volt);//max_blackWire_Volt is 4.0, the same as for the uswitches
  if (uswitchValue >= DFS_on_threshold){//thresholds for DFS are the same for the uswitches
    Serial.println("EE: Contact double checked with microswitches.");
    return true;
  }
  else{
    Serial.println("EE: No contact detected on microswitches.");
    return false;
  }
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
  Serial.print(DFS_on_threshold);
  Serial.print(",");
  Serial.println(blackValue_DFS);
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
  Serial.print(DFS_off_threshold);
  Serial.print(",");
  Serial.println(blackValue_DFS);
  if (blackValue_DFS <= DFS_off_threshold){
    Serial.println("EE: Suction cups on DFS are OFF.");
    return true;
  }
  else{
    Serial.println("EE: Suction cups on DFS are not OFF yet. Try again.");
    return false;
  }
}

