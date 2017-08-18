//End Effector FSM
//##########CONNECTIONS DIAGRAM############################
/*
 * SONAR SENSORS:
 * --------------
 * TOP SENSOR:
 * -----------
 * RED    ---> 5V
 * YELLOW ---> digital pin 9 (PWM)
 * BLUE   ---> GND
 * -----------
 * LEFT SENSOR:
 * ------------
 * RED    ---> 5V
 * YELLOW ---> digital pin 11 (PWM)
 * BLUE   ---> GND
 * ------------
 * RIGHT SENSOR:
 * -------------
 * RED    ---> 5V
 * YELLOW ---> digital pin 6 (PWM)
 * BLUE   ---> GND
 * ------------
 * PAN AND TILT CONTROL:
 * ---------------------
 * BLUE (PAN)   ---> digital pin 5 (PWM)
 * GREEN        ---> GND 
 * RED          ---> GND
 * YELLOW (TILT)---> digital pin 3 (PWM)
 * ---------------------
 * DSFs AND PAN AND TILT RELAYS:
 * -----------------------------
 * RED          ---> 5V
 * YELLOW (P&T) ---> digital pin 13
 * GREEN (DSFs) ---> digital pin 12
 * BLUE         ---> GND
 * ---------------------
 * VOLTAGE DIVIDERS AND DSF SIGNAL:
 * --------------------------------
 * DSF1(BLUE-TOP LEFT)/DSF2(YELLOW-TOP RIGHT)/DSF3(RED- BOTTOM LEFT)/DSF4(GREEN- BOTTOM RIGHT) --^^1K^^---> A0/A1/A2/A3 respectively
 *                                                                                                      |
 *                                                                                                      >
 *                                                                                                      560
 *                                                                                                      >
 *                                                                                                      |
 *                                                                                                     GND                                                            
 */
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
const int tilt = 3, pan = 5;
float FIXED_PAN_POS = 95.0, pan_pos = FIXED_PAN_POS, FIXED_TILT_POS = 135.0, tilt_pos = FIXED_TILT_POS; //MAX_TILT = 140.0, MIN_TILT = 125.0;//130.0 //MAX_PAN = 100.0, MIN_PAN = 90.0;
float MAX_PAN = 100.0, MIN_PAN = 90.0, MAX_TILT = 140.0, MIN_TILT = 135.0; //these are lab test limits
float MIN_MAN_REOR_ANGLE = 2.5;
long abs_d_diff_PAN, abs_d_diff_TILT;
const int pwPinTOP = 9, pwPinLEFT = 11, pwPinRIGHT = 6;//pin 10 crushes ethernet communication!!!
const int arraysize = 31;//more samples increase reaction time
int newArrayFILOLEFT[arraysize], newArrayFILORIGHT[arraysize], newArrayFILOTOP[arraysize], rangevalueLEFT[arraysize], rangevalueRIGHT[arraysize], rangevalueTOP[arraysize], newArrayForSortingTOP[arraysize], newArrayForSortingLEFT[arraysize], newArrayForSortingRIGHT[arraysize];
int modETOP, modETOPprev, modELEFT, modELEFTprev, modERIGHT, modERIGHTprev;
int modETOL = 20.0, modETOL_FACTOR= 1000.0, modETOL_PAN = 20.0;//
long newSampleTOP, newSampleLEFT, newSampleRIGHT;
long pulseLEFT, pulseRIGHT, pulseTOP; 
float PT_MIN = 0.4, PT_MIN_PAN = 1.0;
// State machine variables:
char GScommand = 'a', newGScommand = 'a';//'a' is the idle state of the state machine
//Ethernet variables:
byte mac[] = {0x90, 0xA2, 0xDA, 0x10, 0xEC, 0x19};
IPAddress ip(192, 168, 137, 160);
IPAddress server(192,168, 137, 20);//IP address of the server you're connecting to.
EthernetClient client;
int port = 8888;
//Digital Flow Switch (DFS) variables:
int blackWire_DFS1 = 0, blackWire_DFS2 = 1, blackWire_DFS3 = 2;//Analogue inputs 0, 1 and 2 respectively
int blackValue_DFS1 = 0, blackValue_DFS2 = 0, blackValue_DFS3 = 0;
float DFS_on_threshold = 2.5, DFS_off_threshold = 0.5, max_blackWire_Volt = 4.0;//minimum voltage indicating activation of suction cups attached to DFS.
//Relays:
#define pin_relay_PNT 13
#define pin_relay_DFS1 12
#define pin_greyWire_DSF 2 //controls suction ON
#define pin_whiteWire_DSF 3 //controls RELEASE valve
//Micro-switches:
int uswitch1 = 4; // top: analogue input 4
int uswitch2 = 5; // bottom: analogue input 5
float uswitchValue, uswitchValue2;
//Message to Ground Station:
String leftheader = "l", rightheader = "r", topheader = "o", leftdistance = "0000", rightdistance = "0000", topdistance = "0000";//distance in mm
String panheader = "p", tiltheader = "t", Spanangle = "0000", Stiltangle = "0000";//angle in degx10
String distances_n_angles_2_GS;
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
  //Re-orientation:
  tiltservo.attach(tilt);
  panservo.attach(pan);
  panservo.write(pan_pos);
  tiltservo.write(tilt_pos);
  
  pinMode(pwPinLEFT, INPUT);
  pinMode(pwPinRIGHT, INPUT);
  pinMode(pwPinTOP, INPUT);
  delay(1000);
  for (int i = 0; i < arraysize; i++)
  {
    pulseTOP = pulseIn(pwPinTOP, HIGH);
    rangevalueTOP[i] = pulseTOP / 5.82;//mm

    pulseLEFT = pulseIn(pwPinLEFT, HIGH);
    rangevalueLEFT[i] = pulseLEFT / 5.82;//mm
    
    pulseRIGHT = pulseIn(pwPinRIGHT, HIGH);
    rangevalueRIGHT[i] = pulseRIGHT / 5.82;//mm
  }
  copy_array(rangevalueTOP, newArrayForSortingTOP, arraysize);
  isort(newArrayForSortingTOP, arraysize);
  modETOP = mode(newArrayForSortingTOP, arraysize);
  modETOPprev = modETOP;

  copy_array(rangevalueLEFT, newArrayForSortingLEFT, arraysize);
  isort(newArrayForSortingLEFT, arraysize);
  modELEFT = mode(newArrayForSortingLEFT, arraysize);
  modELEFTprev = modELEFT;
  
  copy_array(rangevalueRIGHT, newArrayForSortingRIGHT, arraysize);
  isort(newArrayForSortingRIGHT, arraysize);
  modERIGHT = mode(newArrayForSortingRIGHT, arraysize);
  modERIGHTprev = modERIGHT;
  
  //Relay:
  pinMode(pin_relay_PNT, OUTPUT);
  pinMode(pin_relay_DFS1,OUTPUT);
  pinMode(pin_greyWire_DSF, OUTPUT);
  pinMode(pin_whiteWire_DSF,OUTPUT);
  digitalWrite(pin_relay_PNT,HIGH);//on NO
  digitalWrite(pin_relay_DFS1,HIGH);
  digitalWrite(pin_greyWire_DSF,HIGH);//on NO
  digitalWrite(pin_whiteWire_DSF,HIGH);
  
  Serial.print("EE: Set-up completed.");
  delay(1000);
}

void loop()
{ 
  bool st_status = false, st_status0 = false, st_status1 = false, st_status2 = false;
  reconnect_GS();// if the server's disconnected, reconnect the client.
  delay(1500);
  switch (GScommand){
      case 's':
        digitalWrite(pin_relay_PNT, LOW);
        Serial.println("EE: Re-orienting end effector.");
        st_status = reorient();//the actual task
        newGScommand = readNprint_GSEE_mssgs();
        if ((newGScommand == 'd') || (newGScommand == 'u') || (newGScommand == 'n') || (newGScommand == 'k') ||(newGScommand == 'c') || (newGScommand == 'i') || (newGScommand == 'a')){
          GScommand = newGScommand;
        }
        if (st_status == true){
          Serial.println("EE: End effector re-oriented.");
          send_char_srv(GScommand);
        }
      break;

      case 'k':
        digitalWrite(pin_relay_PNT, LOW);
        Serial.print("EE: Manual re-orientation of end effector: PAN CLOCKWISE(");
        Serial.print(MIN_MAN_REOR_ANGLE);
        Serial.println(").");
        Serial.println("EE: Go back to Idle to reset MIN_MAN_REOR_ANGLE.");
        pan_pos = pan_pos + MIN_MAN_REOR_ANGLE;
        panservo.write(pan_pos);
        print_distances();
        newGScommand = readNprint_GSEE_mssgs();
        if ((newGScommand == 'd') || (newGScommand == 'u') || (newGScommand == 'n') || (newGScommand == 's') || (newGScommand == 'c') || (newGScommand == 'i') || (newGScommand == 'a')){
          GScommand = newGScommand;
        }
        MIN_MAN_REOR_ANGLE = 0.0;
        send_char_srv(GScommand);
      break;

      case 'n':
        digitalWrite(pin_relay_PNT, LOW);
        Serial.print("EE: Manual re-orientation of end effector: PAN ANTI-CLOCKWISE(");
        Serial.print(MIN_MAN_REOR_ANGLE);
        Serial.println(").");
        Serial.println("EE: Go back to Idle to reset MIN_MAN_REOR_ANGLE.");
        pan_pos = pan_pos - MIN_MAN_REOR_ANGLE;
        panservo.write(pan_pos);
        print_distances();
        newGScommand = readNprint_GSEE_mssgs();
        if ((newGScommand == 'u') || (newGScommand == 'k') || (newGScommand == 's') || (newGScommand == 'c') || (newGScommand == 'i') || (newGScommand == 'a')){
          GScommand = newGScommand;
        }
        MIN_MAN_REOR_ANGLE = 0.0;
        send_char_srv(GScommand);
      break;

      case 'u':
        digitalWrite(pin_relay_PNT, LOW);
        Serial.print("EE: Manual re-orientation of end effector: TILT UP(");
        Serial.print(MIN_MAN_REOR_ANGLE);
        Serial.println(").");
        Serial.println("EE: Go back to Idle to reset MIN_MAN_REOR_ANGLE.");
        tilt_pos = tilt_pos - MIN_MAN_REOR_ANGLE;
        tiltservo.write(tilt_pos);
        print_distances();
        newGScommand = readNprint_GSEE_mssgs();
        if ((newGScommand == 'd') || (newGScommand == 'k') || (newGScommand == 'n') || (newGScommand == 's') || (newGScommand == 'c') || (newGScommand == 'i') || (newGScommand == 'a')){
          GScommand = newGScommand;
        }
        MIN_MAN_REOR_ANGLE = 0.0;
        send_char_srv(GScommand);
      break;

      case 'd':
        digitalWrite(pin_relay_PNT, LOW);
        Serial.print("EE: Manual re-orientation of end effector: TILT DOWN(");
        Serial.print(MIN_MAN_REOR_ANGLE);
        Serial.println(").");
        Serial.println("EE: Go back to Idle to reset MIN_MAN_REOR_ANGLE.");
        tilt_pos = tilt_pos + MIN_MAN_REOR_ANGLE;
        tiltservo.write(tilt_pos);
        print_distances();
        newGScommand = readNprint_GSEE_mssgs();
        if ((newGScommand == 'd') || (newGScommand == 'u') || (newGScommand == 'k') || (newGScommand == 'n') || (newGScommand == 's') || (newGScommand == 'c') || (newGScommand == 'i') || (newGScommand == 'a')){
          GScommand = newGScommand;
        }
        MIN_MAN_REOR_ANGLE = 0.0;
        send_char_srv(GScommand);
      break;
      
      case 'c':
        Serial.println("EE: Activating suction cups.");
        digitalWrite(pin_relay_DFS1,LOW);
        digitalWrite(pin_greyWire_DSF,LOW);//suction on
        digitalWrite(pin_relay_PNT, HIGH);//pnt off
        st_status0 = activate(blackWire_DFS1, blackValue_DFS1);
        st_status1 = activate(blackWire_DFS2, blackValue_DFS2);
        st_status2 = activate(blackWire_DFS3, blackValue_DFS3);
        st_status = st_status0 && st_status1 && st_status2;
        newGScommand = readNprint_GSEE_mssgs();
        if ((newGScommand == 'd') || (newGScommand == 'u') || (newGScommand == 'n') || (newGScommand == 'k') ||(newGScommand == 's') || (newGScommand == 'i') || (newGScommand == 'a')){
          GScommand = newGScommand;
        }
        if (st_status == true){
          Serial.println("EE: Suction cups activated.");
          send_char_srv(GScommand);  
        }
        //uswitches to be used later://check_uswitch_contact(uswitch1);//check_uswitch_contact(uswitch2);//TODO: ANY MORE CHECKS WITH THE USWITCHES?
      break;
  
      case 'i':
        Serial.println("EE: De-activating suction cups.");
        digitalWrite(pin_greyWire_DSF,HIGH);//suction off
        digitalWrite(pin_relay_DFS1,HIGH);
        digitalWrite(pin_whiteWire_DSF,LOW);//release on
        st_status0 = deactivate(blackWire_DFS1, blackValue_DFS1);//after 20s voltage drops, pressure disappears completely after 160s
        st_status1 = deactivate(blackWire_DFS2, blackValue_DFS2);
        st_status2 = deactivate(blackWire_DFS3, blackValue_DFS3);
        st_status = st_status0 && st_status1 && st_status2;
        newGScommand = readNprint_GSEE_mssgs();
        if ((newGScommand == 'd') || (newGScommand == 'u') || (newGScommand == 'n') || (newGScommand == 'k') ||(newGScommand == 's') || (newGScommand == 'c') || (newGScommand == 'a')){
          GScommand = newGScommand;
        }
        if (st_status == true){
          Serial.println("EE: Suction cups deactivated.");
          digitalWrite(pin_relay_PNT, LOW);//pnt back on
          digitalWrite(pin_whiteWire_DSF,HIGH);//release off
          send_char_srv(GScommand);  
        }
        
      break;
  
      case 'a':
        newGScommand = readNprint_GSEE_mssgs();
        if ((newGScommand == 'd') || (newGScommand == 'u') || (newGScommand == 'n') || (newGScommand == 'k') ||(newGScommand == 's') || (newGScommand == 'c') || (newGScommand == 'i')){
            GScommand = newGScommand;
        }
        MIN_MAN_REOR_ANGLE = 2.5;
        //digitalWrite(pin_relay_PNT, HIGH);// if pnt is switched off it'll flop down in idle state
        digitalWrite(pin_relay_PNT, LOW);
        Serial.println("EE: Idle state.");
      break;
      
      default:
        newGScommand = readNprint_GSEE_mssgs();
        if ((newGScommand == 'd') || (newGScommand == 'u') || (newGScommand == 'n') || (newGScommand == 'k') || (newGScommand == 's') || (newGScommand == 'c') || (newGScommand == 'i') || (newGScommand == 'a')){
          GScommand = newGScommand;
        }
        Serial.println("EE: Waiting in state machine...");
      break;
    }
    store_distances_for_GS();
    store_pnt_angles_for_GS(pan_pos, tilt_pos);
    distances_n_angles_2_GS = leftdistance + rightheader + rightdistance + topheader + topdistance + panheader + Spanangle + tiltheader + Stiltangle;
    if(client.connected())
    {
      client.print(distances_n_angles_2_GS);
    }
    Serial.println(distances_n_angles_2_GS);
}
//Activate suction:
void activate_suction()
{
    digitalWrite(pin_greyWire_DSF,LOW);
    digitalWrite(pin_whiteWire_DSF,HIGH);
    delay(2000);
}

void deactivate_suction()
{
    digitalWrite(pin_greyWire_DSF,HIGH);
    digitalWrite(pin_whiteWire_DSF,LOW);
    delay(2000);
}

void ejector_standby()//keep ejector powered on but in idle state (no suction)
{
    digitalWrite(pin_greyWire_DSF,HIGH);
    digitalWrite(pin_whiteWire_DSF,HIGH);
    delay(2000);
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

void print_distances(){
  copy_array(rangevalueTOP, newArrayForSortingTOP, arraysize);
  copy_array(rangevalueLEFT, newArrayForSortingLEFT, arraysize);
  copy_array(rangevalueRIGHT, newArrayForSortingRIGHT, arraysize);
  
  isort(newArrayForSortingTOP, arraysize);
  isort(newArrayForSortingLEFT, arraysize);
  isort(newArrayForSortingRIGHT, arraysize);

  modETOP = mode(newArrayForSortingTOP, arraysize);
  modELEFT = mode(newArrayForSortingLEFT, arraysize);
  modERIGHT = mode(newArrayForSortingRIGHT, arraysize);

  Serial.println("EE: L,R,T distance:");
  Serial.print(modELEFT);
  Serial.print(",");
  Serial.print(modERIGHT);
  Serial.print(",");
  Serial.println(modETOP);
    
  newSampleTOP = pulseIn(pwPinTOP, HIGH) / 5.82;
  newSampleLEFT = pulseIn(pwPinLEFT, HIGH) / 5.82;
  newSampleRIGHT = pulseIn(pwPinRIGHT, HIGH) / 5.82;
  
  add_sample_filo(rangevalueTOP, newArrayFILOTOP, arraysize, newSampleTOP);
  add_sample_filo(rangevalueLEFT, newArrayFILOLEFT, arraysize, newSampleLEFT);
  add_sample_filo(rangevalueRIGHT, newArrayFILORIGHT, arraysize, newSampleRIGHT);
  
  copy_array(newArrayFILOTOP, rangevalueTOP, arraysize);
  copy_array(newArrayFILOLEFT, rangevalueLEFT, arraysize);
  copy_array(newArrayFILORIGHT, rangevalueRIGHT, arraysize);
}

void store_distances_for_GS(){
  
 leftdistance = modELEFT;
 rightdistance = modERIGHT;
 topdistance = modETOP;
 
 if(modELEFT > 10000){
  leftdistance = "10000";
 }
 if(modELEFT < 1000){
  leftdistance = "0" + leftdistance;
 }
 if(modERIGHT > 10000){
  rightdistance = "10000";
 }
 if(modERIGHT < 1000){
  rightdistance = "0" + rightdistance;
 }
 if(modETOP > 10000){
  topdistance = "10000";
 }
 if(modETOP < 1000){
  topdistance = "0" + topdistance;
 }
}

void store_pnt_angles_for_GS(float pan_pos, float pan_tilt){
 int panangle_formatted, tiltangle_formatted;
 panangle_formatted = (int)(pan_pos * 10);
 tiltangle_formatted = (int)(tilt_pos * 10);
 Spanangle = panangle_formatted;
 Stiltangle = tiltangle_formatted;
 if(panangle_formatted < 1000){
  Spanangle = "0" + Spanangle;
 }
 if(tiltangle_formatted < 1000){
  Stiltangle = "0" + Stiltangle;
 }
}

bool reorient(){
  
  bool tilt_ok = false, pan_ok = false;

  print_distances();
  
  //tilt:
  abs_d_diff_TILT = abs( modETOP - modELEFT );
  if ( ( abs_d_diff_TILT < modETOL ) || ( abs_d_diff_TILT > ( modETOL_FACTOR * modETOL ) ) )
  {
   tilt_pos = tilt_pos;
   tilt_ok = true;
  }
  else{
    if(modETOP > modELEFT)// && tilt_pos <= MAX_TILT)
    {
      tilt_pos = tilt_pos + PT_MIN;
    }
    else
    {
      tilt_pos = tilt_pos - PT_MIN;
    }
    if (tilt_pos >= MAX_TILT)
    {
      tilt_pos = MAX_TILT;
    }
    if (tilt_pos <= MIN_TILT)
    {
      tilt_pos = MIN_TILT;
    }
  }
//pan:
  abs_d_diff_PAN = abs( modELEFT - modERIGHT );
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
  
  tiltservo.write(tilt_pos);//set pan position //tiltservo.write(FIXED_TILT_POS);//WHILE TESTING PAN ONLY
  panservo.write(pan_pos);//set pan position //panservo.write(FIXED_PAN_POS);
 
  if (tilt_ok == true && pan_ok == true)
    return true;  
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
    //delay(180000);
    //delay(300000);
    delay(1000);
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

//------------------------------------------------------------------------//

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
