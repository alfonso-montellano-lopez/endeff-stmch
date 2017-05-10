//End Effector FSM

//Ethernet includes:
#include <SPI.h>
#include <Ethernet2.h>

// State machine variable
char GScommand = 'b';

//Ethernet:
byte mac[] = {0x90, 0xA2, 0xDA, 0x10, 0xEC, 0x19};
IPAddress ip(192, 168, 137, 160);
IPAddress server(192,168, 137, 20);//IP address of the server you're connecting to.
EthernetClient client;
int port = 8888;

// standard arduino functions
void setup()
{ 
  Serial.begin(9600);
  //Ethernet:
  Ethernet.begin(mac, ip);
  delay(1000);// give the Ethernet shield a second to initialize:
  Serial.println("EE: Ethernet connecting...");
  
  // if you get a connection, report back via serial:
  if (client.connect(server, port)) {
    Serial.println("EE: Ethernet connected.");
  }
  else {
    // if you didn't get a connection to the server:
    Serial.println("EE: Ethernet connection failed.");
  }
  
  Serial.print("EE: Set-up completed.");//client.write("EE ready.");

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
  
  bool ok = true;

  delay(2000);
 
  if (ok == true)
  {
    return true;  
  }
  else
    return false; 
}

bool activate(){
  
  bool ok = true;

  delay(2000);
 
  if (ok == true)
  {
    return true;  
  }
  else
    return false; 
}

bool deactivate(){
  
  bool ok = true;

  delay(2000);
 
  if (ok == true)
  {
    return true;  
  }
  else
    return false;
}
