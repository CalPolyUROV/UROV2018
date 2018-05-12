/**
 * connection_test
 * This code would have been running in our UROV. It was still under developement hence why its messy.
 * Use this file as an example of how to read data from a website (in this case, a website setup by another
 *  ESP on the OBS) using AT commands.
 */

#include <Wire.h>
#include <Math.h>
#define DEBUG true
#define NUM_DATA_POINTS 16

void setup() {
  // put your setup code here, to run once:
  Wire.begin();
  Serial.begin(9600);    ///////For Serial monitor 
  String data = "";
  float angles[2];

  // we use Serial1 for ESP communication
  Serial1.begin(9600); ///////ESP Baud rate
  sendData("AT+RST\r\n",2000,DEBUG); // reset module
  sendData("AT+CWMODE=1\r\n",1000,DEBUG); // configure as access point
  sendData("AT+CWJAP_CUR=\"AI-THINKER_137050\",\"\"\r\n", 5000, DEBUG);
  sendData("AT+CIPSTART=\"TCP\",\"192.168.4.1\",80\r\n", 5000, DEBUG);
  //Serial.println(checkConnection());
  //sendData("AT+CIPSEND=3\r\n", 3000, DEBUG);
  //sendData("GET\r\n", 3000, DEBUG);
  delay(1500);
  while(1){
    // these commands are used to read data from the website
    sendData("AT+CIPSEND=3\r\n",5000, false);
    data = sendData("GET\r\n", 5000,DEBUG);
    
    parse_angles(data, angles, true); //parses angles from read data
    data = sendData("AT\r\n", 5000,DEBUG);
  
    if(data.length() > 20) {
      Serial.print("data length: ");
      Serial.println(data.length());
      break;
    }
    delay(1500);
    sendData("AT+CIPSTART=\"TCP\",\"192.168.4.1\",80\r\n",3000, false);
  }
  data = data.substring(31, data.length() - 8);
  
  
  float points[NUM_DATA_POINTS];
  parse_float_values(data, points, DEBUG);

  Serial.print("original string: ");
  Serial.println(data);
  Serial.print("[");
  for(int j = 0; j < 16; j++){
    Serial.print(points[j]);
    Serial.print(", ");
  }
  Serial.print("]");
}

void loop() {
}

bool checkConnection(){
  String response = sendData("AT+CIPSTATUS\r\n", 3000, false);
  Serial.println("c: " + response[19]);
  if(response[19] != '2'){
    return false;
  }

  return true;
}

/**
 * Given a comma delimited list of float values, will save those values into an array of floats.
 * @param value_list comma delimited list of floats
 * @param points array of floats that will contain float values after function returns
 * @param debug if true, prints out debug info
 */
void parse_float_values(String value_list, float* points, bool debug) {
  int start_idx = 0;
  int end_idx = 0;
  int nData = 0;
  String single_value;

  while (nData < NUM_DATA_POINTS) {
    end_idx = value_list.indexOf(",", start_idx); //get index bounds for next comma delimited value

    //needed to get last value
    if (nData == NUM_DATA_POINTS-1) {
      end_idx = value_list.length() + 1;
    }
    
    single_value = value_list.substring(start_idx, end_idx+1); //get next value as string
    
    // Debug messages
    if (debug) {
      Serial.print("single_value: X"); //'X' used to see any whitespace
      Serial.print(single_value);
      Serial.println("X");
    }
    
    points[nData] = single_value.toFloat();
    nData++;
    start_idx = end_idx + 1; //start at char after comma for next iteration
  }
  
  return points;
}

/**
 * parse_angles
 * @param angle_list a comma-delimited list of float values
 * @param debug set true to display debug info, false otherwise
 * @return angles an array of TWO float angles
 */
void parse_angles(String angle_list, float* angles, bool debug) {
  int start_idx = 0;
  int end_idx = 0;
  int nData = 0;
  String single_value;
  angles[0] = NULL;
  angles[1] = NULL;

  // getting second IPD line
  start_idx = angle_list.indexOf("+IPD,");
  angle_list = angle_list.substring(start_idx);
  start_idx = angle_list.indexOf("+IPD,", 5);
  angle_list = angle_list.substring(start_idx);
  start_idx = angle_list.indexOf(':');
  angle_list = angle_list.substring(start_idx + 1);

  start_idx = angle_list.indexOf('=');

  if (angle_list.substring(0, start_idx) != "Voltage") {
    if (debug) {
      Serial.print("\n====FAIL: ");
      Serial.println(angle_list);
      Serial.println("====\n");
    }

    return angles;
  }

  //get index of second and third equal signs
  start_idx++;
  end_idx = angle_list.indexOf('=', start_idx);
  start_idx = end_idx + 1;
  end_idx = angle_list.indexOf('=', start_idx); 
  single_value = angle_list.substring(start_idx, end_idx);

  angles[0] = single_value.toFloat();

  // get second to last equal sign's index
  start_idx = end_idx + 1;
  single_value = angle_list.substring(start_idx);
  angles[1] = single_value.toFloat();

  if (debug) {
    Serial.print("Xa: ");
    Serial.println(angles[0]);
  
    Serial.print("Ya: ");
    Serial.println(angles[1]);
  }
  
}

/*void espsend(String d)
         {
             String cipSend = " AT+CIPSEND=";
             cipSend += connectionId; 
             cipSend += ",";
             cipSend +=d.length();
             cipSend +="\r\n";
             sendData(cipSend,1000,DEBUG);
             sendData(d,1000,DEBUG); 
         }
*/
String sendData(String command, const int timeout, boolean debug)
{
    String response = "";
    Serial1.print(command);
    long int time = millis();
    while( (time+timeout) > millis())
    {
      while(Serial1.available())
      {
         char c = Serial1.read(); // read the next character.
         response += c;
      }  
    }
    
    if(debug)
    {
      Serial.print(response); //displays the esp response messages in arduino Serial monitor
    }
    
    return response;
}
