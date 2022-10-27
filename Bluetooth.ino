#include <SoftwareSerial.h>  //Autorise les comunication serial sur des digital pin

SoftwareSerial bluetoothSerial = SoftwareSerial(1,2);  // (rxPin, txPin)


void setup()
{
  Serial.begin(9600);
  bluetoothSerial.begin(9600);
}

void loop()
{
  String S_BInput;
  String S_BFeature;
  String S_BValue;

  sendCommand("Hello world",bluetoothSerial);
  S_BInput= bluetoothSerial.readString();
  getCommand(S_BInput,&S_BFeature,&S_BValue);

  if (S_BFeature == "stop" )
  {
    //TO-DO
  }
  if(S_BFeature == "start")
  {
    //TO-DO
  }
  delay(150);
}


/*
 * Fonction utilisée pour extraire la fonction et la valeur
 * reçut en commande lors de la communication bluetooth
 * de la forme "feature:value".
 */
void getCommand(String S_command, String* S_feature, String* S_value){
  int index;
  char* delimiterPos;
  int str_len;
  char* feature;
  char* value;
  str_len = S_command.length() + 1;
  char command[str_len];
  S_command.toCharArray(command, str_len);

  feature = (char*)calloc(str_len,sizeof(char));
  value = (char*)calloc(str_len,sizeof(char));

  delimiterPos = strchr(command,':');
  index = (int)(delimiterPos - command);
  strcat(feature,command);
  feature[index]='\0';
  value = (command+index+1);

  *S_feature = String(feature);
  *S_value = String(value);

}

boolean sendCommand(String message,SoftwareSerial serial){
  if(serial.available())
  {
    serial.println(message);
    return true;  
  }
  else{
    return false;
  }
}      






