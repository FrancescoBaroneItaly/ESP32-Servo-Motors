//MQTT CALLBACKS
void mqtt_callback(char* topic, byte* payload, unsigned int len) {

  /*
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("]: ");
  
  Serial.println();
  */
  
  String iot_topic = String(topic);
  String target = "";
  
  target = "app/control/servo/+";  
  if( topic_matches(target.c_str(), iot_topic.c_str()) ){

    //Serial.println("Receive Servo Motor control");

    char *token;  
            
    token = strtok(topic, "/");
    int n=0;
    
    // walk through other tokens 
    while( token != NULL ) {

      //Serial.println(token);
      if(n==3){
        
        //cmdid=1;
        servo_num = atoi(token);
        servo_angle = atoi((char *)payload);

        SERVO_TARGET[servo_num]=servo_angle;
        }

      token = strtok(NULL, "/");
      n++;
      }//while
    }//if

    //------------------

  target = "app/control/servo";  
  if( topic_matches(target.c_str(), iot_topic.c_str()) ){

    //Serial.println("Receive Servo Motor control");

    char *token;  
            
    token = strtok((char *)payload, ",");
    int n=0;
    
    // walk through other tokens 
    while( token != NULL ) {

      servo_angle = atoi(token);

      if(servo_angle>=0 && n<16)SERVO_TARGET[n]=servo_angle;
      
      token = strtok(NULL, ",");
      n++;
      }//while
    }//if
    
  }
