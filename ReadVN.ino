const int bufferSize = 256; // Define the maximum length of the input string

#define SIZEINPUT 1
String typeInput[] = {"Yaw","Pitch","Roll","PX","PY","PZ","VX","VY","VZ","AX","AY","AZ","GX","GY","GZ"};
float val1[] ={0};
int valtest = 0;
void setup() {
  Serial1.begin(115200); // Specify the port used by Arduino B
  Serial.begin(115200); // Initialize the serial communication
  Serial.setTimeout(1); // Set the timeout in milliseconds for reading
}

void loop() {

  if (Serial.available()) { // Check if data is available to read
    char buffer[bufferSize]; // Create a character array to store the input string
    memset(buffer, 0, sizeof(buffer)); // Clear the buffer

    int bytesRead = Serial.readBytesUntil('\n', buffer, bufferSize - 1); // Read the incoming data

    // Process the input string
    if (bytesRead > 0) {
      buffer[bytesRead] = '\0'; // Null-terminate the string
      int i = 0;
      // Remove portion after the asterisk
      char* asteriskPos = strchr(buffer, '*');

      if (asteriskPos != NULL) {
        *asteriskPos = '\0'; // Null-terminate the string at the asterisk position
      }

      // Tokenize the input string based on the comma delimiter
      char* token = strtok(buffer, ",");
      token = strtok(NULL, ","); // Get the next token
      while (token != NULL) {
        if(valtest == 0){
          valtest =1;
                  float value = atof(token);
      
  val1[i] = value;
    
          
        }
        else{
        float value = atof(token);
       
  Serial.print(typeInput[i]);
          Serial.print(" :");
          Serial.println(float(value-val1[i]),5);
i++;

        token = strtok(NULL, ","); // Get the next token
        }
      }
    }
  }
}