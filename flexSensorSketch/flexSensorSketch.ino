//Constants:
const int flexPin = A1; //pin A0 to read analog input

//Variables:
int value; //save analog value


void setup()
{
  
  Serial.begin(9600);       //Begin serial communication

}

void loop()
{
  
  value = analogRead(flexPin);         //Read and save analog value from potentiometer
  Serial.println(value);               //Print value
  delay(100);        
}