


int num = 0;
int total = 0;
int counter = 0;
int numData = 0;
String inString = "";

void setup() {
  Serial.begin(9600);
}

void loop() {

while (Serial.available() > 0) {
    char inChar = Serial.read();

    if (isDigit(inChar)) {
      inString += inChar;  // Append the character to the string
    }
    
    else if (inChar == '\n') {
      counter++;
      num = inString.toInt();
      total += num;
//      Serial.print("Value:");
//      Serial.println(num);
//      Serial.print("Counter:");
//      Serial.println(counter);
      inString = "";
      
      if(counter == 10){
        numData++;
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print(numData);
        lcd.print(" Total:");
        total = total/10;
        lcd.println(total);
        Serial.println(total);
        total = 0;
        counter = 0;
      }
    }
  }
}

//  Send
//  Serial.println("Hello from Arduino");
//  delay(1000);

//Receive 

//while (Serial.available() > 0) {
//    int inChar = Serial.read();
//    if (isDigit(inChar)) {
//      // convert the incoming byte to a char and add it to the string:
//      inString += (char)inChar;
//    }
//    // if you get a newline, print the string, then the string's value:
//    if (inChar == '\n') {
//      num  = num + inString.toInt();
//      lcd.print("Value:");
//      lcd.println(num);
//      lcd.print("String: ");
//      lcd.println(inString);
//      // clear the string for new input:
//      inString = "";
//    }
//  }
//
//}