
/*
* Getting Started example sketch for nRF24L01+ radios
* This is a very basic example of how to send data from one node to another
* Updated: Dec 2014 by TMRh20
*/

#include <SPI.h>
#include "RF24.h"

/****************** User Config ***************************/
/***      Set this radio as radio number 0 or 1         ***/
bool radioNumber = 1;

/* Hardware configuration: Set up nRF24L01 radio on SPI bus plus pins 9 & 10 */
RF24 radio(5,6);
/**********************************************************/

byte addresses[][6] = {"1NodE","2NodE"};

// Used to control whether this node is sending or receiving
bool role = 0;
int sent =9;  // default stop command

void setup() {
  Serial.begin(115200);
  Serial.println(F("RF24/examples/GettingStarted"));
  Serial.println(F("*** PRESS 'T' to begin transmitting to the other node"));
  
  radio.begin();

  // Set the PA Level low to prevent power supply related issues since this is a
 // getting_started sketch, and the likelihood of close proximity of the devices. RF24_PA_MAX is default.
  radio.setPALevel(RF24_PA_LOW);
  
  // Open a writing and reading pipe on each radio, with opposite addresses
  if(radioNumber){
    radio.openWritingPipe(addresses[1]);
    radio.openReadingPipe(1,addresses[0]);
  }else{
    radio.openWritingPipe(addresses[0]);
    radio.openReadingPipe(1,addresses[1]);
  }
  
  // Start the radio listening for data
  radio.startListening();
}

void loop() {
  
  
/****************** Ping Out Role ***************************/  
if (role == 1)  {
    
    radio.stopListening();                                    // First, stop listening so we can talk.
    
    
    Serial.println(F("Now sending"));

//    unsigned long time = micros();                             // Take the time, and send it.  This will block until complete
//     if (!radio.write( &time, sizeof(unsigned long) )){
//       Serial.println(F("failed"));
//     }

    sent = Serial.read();                            // Take the time, and send it.  This will block until complete
     if (!radio.write( &sent, sizeof(unsigned long) )){
       Serial.println(F("failed"));
     }

        
    radio.startListening();                                    // Now, continue listening
    
    unsigned long started_waiting_at = micros();               // Set up a timeout period, get the current microseconds
    boolean timeout = false;                                   // Set up a variable to indicate if a response was received or not
    
    while ( ! radio.available() ){                             // While nothing is received
      if (micros() - started_waiting_at > 200000 ){            // If waited longer than 200ms, indicate timeout and exit while loop
          timeout = true;
          break;
      }      
    }
        
    if ( timeout ){                                             // Describe the results
        Serial.println(F("Failed, response timed out."));
    }else{

        int rec;                                 // Grab the response, compare, and send to debugging spew
        radio.read( &rec, sizeof(int) );
        unsigned long time = micros();


        
        // Spew it
        Serial.print(F("Sent "));
        Serial.print(sent);
        Serial.print(F(", Got response "));
        switch(rec){
        case 0:
          // drive forward
          Serial.println("forward");
          break;
        case 1:
        // stop
          Serial.println("stop");
          break;
        case 2:
        // drive reverse
          Serial.println("reverse");
          break;
        default:
        // stop
         Serial.println("default stop");
      }
       
    }

    // Try again 1s later
    delay(1000);
  }



/****************** Pong Back Role ***************************/

  if ( role == 0 )
  {
    unsigned long got_time;
    int rec;
    
    if( radio.available()){
                                                                    // Variable for the received timestamp
      while (radio.available()) {                                   // While there is data ready
        radio.read( &rec, sizeof(unsigned long) );             // Get the payload        
      }
     
      radio.stopListening();                                        // First, stop listening so we can talk   
      radio.write( &rec, sizeof(unsigned long) );              // Send the final one back.      
      radio.startListening();                                       // Now, resume listening so we catch the next packets.     
      Serial.print(F("Sent response "));
      
        switch(rec){
        case 0:
          // drive forward
          Serial.println("forward");
          break;
        case 1:
        // stop
          Serial.println("stop");
          break;
        case 2:
        // drive reverse
          Serial.println("reverse");
          break;
        default:
        // stop
         Serial.println("default stop");
      }

       
   }
 }




/****************** Change Roles via Serial Commands ***************************/

  if ( Serial.available() )
  {
    char c = toupper(Serial.read());
    if ( c == 'T' && role == 0 ){      
      Serial.println(F("*** CHANGING TO TRANSMIT ROLE -- PRESS 'R' TO SWITCH BACK"));
      role = 1;                  // Become the primary transmitter (ping out)
    
   }else
    if ( c == 'R' && role == 1 ){
      Serial.println(F("*** CHANGING TO RECEIVE ROLE -- PRESS 'T' TO SWITCH BACK"));      
       role = 0;                // Become the primary receiver (pong back)
       radio.startListening();
       
    }
  }


} // Loop

