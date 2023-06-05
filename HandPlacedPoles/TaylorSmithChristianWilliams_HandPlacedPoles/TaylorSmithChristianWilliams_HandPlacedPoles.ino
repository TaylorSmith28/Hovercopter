#include "AiEsp32RotaryEncoder.h"
#include "Arduino.h"
#include <Six302.h>
#include <math.h>

#define ROTARY_ENCODER_A_PIN 16
#define ROTARY_ENCODER_B_PIN 4
#define ROTARY_ENCODER_BUTTON_PIN 35
#define ROTARY_ENCODER_VCC_PIN -1 
#define ROTARY_ENCODER_STEPS 4

AiEsp32RotaryEncoder rotaryEncoder = AiEsp32RotaryEncoder(ROTARY_ENCODER_A_PIN, ROTARY_ENCODER_B_PIN, ROTARY_ENCODER_BUTTON_PIN, ROTARY_ENCODER_VCC_PIN, ROTARY_ENCODER_STEPS);

CommManager cm(5000, 50000);

const int MOTOR_PIN = 5;
const int FREQUENCY = 5000;
const int MOTOR_CHANNEL = 0;
const int RESOLUTION = 10;

bool zero;
bool poles;
float output;
float PWM;
float PWM_FB;
float G1 = 1.1482;
float G2 = 1.049;
float theta = 0;
float prev_theta = 0;
float theta_dot = 0;
unsigned long prev_time = 0; 
unsigned long delta_time = 0;

void loop_6302(void* pvParameters){
  while(true){
    cm.step();
  }
}

void rotary_loop()
{
  // Calculate Change in Time
	delta_time = millis()-prev_time;
	
  // Only Update if the Encoder is Changed
	if (!rotaryEncoder.encoderChanged())
	{
    theta_dot = 0;
		return;
	}

  // Update prev_time
  prev_time = millis();
  
  // Get Output from the Encoder
  output = rotaryEncoder.readEncoder();
  prev_theta = theta;

  // Calculate the Change in Theta
  if (output < 1250){
    theta = output/2500*(PI*2);
  } else {
    theta = -(2500-output)/2500*(PI*2);
  }
  
  if (delta_time != 0){
    theta_dot = (theta-prev_theta)/((float(delta_time)/1000.));   
  }
}

void setup()
{

  // Setup 6302
    cm.addPlot(&theta, "Theta", -HALF_PI, HALF_PI);
    cm.addPlot(&theta_dot, "Theta_dot", -(PI*2), (PI*2));
    cm.addPlot(&PWM_FB, "PWM", 0, 1023);
    cm.addButton(&zero,"Zero");
    cm.addSlider(&G1,"G1",0,5,.0001);
    cm.addSlider(&G2,"G2",0,5,0.0001);
    cm.addToggle(&poles, "Use Poles");
    cm.connect(&Serial, 115200);
  
  // Create a Seperate Process for 6302 View
  xTaskCreatePinnedToCore(loop_6302,"view6302_loop",150000,NULL,1,NULL,0);
  delay(500);

  // Setup Rotary Encoder
    // Begin Communication with the Encoder
    rotaryEncoder.begin();

    // Setup Encoder
    rotaryEncoder.setup(
      [] { rotaryEncoder.readEncoder_ISR(); });

    // Set Plot Boundaries
    rotaryEncoder.setBoundaries(0, 2500, true);

    // Disable Acceleration Values
    rotaryEncoder.disableAcceleration();

  // Setup PWM
    ledcSetup(MOTOR_CHANNEL, FREQUENCY, RESOLUTION);
    ledcAttachPin(MOTOR_PIN, MOTOR_CHANNEL);
}

void loop()
{

  // If Zero Button is Pressed Reset Encoder
  if (zero == true) {
    rotaryEncoder.reset();
  }
  
  // Update Rotary Encoder Values
  rotary_loop();

  PWM_FB = constrain((PWM + (-G1*theta-G2*theta_dot)*1023),0,1023);

  // Allow Toggling Between Poles
  if (poles == true){
    ledcWrite(MOTOR_CHANNEL, (int)PWM_FB);
  } else  {
    ledcWrite(MOTOR_CHANNEL, (int)PWM);
  }

  // Delay for Changes to Take Effect
  delay(1);
}
