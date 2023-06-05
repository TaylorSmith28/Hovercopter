
/* Libraries for the Encoder, Arduino, and 6302 view */
#include "AiEsp32RotaryEncoder.h"
#include "Arduino.h"
#include <Six302.h>
#include <math.h>

/* Pins */
#define ROTARY_ENCODER_A_PIN 16
#define ROTARY_ENCODER_B_PIN 4
#define ROTARY_ENCODER_BUTTON_PIN 35
#define ROTARY_ENCODER_VCC_PIN -1 
#define ROTARY_ENCODER_STEPS 4
#define MOTOR_PIN 5

/* State Space Model */
float A[2][2] = {{0, 1}, {-33.09, -0.6}};
float B[2] = {0, 169};
float mTheta = 0.0;
float lx[2] = {0.0, 0.0};
float cx[2] = {0.0, 0.0};
float lxHat[2] = {0.0, 0.0};
float cxHat[2] = {0.0, 0.0};
float ax[2] = {0.0, 0.0};
float diff[2] = {0.0, 0.0};

/* Measurements */
float theta;
float prev_theta;
float theta_dot;
float prev_time;
float output;

/* Gain Values */
float G[2] = {1.0139, 2.9725}; // LQR Gain Values

/* FOO */
float K[2] = {20.4, 64.66}; // Manually Placed Poles

/* ROO */
float L = 9.033;
float H = 169.033;
float F = -0.41 - L;
float GG = 0;

/* 6302 View Setup */
const int AFREQUENCY = 5000;
const int RFREQUENCY = 50000;
CommManager cm(AFREQUENCY, RFREQUENCY);

/* PWM Setup */
float PWM;
float PWML = 0.0;
float PWMS = 0.0;
float PWMT = 0.0;
const int PWM_FREQUENCY = 78000;
const int PWM_CHANNEL = 0;
const int PWM_RESOLUTION = 10;
const int PWM_MAX = 1023;

/* Encoder Setup */
AiEsp32RotaryEncoder rotaryEncoder = AiEsp32RotaryEncoder(ROTARY_ENCODER_A_PIN, ROTARY_ENCODER_B_PIN, ROTARY_ENCODER_BUTTON_PIN, ROTARY_ENCODER_VCC_PIN, ROTARY_ENCODER_STEPS);
float encoder_pos = 0.0;
float encoder_theta = 0.0; // Hover arm in degrees

/* Time Setup */
float d_time = 0.005;
bool control = false;
bool foo = false;
bool roo = false;
bool set_ref = false;

void loop_6302(void* pvParameters){
  while(true){
    cm.step();
  }
}

void setup(){

  /* Initialize 6302 View */
  /* Plots */
  cm.addPlot(&mTheta, "Theta", -PI, PI);
  cm.addPlot(&diff[0], "Observer Theta", -PI, PI);
  cm.addPlot(&diff[1], "Observer Theta Dot", -PI, PI);
  cm.addPlot(&PWMS, "PWM", -3000, 3000);

  /* Set Reference Button */
  cm.addButton(&set_ref,"Zero");

  /* Slider for Gain Values */
  cm.addSlider(&G[0],"G1", 0, 5, 0.0001);
  cm.addSlider(&G[1],"G2", 0, 5, 0.0001);
  cm.addSlider(&K[0],"K1", 0, 100, 0.0001);
  cm.addSlider(&K[1],"K2", 0, 100, 0.0001);
  cm.addSlider(&L,"L", 0, 100, 0.0001);
  
  /* Toggle Options */
  cm.addToggle(&control, "Control");
  cm.addToggle(&foo, "Full Order Observer");
  cm.addToggle(&roo, "Reduced Order Observer");

  cm.connect(&Serial, 115200);

  /* Seperate Process for 6302 View */
  xTaskCreatePinnedToCore(loop_6302,"view6302_loop",150000,NULL,1,NULL,0);
  delay(500);

  /* Setup Rotary Encoder */
  rotaryEncoder.begin();
  rotaryEncoder.setup(
      [] { rotaryEncoder.readEncoder_ISR(); });

  rotaryEncoder.setBoundaries(0, 2500, true);

  // Disable Acceleration Values
  rotaryEncoder.disableAcceleration();

  /* Setup PWM */
  ledcSetup(PWM_CHANNEL, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcAttachPin(MOTOR_PIN, PWM_CHANNEL);
}

void rotary_loop()
{
  // Calculate Change in Time
	d_time = millis()-prev_time;
	
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
  
  if (d_time != 0){
    theta_dot = (theta-prev_theta)/((float(d_time)/1000.));   
  }
}

void loop(){

  /* Keep record */
  lx[0] = cx[0];
  lx[1] = cx[1];
  lxHat[0] = cxHat[0];
  lxHat[1] = cxHat[1];

  rotary_loop();

  /* Update Theta Measurements */
  mTheta = theta;
  cx[0] = theta;
  cx[1] = theta_dot;

  /* Reduced Order Observer */
  if (roo){

    cxHat[0] = mTheta;
    cxHat[1] = (d_time * F - d_time * H * G[1] + 1) * lxHat[1] - (d_time * H * G[0] - GG + L) * lxHat[0] + L * mTheta;

    ax[0] = mTheta;
    ax[1] = cxHat[1];

    diff[0] = cx[0] - cxHat[0];
    diff[1] = cx[1] - cxHat[1];

  /* Full Order Observer */
  }else if(foo){

    cxHat[0] = ((A[0][0] - K[0] - B[0] * G[0]) * d_time + 1) * lxHat[0] + ((A[0][1] - B[0] * G[1]) * d_time) * lxHat[1] + K[0] * d_time * mTheta;
    cxHat[1] = ((A[1][0] - K[1] - B[1] * G[0]) * d_time) * lxHat[0] + ((A[1][1] - B[1] * G[1]) * d_time + 1) * lxHat[1] + K[1] * d_time * mTheta;

    ax[0] = cxHat[0];
    ax[1] = cxHat[1];

    diff[0] = cx[0] - cxHat[0];
    diff[1] = cx[1] - cxHat[1];

  /* Manual Control */
  }else{

    ax[0] = cx[0];
    ax[1] = cx[1];
  }

  /* Calculate Small Signal Portion */
  if (control){
    PWMS = constrain((PWM+(-1 * G[0] * ax[0] - G[1] * ax[1]) * PWM_MAX),0,1023);
  }else{
    PWMS = 0;
  }

  ledcWrite(PWM_CHANNEL, (int)PWMS);

  if(set_ref){
    rotaryEncoder.reset();
  }

  delay(5);
}
