#include "AiEsp32RotaryEncoder.h"
#include "Arduino.h"
#include <Six302.h>
#include <math.h>

// Needed for Six302
CommManager cm(1000, 10000);
float output;

#define ROTARY_ENCODER_A_PIN 4
#define ROTARY_ENCODER_B_PIN 16
#define ROTARY_ENCODER_BUTTON_PIN 35
#define ROTARY_ENCODER_VCC_PIN -1
#define ROTARY_ENCODER_STEPS 4

AiEsp32RotaryEncoder rotaryEncoder = AiEsp32RotaryEncoder(ROTARY_ENCODER_A_PIN, ROTARY_ENCODER_B_PIN, ROTARY_ENCODER_BUTTON_PIN, ROTARY_ENCODER_VCC_PIN, ROTARY_ENCODER_STEPS);

void rotary()
{
	// Only Update If Values Change
	if (!rotaryEncoder.encoderChanged())
	{
		return;
	}
  output = rotaryEncoder.readEncoder();
  cm.step();
}

void setup()
{
  // Plotting 
  cm.addPlot(&output, "Plot", 0, 3500);

  // Connect Serial
  cm.connect(&Serial, 115200);

	// Initialize Rotary Encoder
	rotaryEncoder.begin();
	rotaryEncoder.setup(
		[] { rotaryEncoder.readEncoder_ISR(); });

	// Set Max Rotary Values as 2500
	rotaryEncoder.setBoundaries(0, 2500, false);

  // Disable Acceleration
	rotaryEncoder.disableAcceleration();
}

void loop()
{
  // Plotting
  cm.step();
  
	// Collecting Values From Rotary Encoder
	rotary();
}