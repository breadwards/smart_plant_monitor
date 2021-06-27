# Smart Plant Watering

## Project Description 

The firmware in this repo runs on an Arduino Leonardo with a DC pump, 4-way valve, and capacitive sensors attached.
 The sensors indirectly measure soil moisture, and the firmware uses these measurements to figure out when to water
 the plant (by turning on pump / corresponding relay). I've added some custom display elements (rolling plots of measuremnets) 
 and some safety checks (is the water source empty, sensor/hose disconnected) to avoid flooding issues.

## Feature Roadmap

I've considered a few features that might be worth adding in the future:
* Button interrupts: Right now, `loop()` just polls the state of the button, which can be flakey.
* Pump control: Might be overkill, but a simple on/off PID control loop could be added for more precise watering. An alternative approach might be to use Reinforcement Learning to come up with the optimal policy for watering.