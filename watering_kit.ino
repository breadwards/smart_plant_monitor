#include <U8glib.h>

#include <Wire.h>
#include "U8glib.h"
U8GLIB_SSD1306_128X64 u8g(U8G_I2C_OPT_NONE);    // I2C
#include "Wire.h"
#include "RTClib.h"
RTC_DS1307 RTC;

#define NUM_SENSORS 4
#define MAX_RAW_MEASUREMENT 600
#define MIN_RAW_MEASUREMENT 360
#define TOO_LOW 25
#define TOO_HIGH 40
#define TIME_HISTORY 100
#define NUM_DISPLAY_STATES 4

int focus_sensor = 0;

// division size (pixels) for x and y in rolling plot
const unsigned int grid_interval_x = 27;
const unsigned int grid_interval_y = 31;

// range of pixels for each dimension
const unsigned int origin_x = 10;
const unsigned int origin_y = 0;
const unsigned int dim_x = 120;
const unsigned int dim_y = 65;

// bump the y axis trace such that the 
// lowest possible value is not drawn on top 
// a grid line
const int display_y_offset = -4;

struct measurement_trace_t {
  unsigned int history[TIME_HISTORY];
  unsigned int time_index = 0;
};
measurement_trace_t measurement_trace;

struct system_state_t {
  // Use this to configure which relays have plants connected
  unsigned char active_relay_mask;
  // button input
  unsigned int button_pin;
  // Toggle which trace is active on the LED display
  unsigned int display_state;
  // When we engage the pump, record what the initial error was
  float error_on_egage[NUM_SENSORS];
  // Sensor to display live readout from
  unsigned int focus_sensor;
  // Read out raw measurements and normalize to %
  float normalized_mv[NUM_SENSORS];
  // In cases where the water runs out, force the pump off
  unsigned short pump_override;
  // Pin to control pump
  unsigned int pump_pin;
  // Pump state (0: off, 1: on)
  unsigned int pump_state;
  // Map pins to each relay
  int relay_pinouts[NUM_SENSORS];
  // Keep track of each relay in a bitfield (0: closed, 1: open)
  unsigned char relay_state_mask;
  // Map array of sensors
  int sensor_pinouts[NUM_SENSORS];
  // Keep track of how long the pump has been on
  int ticks_since_pump_engagement;
};
system_state_t system_state = { 
  0x1,               // active_relay_mask
  12,                // button_pin 
  0,                 // display_state
  {0, 0, 0, 0},      // error_on_engage
  0,                 // focus_sensor
  {0, 0, 0, 0},      // normalized_mv
  0,                 // pump_override
  4,                 // pump_pin
  0,                 // pump_state
  {6, 8, 9, 10},     // relay_pinouts
  0,                 // relay_state_mask
  {A0, A1, A2, A3}, // sensor_pinouts
  0                  // ticks_since_pump_engagemnet
};

void setup() {
  delay(2000);
  Wire.begin();
  RTC.begin();
  Serial.begin(9600);

  for (unsigned int relay_index = 0; relay_index < NUM_SENSORS; ++relay_index) {
    pinMode(system_state.relay_pinouts[relay_index], OUTPUT);
  }
  // declare pump as output
  pinMode(system_state.pump_pin, OUTPUT);
  // declare switch as input
  pinMode(system_state.button_pin, INPUT);
  //pinMode(ROTARY_ANGLE_SENSOR, INPUT);

  // on boot, always start with all relays off / pump off
  for (unsigned int relay_index = 0; relay_index < NUM_SENSORS; ++relay_index) {
    digitalWrite(system_state.relay_pinouts[relay_index], LOW);
    system_state.relay_state_mask &= ~(1 << relay_index);
    delay(50);
  }
  digitalWrite(system_state.pump_pin, LOW);
  system_state.pump_state = 0;
  system_state.ticks_since_pump_engagement = 0;
  delay(50);

  poll_sensors();
  control_pump();
}

void plotRollingTimeSeries(const int sensor) {

  // draw vertical grid lines
  for (unsigned int ii = origin_x; ii <= dim_x; ii += grid_interval_x) {
    for (int jj = origin_y; jj <= dim_y; ++jj) {
      u8g.drawPixel(ii, jj);     
    }
  }
  // draw horizontal grid lines
  for (unsigned int jj = origin_y; jj <= dim_y; jj += grid_interval_y) {
    for (int ii = origin_x; ii < dim_x; ++ii) {
      u8g.drawPixel(ii, jj);  
    }
  }
  // draw time series history
  for (unsigned int ti = 0; ti < min(measurement_trace.time_index, TIME_HISTORY-1); ++ti) {
      // After reaching the end of the time series,
      // begin shuffling all values backwards
      if (measurement_trace.time_index >= (TIME_HISTORY-1)) {
        measurement_trace.history[ti] = measurement_trace.history[ti+1];
      } 
      const int y_pos = measurement_trace.history[ti];
      u8g.drawPixel(origin_x + ti, y_pos);
   }

  // (1) read out raw analog value
  // (2) normalize to [origin_y, dim_y]
  // (3) round, then truncate to integer value
  const int moisture_normalized_pixels = round(max(0, map(analogRead(sensor), 590, 360, origin_y, dim_y)));
  // clamp index of most recent measurement to end of array
  const int current_index = min(measurement_trace.time_index, TIME_HISTORY-1);  
  measurement_trace.history[current_index] = display_y_offset + origin_y + dim_y - moisture_normalized_pixels;

  // draw most recent measurement
  u8g.drawPixel(origin_x + current_index, measurement_trace.history[current_index]);
  ++measurement_trace.time_index;

  // draw semantic name of sensor/relay
  u8g.setFont(u8g_font_6x13);
  u8g.setPrintPos(105, 11);
  switch (sensor) {
    case A0:
      u8g.print("A0");
      break;
    case A1:
      u8g.print("A1");
      break;
    case A2:
      u8g.print("A2");
      break;
    case A3:
      u8g.print("A3");
      break;
  }
  u8g.setPrintPos(11, 11);
  u8g.print(round(max(0, map(analogRead(sensor), 590, 360, 0, 100))), DEC);
  u8g.setPrintPos(25, 11);
  u8g.print("%"); 
  
}

/*
* Main loop
*/
void loop() {
  // read the value from the moisture sensors:
  poll_sensors();
  
  // if any of the plants need to be watered, do that
  control_pump();

  // toggle display screen
  int button_state = digitalRead(system_state.button_pin);
  if (button_state != 1) {
    system_state.display_state = (system_state.display_state + 1) % NUM_DISPLAY_STATES;
    focus_sensor = (focus_sensor + 1) % 4;
  }

  // repaint display
  switch(system_state.display_state) {
    case 0:
    case 1:
    case 2:
    case 3:
      {
        u8g.firstPage();
        do {
          if (system_state.pump_override == 0) {
            plotRollingTimeSeries(system_state.sensor_pinouts[focus_sensor]);
          } else {
            u8g.setPrintPos(10, 30);
            u8g.print("Water source low!");
            u8g.setPrintPos(10, 40);
            u8g.print("Forcing off pump.");
          }
          
        } while ( u8g.nextPage() );      
      }
      break;
    // TODO: add other pages
  }
}

/*
* Read out sensor measurements into normalized (%) values
*/
void poll_sensors() {
  for (unsigned int sensor_index = 0; sensor_index < NUM_SENSORS; ++sensor_index) {
    system_state.normalized_mv[sensor_index] = max(0, map(analogRead(system_state.sensor_pinouts[sensor_index]), MAX_RAW_MEASUREMENT, MIN_RAW_MEASUREMENT, 0, 100));
    delay(20);
  }
}

/*
* Use thresholding to determine relay/pump status
*/
void control_pump() {
  if (system_state.pump_override > 0) {
    return;
  }

  for (unsigned int sensor_index = 0; sensor_index < NUM_SENSORS; ++sensor_index) {
    // ignore hoses that arent plugged in
    if ((system_state.active_relay_mask & (1 << sensor_index)) == 0) {
      continue;
    }

    // plant n needs water?
    if (system_state.normalized_mv[sensor_index] < TOO_LOW) {
      // toggle relay
      digitalWrite(system_state.relay_pinouts[sensor_index], HIGH);
      system_state.error_on_egage[sensor_index] = system_state.normalized_mv[sensor_index];
      system_state.active_relay_mask |= (1 << sensor_index);
      delay(50);

      // if the pump is not already engaged, turn it on
      if (system_state.pump_state == 0) {
        digitalWrite(system_state.pump_pin, HIGH);
        system_state.pump_state = 1;
        delay(50);
      }
    } 
    // plant n has enough water?
    else if (system_state.normalized_mv[sensor_index] > TOO_HIGH) {
      // toggle relay
      digitalWrite(system_state.relay_pinouts[sensor_index], LOW);
      system_state.relay_state_mask &= ~(1 << sensor_index);
      delay(50);

      // if not other relays are open, turn the pump off
      if (system_state.relay_state_mask == 0) {
        digitalWrite(system_state.pump_pin, LOW);
        system_state.pump_state = 0;
        system_state.ticks_since_pump_engagement = 0;
        delay(50);
      }
    }

    // use some crude watchdogging to detect cases where
    // we ought to shut the pump off, water source is empty
    if ((system_state.pump_state == 1) && (system_state.ticks_since_pump_engagement > 10)) {
      for (unsigned int relay_index = 0; relay_index < NUM_SENSORS; ++relay_index) {
        if (abs(int(system_state.error_on_egage[relay_index]) - int(system_state.normalized_mv[sensor_index])) < 10) {
          system_state.pump_override = 1;
          digitalWrite(system_state.pump_pin, LOW);
          delay(50);
        }
      }
    }
    // pump is on, but not long enough that we should 
    // have made meaningful progress
    else if (system_state.pump_state == 1) {
      ++system_state.ticks_since_pump_engagement;
    }
    else {
      system_state.ticks_since_pump_engagement = 0;
    }
  }
}
