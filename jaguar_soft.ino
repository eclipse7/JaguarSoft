/*
  Jaguar XJ X350/X-Type/S-Type head unit button matrix decoder and video selector, with steering wheel ladder resistor detection and HID keyboard output.

  When NAV button is pressed, change video input to HDMI, disable Jag touch matrix and enable USB touch
  When NAV button is held for 5 seconds, change video input to RGBS, enable Jag touch matrix and disable USB touch
  When NAV button is pressed and already in Android mode, send 'F10' keycode via USB to launch Navigation App
  If AUDIO, TEL, CLIMATE or MENU pressed, change video input to RGBS, enable Jag touch matrix and disable USB touch
  if AUDIO is held for 5 seconds, toggle between CD audio and Aux audio input
  If 'Skip Backwards' steering wheel control button pressed and the CD is playing and Aux audio input selected, send KEYCODE_MEDIA_PREVIOUS via USB.
  If 'Skip Forwards' steering wheel control button pressed and the CD is playing and Aux audio input selected,  send KEYCODE_MEDIA_NEXT via USB.
  If ACC switched power goes low, wait for 10 mins then send 'F7' keycode via USB then issue shutdown command and enter sleep mode (configure F7 in Odroid Utility to launch Terminal Emulator)
  IF ACC switched power goes high or Unlock goes high, wake from sleep and restore previous output states

  USER SETTINGS listed underneath these comments:
  Set serialDebug=1 to send debug messages via serial connection.
  Set steeringWheelControl=1 to enable steering wheel audio button detection
  Set poweroff_timer in milliseconds to set shutdown delay

  NOTE: You need to install the HID-Project library into the Arduino IDE for this to compile correctly
  NOTE: Assign F7 in Odroid Utility to launch Terminal Emulator - essential to allow soft-shutdown via USB
  NOTE: Assign F10 in Odroid Utility to launch your preferred Navigation app

  Version 2.7 - edited by Eclipse
  Version 2.5 - Now correctly switches to HDMI mode when NAV pressed when screen is off.
  Version 2.4 - Holding MENU for 5 seconds switches off the LCD panel. Pressing AUDIO, TEL, CLIMATE, NAV or MENU switches the LCD panel back on. LCD panel switched on at power on and after waking from sleep mode
  Version 2.3 - Holding AUDIO for 5 secs switches relay 3 and 4 on to changeover from CD audio to Aux input. Holding NAV for 5 secs switches to Jag mode to access original Jag navigation. EEPROM use removed as now unnecessary due to now having constant power. Cleaned up the Serial debugging code, and removed unneeded i2c code.
  Version 2.2 - Improved debugging for steering wheel button detection. Altered the cdplaying_sensor threshold to allow connection to the main board instead of the CD transport
  Version 2.1 - Reassigned Switch ACC live detect to D0 and changed level detect as now pulled down via opto-isolator. Added wake function on D1 so when car unlocked and footwell lights illuminated the Odroid boots.
  Version 2.0 - Supports relay shield for Odroid powerdown and touch matrix control. Supports Arduino sleep mode for UPS functionallity. I/O has been re-assigned since previous versions so requires wiring changes if upgrading from v1.x versions. Removed ability to hold NAV to revert to OEM Jaguar Nav.
  Version 1.6 - Config option to disable/enable the steering wheel control option, use this depending on whether the CD Player steering wheel control detection mod has been performed or not
  Version 1.5 - Uses new HID-Project library for keyboard control.Also fixed problem where MENU button didn't switch to Jag mode
  Version 1.4 - Outputs keycodes for 'SEARCH+N' when NAV pressed and already in Android mode (run the following from Android terminal to set default app: am start -n com.android.settings/.Settings\$QuickLaunchSettingsActivity)
  Version 1.3 - Last saved state restored correctly
  Version 1.2 - Changed matrix to use Analog inputs instead of pull-down resistors, and adjusted loop and delay timings for more accurate button detection, and add output 12 for OEM IR touch matrix relay control
  Version 1.0 - First version with steering wheel control detection

  Ben Willcox 2016-2017
  ben.willcox@willcoxonline.com
*/

// USER SETTINGS:
//------------------------------------------------------------------------------------------------------
int serialDebug = 0; // 1 = Debug info via serial connection
int steeringWheelControl = 1; // 1 = Enable detection of steering wheel audio controls
unsigned long poweroff_timer = 60000; // delay in ms before shutdown is triggered (60000 is 1 min)
// -----------------------------------------------------------------------------------------------------


// ADVANCED SETTINGS:
//------------------------------------------------------------------------------------------------------
int resistor_ladder_tolerance = 20; // Tolerance applied to steering wheel control input levels
int steeringcontrol_trackdown = 745; // Average Analogue level for 'Track Down' button
int steeringcontrol_trackup = 600; // Average Analogue level for 'Track Up' button
int button_hold_threshold = 4; // Number of loop iterations to trigger a button hold function
int debounce_delay = 200; // Delay after button press to eliminate multiple triggers
int row_scan_delay = 10000; // in microseconds, delay after a column change detection before checking row values
// -----------------------------------------------------------------------------------------------------

#include <Wire.h>
#include <HID-Project.h>
#include <HID-Settings.h>
#include <avr/sleep.h>

unsigned long timer;
int pin_steering_wheel_value = 0;
int pin_cdplaying_sensor_state = 0;
unsigned long button_hold_timer = millis();
boolean last_output_state = LOW; // Jag mode = LOW, Android mode = HIGH
boolean last_audio_state = LOW; // CD = LOW, Aux = HIGH
boolean last_rtdpower_state = LOW;
int button_hold_counter = 0;

// CDC button
boolean button_enable = 1;
int count_button_press = 0;
int count_button_unpress = 0;
//

boolean column_1_state = 0;
boolean last_column_1_state = 0;
boolean column_2_state = 0;
boolean last_column_2_state = 0;
boolean column_3_state = 0;
boolean last_column_3_state = 0;
boolean column_4_state = 0;
boolean last_column_4_state = 0;
int row_a_state = 0;
int row_b_state = 0;
int row_c_state = 0;
int row_d_state = 0;

//Digital Pins ---------------------------------------------------
int pin_acc_detect = 0; // Opto 1
int pin_unlock_detect = 1; // Opto 2
int pin_2; //reserved for i2c
int pin_3; //reserved for i2c
int pin_audio1 = 4; //Relay 4 - Audio 1 - red - cd
int pin_audio2 = 5; //Relay 3 - Audio 2 - green - aux
int pin_touchmatrix_power = 6; //Relay 2 - Jaguar Touch Matrix Power
int pin_odroid_power = 7; //Relay 1 - Odroid Power
int pin_matrix_column1 = 8;
int pin_matrix_column2 = 9;
int pin_matrix_column3 = 10;
int pin_matrix_column4 = 11;
int pin_cdc = 12; // button CDC - white
int pin_resistivetouch_power = 13;
// A0 - pink
// A1 - yellow

//Analogue Pins ---------------------------------------------------
int pin_steeringcontrol_sensor = 0; // Connects to steering wheel audio controls resistor ladder output
int pin_cdplaying_sensor = 1; // Connects to 'CD Playing' output pin
int pin_matrix_rowa = 2;
int pin_matrix_rowb = 3;
int pin_matrix_rowc = 4;
int pin_matrix_rowd = 5;

// i2c RTD control ---------------------------------------------------
int incomingByte = 0;	// for incoming serial data
int controlword[] = {0, 0, 0, 0, 0, 0, 0, 0};
int controlact = 0;
long f = 0;
long ggg = 0;
int g = 0;
byte start_count;
boolean i2c_received;
byte command[8]; //
byte answer[32]; //
byte cur_page;
int i2c_counter;
boolean osd_flag;
boolean parking_flag;
boolean climate_flag;
boolean trip_flag;
boolean time_flag;
boolean cardata_flag;
boolean generalcontrol_flag;
byte left_temp;
byte right_temp;
byte outside_temp;
byte input_control;   // 00 - оставить без изменения, 01-AV1, 02- AV2, 03- HDMI, 04- YPbPr, 05 - VGA
byte power_control;   // 00 -не менять, 01 - выключить, 02-включить
byte backlight_control;
byte bright_control;
byte contrast_control;
byte autolight_control;  //unchanged, 01 -on, 02-off.
byte last_input;   //1=AV1,0=AV2,6=HDMI,7=YPBPR,9=VGA
byte last_board_status;   //1=AV1,0=AV2,6=HDMI,7=YPBPR,9=VGA
//--------------------------------------------------------------------------------------------------------
void setup()
{
  Consumer.begin();             // Keyboard emulation
  Wire.begin(2);                // join i2c bus with address #25
  Wire.onRequest(requestEvent); // register event
  Wire.onReceive(receiveEvent); // register wire.write interrupt event

  pinMode(pin_odroid_power, OUTPUT);
  pinMode(pin_acc_detect, INPUT_PULLUP); // Switched ACC +12v detect line (interrupt)
  pinMode(pin_matrix_column1, INPUT);
  pinMode(pin_matrix_column2, INPUT);
  pinMode(pin_matrix_column3, INPUT);
  pinMode(pin_matrix_column4, INPUT);
  pinMode(pin_touchmatrix_power, OUTPUT);
  pinMode(pin_resistivetouch_power, OUTPUT);
  pinMode(pin_audio1, OUTPUT);
  pinMode(pin_audio2, OUTPUT);
  pinMode(pin_cdc, INPUT_PULLUP);        // CDC button

  if (serialDebug == 1)
  {
    Serial.begin(115200);
  };
  digitalWrite(pin_resistivetouch_power, HIGH);  // turn off android multitouch
  last_output_state = HIGH;
  set_jaguar_mode();
  set_audio_cd();
  rtdpower_on();
  digitalWrite(pin_odroid_power, HIGH);
}
//--------------------------------------------------------------------------------------------------------
void restore_state()
{
  if (last_output_state == HIGH)
  {
    last_output_state = LOW; //Force change to android mode
    set_android_mode();
    debug("Last saved state was LOW");
  }
  else
  {
    last_output_state = HIGH; //Force change to jaguar mode
    set_jaguar_mode();
    debug("Last saved state was HIGH");
  };

  if (last_audio_state == LOW)
  {
    set_audio_cd();
    debug("Last audio state was LOW");
  }
  else
  {
    set_audio_aux();
    debug("Last audio state was HIGH");
  };
  rtdpower_on();
}
//--------------------------------------------------------------------------------------------------------
void loop()
{
  check_buttons();
  if (steeringWheelControl == 1)
  {
    check_steering_controls();
  }
  check_cdc_button();
  check_sleep();
}
//--------------------------------------------------------------------------------------------------------
void requestEvent() {

  if ((answer[0] == 0xee) || (answer[0] == 0xe2))
  {
    Wire.write(answer, 16); // respond with message
  }
  else
  {
    Wire.write(answer, 8); // respond with message
  }
}

//--------------------------------------------------------------------------------------------------------
void receiveEvent(int howMany) {

  while (Wire.available()) // loop through all incoming bytes
  {
    command[i2c_counter] = Wire.read(); // receive byte as a character
    i2c_counter++; // increase index by 1
  }
  i2c_counter = 0;
  process_command();
}

//------------------------------------------------------------------------
void CF_general_control_flag()
{
  generalcontrol_flag = 0;
}

//------------------------------------------------------------------------
void process_command()
{
  if ((command[0] == 0xe0) && (command[1] == 0x01))
  {
    if (command[4] != last_board_status)
    {
      debug("board status = ");

      last_board_status = command[4];
    }
  }
  if ((command[0] == 0xe0) && (command[1] == 0x01) && (command[3] != last_input))
  {
    switch (command[3])  //1=AV1,0=AV2,6=HDMI,7=YPBPR,9=VGA
    {
      case 0:
        debug("input switched to AV2");
        last_input = command[3];
        break;
      case 1:
        debug("input switched to AV1");
        last_input = command[3];
        break;
      case 6:
        debug("input switched to HDMI");
        last_input = command[3];
        break;
      case 7:
        debug("input switched to YPBPR");
        last_input = command[3];
        break;
      case 9:
        debug("input switched to VGA");
        last_input = command[3];
        break;
      default:
        break;
    }
  }

  if ((command[0] == 0xf0))
  { //clear flag page
    if (command[6] == 0xff) CF_general_control_flag();
  }
  else if ((command[0] == 0xe0) && (command[1] == 0x01))
  { //main data page
    cur_page = 1;
    command[0] = 0;
    command[1] = 0;
    answer[0] = 0xE0; //page indicator
    answer[1] = parking_flag; //parking flag
    answer[2] = trip_flag; //trip flag
    answer[3] = climate_flag; //climate flag
    answer[4] = time_flag; //time
    answer[5] = cardata_flag; //car data flag
    answer[6] = generalcontrol_flag; // general control flag
    answer[7] = osd_flag; //reserved
  }
  else if ((command[0] == 0xe1) && (command[1] == 0x01))
  { //send general control data
    cur_page = 2;
    command[0] = 0;
    command[1] = 0;
    answer[0] = 0xE1; //page indicator
    answer[1] = input_control; //input  01=AV, 02=VGA
    answer[2] = bright_control; //bright
    answer[3] = contrast_control; //contrast
    answer[4] = backlight_control; //backlight
    answer[5] = 0x00; //saturation
    answer[6] = power_control; //on/off   01=OFF, 02 = ON
    answer[7] = autolight_control; //autolight control
  }
  else if ((command[0] == 0xee) && (command[1] == 0x01))
  { //send parking data
    cur_page = 3;
  }
  else if ((command[0] == 0xef) && (command[1] == 0x01))
  { //send climate data
    cur_page = 4;
  }
  else if ((command[0] == 0xe2) && (command[1] == 0x01))
  { //send osd config data
    cur_page = 5;
  }
}

void check_buttons()
{
  if ((millis() - button_hold_timer) > 500)
  {
    button_hold_counter = 0;
    button_hold_timer = millis();
  }

  column_1_state = digitalRead(pin_matrix_column1);
  if (column_1_state == HIGH) {
    debug("COL 1 pressed");
    delayMicroseconds(row_scan_delay);
    row_a_state = analogRead(pin_matrix_rowa);
    row_b_state = analogRead(pin_matrix_rowb);
    row_c_state = analogRead(pin_matrix_rowc);

    if (row_a_state == LOW)
    {
      debug("AUDIO pressed");
      rtdpower_on();
      set_jaguar_mode();
    }
    if (row_b_state == LOW)
    {
      debug("TEL pressed");
      rtdpower_on();
      set_android_mode();
      Keyboard.press(KEY_F9);
      Keyboard.releaseAll();
    }
    if (row_c_state == LOW)
    {
      debug("CLIMATE pressed");
      rtdpower_on();
      set_jaguar_mode();
    }
    delay(debounce_delay);
  }

  //  column_2_state = digitalRead(pin_matrix_column2);
  //  if (column_2_state != last_column_2_state)
  //  {
  //    if (column_2_state == HIGH)
  //    {
  //      debug("COL 2 pressed");
  //      delayMicroseconds(row_scan_delay);
  //      row_a_state = analogRead(pin_matrix_rowa);
  //      row_b_state = analogRead(pin_matrix_rowb);
  //      row_c_state = analogRead(pin_matrix_rowc);
  //      row_d_state = analogRead(pin_matrix_rowd);
  //      if (row_a_state == LOW)
  //      {
  //        debug("FAN DOWN pressed");
  //        delay(debounce_delay);
  //      }
  //      if (row_b_state == LOW)
  //      {
  //        debug("AUTO pressed");
  //        delay(debounce_delay);
  //      }
  //      if (row_c_state == LOW)
  //      {
  //        debug("LEFT TEMP DOWN pressed");
  //        delay(debounce_delay);
  //      }
  //      if (row_d_state == LOW)
  //      {
  //        debug("LEFT TEMP UP pressed");
  //        delay(debounce_delay);
  //      }
  //    }
  //  }

  column_3_state = digitalRead(pin_matrix_column3);
  if (column_3_state == HIGH) {
    debug("COL 3 pressed");
    delayMicroseconds(row_scan_delay);
    row_a_state = analogRead(pin_matrix_rowa);
    row_b_state = analogRead(pin_matrix_rowb);
    row_c_state = analogRead(pin_matrix_rowc);
    row_d_state = analogRead(pin_matrix_rowd);
    if ((row_a_state == LOW) && (row_d_state > 10))
    {
      debug("NAV pressed");
      rtdpower_on();
      if (last_output_state == LOW)
      {
        set_android_mode();
      }
      else
      {
        Keyboard.press(KEY_F10);
        Keyboard.releaseAll();
      }
      button_hold_timer = millis();
      button_hold_counter++;
      debug(String(button_hold_counter));
      if (button_hold_counter > button_hold_threshold)
      {
        set_jaguar_mode();
      }
    }
    if (row_b_state == LOW)
    {
      debug("MENU pressed");
      rtdpower_on();
      set_jaguar_mode();
      button_hold_timer = millis();
      button_hold_counter++;
      debug(String(button_hold_counter));
      if (button_hold_counter > 2)
      {
        power_control = 1;
        generalcontrol_flag = 1;
        last_rtdpower_state = LOW;
      }
    }
    if (row_c_state == LOW)
    {
      debug("RECIRC pressed");
    }
    delay(debounce_delay);
  }


  //  column_4_state = digitalRead(pin_matrix_column4);
  //  if (column_4_state != last_column_4_state)
  //  {
  //    if (column_4_state == HIGH)
  //    {
  //      debug("COL 4 pressed");
  //      delayMicroseconds(row_scan_delay);
  //      row_a_state = analogRead(pin_matrix_rowa);
  //      row_b_state = analogRead(pin_matrix_rowb);
  //      row_c_state = analogRead(pin_matrix_rowc);
  //      row_d_state = analogRead(pin_matrix_rowd);
  //      if (row_a_state == LOW)
  //      {
  //        debug("FRONT HEAT pressed");
  //        delay(debounce_delay);
  //      }
  //      if (row_b_state == LOW)
  //      {
  //        debug("REAR HEAT pressed");
  //        delay(debounce_delay);
  //      }
  //      if (row_c_state == LOW)
  //      {
  //        debug("RIGHT TEMP UP pressed");
  //        delay(debounce_delay);
  //      }
  //      if (row_d_state == LOW)
  //      {
  //        debug("RIGHT TEMP DOWN pressed");
  //        delay(debounce_delay);
  //      }
  //    }
  //  }
}

void check_cdc_button() {
  if (digitalRead(pin_cdc) == LOW) {
    if (button_enable) {
      count_button_press++;
      delay(1);
      if (count_button_press > 20) {
        count_button_press = 0;
        button_enable = 0;
        // switch audio mode
        if (last_audio_state == LOW) {
          set_audio_aux();
        } else {
          set_audio_cd();
        }
      }
    }
  } else {
    count_button_press = 0;
    if (button_enable == 0) {
      count_button_unpress++;
      if (count_button_unpress > 200) {
        count_button_unpress = 0;
        button_enable = 1;
      }
    }
  }
}

void set_android_mode()
{
  if (last_output_state == LOW)
  {
    digitalWrite(pin_touchmatrix_power, HIGH);
    digitalWrite(pin_resistivetouch_power, LOW);
    debug("Setting Android mode");
    last_output_state = HIGH;
    input_control = 3;
    generalcontrol_flag = 1;
  }
}

void set_jaguar_mode()
{
  if (last_output_state == HIGH)
  {
    digitalWrite(pin_touchmatrix_power, LOW);
    digitalWrite(pin_resistivetouch_power, HIGH);
    debug("Setting Jaguar mode");
    last_output_state = LOW;
    input_control = 5;
    generalcontrol_flag = 1;
  }
}

void set_audio_aux()
{
  digitalWrite(pin_audio1, LOW);
  digitalWrite(pin_audio2, HIGH);
  last_audio_state = HIGH;
  debug("Setting Audio Aux (Relays on)");
}

void set_audio_cd()
{
  digitalWrite(pin_audio1, HIGH);
  digitalWrite(pin_audio2, LOW);
  last_audio_state = LOW;
  debug("Setting Audio CD (Relays off)");
}

void check_steering_controls()
{
  pin_steering_wheel_value = analogRead(pin_steeringcontrol_sensor);
  pin_cdplaying_sensor_state = analogRead(pin_cdplaying_sensor);

  if ((pin_steering_wheel_value > 100) && (pin_steering_wheel_value < 1000))
  {
    debug("Steering Wheel Button Voltage:" + String(pin_steering_wheel_value)); // send debug value if anything is detected
    debug("CD Playing Flag Voltage:" + String(pin_cdplaying_sensor_state));
  }

  if ((pin_steering_wheel_value < steeringcontrol_trackup + resistor_ladder_tolerance) && (pin_steering_wheel_value > steeringcontrol_trackup - resistor_ladder_tolerance) && (pin_cdplaying_sensor_state < 512) && (last_audio_state == HIGH))
  {
    Consumer.write(MEDIA_NEXT);
    debug("KEYCODE_MEDIA_NEXT");
    delay(debounce_delay);
  }
  if ((pin_steering_wheel_value < steeringcontrol_trackdown + resistor_ladder_tolerance) && (pin_steering_wheel_value > steeringcontrol_trackdown - resistor_ladder_tolerance) && (pin_cdplaying_sensor_state < 512) && (last_audio_state == HIGH))
  {
    Consumer.write(MEDIA_PREVIOUS);
    debug("KEYCODE_MEDIA_PREVIOUS");
    delay(debounce_delay);
  }
}

void rtdpower_on()
{
  if (last_rtdpower_state == LOW)
  {
    power_control = 2;
    generalcontrol_flag = 1;
    last_rtdpower_state = HIGH;
    delay(1000);
  }
}

void debug(String output_string)
{
  if (serialDebug == 1)
  {
    Serial.println(output_string);
  }
}

void processInterrupt()
{
  if (digitalRead(pin_acc_detect) == HIGH) // If ignition off and footwell lights go off, stay asleep
  {
    sleep();
  }
}

void sleep()
{
  debug("Sleeping...");
  delay(1000);
  digitalWrite(pin_odroid_power, LOW);
  digitalWrite(pin_touchmatrix_power, LOW);
  digitalWrite(pin_resistivetouch_power, HIGH);
  digitalWrite(pin_audio1, LOW);
  digitalWrite(pin_audio2, LOW);
  noInterrupts ();           // make sure we don't get interrupted before we sleep
  //attachInterrupt(digitalPinToInterrupt(pin_unlock_detect), processInterrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pin_acc_detect), processInterrupt, CHANGE);
  EIFR |= 0x01; // clear any queued interrupts
  set_sleep_mode (SLEEP_MODE_PWR_DOWN);
  sleep_enable ();          // enables the sleep bit in the mcucr register
  interrupts ();           // interrupts allowed now, next instruction WILL be executed
  USBCON |= _BV(FRZCLK);
  PLLCSR &= ~_BV(PLLE);
  USBCON &= ~_BV(USBE);
  sleep_cpu ();            // here the device is put to sleep
  // Sleeping now, waiting for interrupt
  //detachInterrupt(digitalPinToInterrupt(pin_unlock_detect));
  detachInterrupt(digitalPinToInterrupt(pin_acc_detect));
  sleep_disable ();         // first thing after waking from sleep:
  delay(100);
  USBDevice.attach();
  delay(100);
  if (serialDebug == 1)
  {
    Serial.begin(115200);  // start serial for output
  }
  delay(100);
  debug("Waking...");
  digitalWrite(pin_odroid_power, HIGH);
  timer = millis();
  restore_state();
}

void check_sleep()
{
  //debug("ACC"+String(digitalRead(pin_acc_detect)));
  //debug("Unlock"+String(digitalRead(pin_unlock_detect)));
  if (digitalRead(pin_acc_detect) == HIGH)
  {
    if (millis() - timer > poweroff_timer)
    {
      Keyboard.press(KEY_F7);
      Keyboard.releaseAll();
      delay(5000); // delay to allow terminal to load
      Keyboard.print("reboot -p\n");
      delay(5000); // delay to allow full Odroid shutdown
      sleep();
    }
  }
  else
  {
    timer = millis();
  }
}

