package com.team5959.subsystems;

import edu.wpi.first.hal.CANAPIJNI;

public class TejuinoBoard {


  public final int TEJUINO_DEVICE_NUMBER_0 = 0;
  public final int TEJUINO_DEVICE_NUMBER_1 = 1;
  //Led strip names
  public final int TEJUINO_ONBOARD_LEDS = 0;
  public final int TEJUINO_EXTERNAL_LEDS = 1;
  public final int TEJUINO_EXTERNAL2_LEDS = 2;


  public final int LED_STRIP_0 = TEJUINO_ONBOARD_LEDS;
  public final int LED_STRIP_1 = TEJUINO_EXTERNAL_LEDS;
  public final int LED_STRIP_2 = TEJUINO_EXTERNAL2_LEDS;

  private static final int TEJUINO_MANUFACTURER = 8;
  private static final int TEJUINO_DEVICE_TYPE = 10;

  private static final int TEJUINO_LED_API_ID = 0;
  private static final int TEJUINO_GPIO_API_ID = 1; //Reserved for V2

 
  private int can_handle = 0;

  
  public void init(int tejuino_device_number) {
    can_handle = CANAPIJNI.initializeCAN(TEJUINO_MANUFACTURER, tejuino_device_number, TEJUINO_DEVICE_TYPE);
  }

  public void single_led_control(int led_strip, int led_number, int red, int green, int blue) {
    byte[] data = new byte[8];
    data[0] = (byte) led_number;
    data[1] = (byte) 0;
    data[2] = (byte) red;
    data[3] = (byte) green;
    data[4] = (byte) blue;
    data[5] = (byte) 0;
    data[6] = (byte) 0;
    data[7] = (byte) led_strip;
    CANAPIJNI.writeCANPacket(can_handle, data, TEJUINO_LED_API_ID);
  }

  public void all_led_control(int led_strip, int red, int green, int blue) {
    byte[] data = new byte[8];
    data[0] = (byte) 0;
    data[1] = (byte) 0;
    data[2] = (byte) red;
    data[3] = (byte) green;
    data[4] = (byte) blue;
    data[5] = (byte) 0;
    data[6] = (byte) 1;
    data[7] = (byte) led_strip;
    CANAPIJNI.writeCANPacket(can_handle, data, TEJUINO_LED_API_ID);
  }

  public void all_leds_red(int led_strip) {
    byte[] data = new byte[8];
    data[0] = (byte) 0;
    data[1] = (byte) 0;
    data[2] = (byte) 255;
    data[3] = (byte) 0;
    data[4] = (byte) 0;
    data[5] = (byte) 0;
    data[6] = (byte) 1;
    data[7] = (byte) led_strip;
    CANAPIJNI.writeCANPacket(can_handle, data, TEJUINO_LED_API_ID);
  }

  public void all_leds_green(int led_strip) {
    byte[] data = new byte[8];
    data[0] = (byte) 0;
    data[1] = (byte) 0;
    data[2] = (byte) 0;
    data[3] = (byte) 255;
    data[4] = (byte) 0;
    data[5] = (byte) 0;
    data[6] = (byte) 1;
    data[7] = (byte) led_strip;
    CANAPIJNI.writeCANPacket(can_handle, data, TEJUINO_LED_API_ID);
  }
  public void all_leds_blue(int led_strip) {
    byte[] data = new byte[8];
    data[0] = (byte) 0;
    data[1] = (byte) 0;
    data[2] = (byte) 0;
    data[3] = (byte) 0;
    data[4] = (byte) 255;
    data[5] = (byte) 0;
    data[6] = (byte) 1;
    data[7] = (byte) led_strip;
    CANAPIJNI.writeCANPacket(can_handle, data, TEJUINO_LED_API_ID);
  }

  public void all_leds_white(int led_strip) {
    byte[] data = new byte[8];
    data[0] = (byte) 0;
    data[1] = (byte) 0;
    data[2] = (byte) 255;
    data[3] = (byte) 255;
    data[4] = (byte) 255;
    data[5] = (byte) 0;
    data[6] = (byte) 1;
    data[7] = (byte) led_strip;
    CANAPIJNI.writeCANPacket(can_handle, data, TEJUINO_LED_API_ID);
  }

  public void all_leds_purple(int led_strip) {
    byte[] data = new byte[8];
    data[0] = (byte) 0;
    data[1] = (byte) 0;
    data[2] = (byte) 153;
    data[3] = (byte) 51;
    data[4] = (byte) 255;
    data[5] = (byte) 0;
    data[6] = (byte) 1;
    data[7] = (byte) led_strip;
    CANAPIJNI.writeCANPacket(can_handle, data, TEJUINO_LED_API_ID);
  }

  public void all_leds_yellow(int led_strip) {
    byte[] data = new byte[8];
    data[0] = (byte) 0;
    data[1] = (byte) 0;
    data[2] = (byte) 255;
    data[3] = (byte) 255;
    data[4] = (byte) 0;
    data[5] = (byte) 0;
    data[6] = (byte) 1;
    data[7] = (byte) led_strip;
    CANAPIJNI.writeCANPacket(can_handle, data, TEJUINO_LED_API_ID);
  }

  public void turn_off_all_leds(int led_strip) {
    byte[] data = new byte[8];
    data[0] = (byte) 0;
    data[1] = (byte) 0;
    data[2] = (byte) 0;
    data[3] = (byte) 0;
    data[4] = (byte) 0;
    data[5] = (byte) 0;
    data[6] = (byte) 1;
    data[7] = (byte) led_strip;
    CANAPIJNI.writeCANPacket(can_handle, data, TEJUINO_LED_API_ID);
  }

  public void rainbow_effect(int led_strip) {
    byte[] data = new byte[8];
    data[0] = (byte) 0;
    data[1] = (byte) 0;
    data[2] = (byte) 0;
    data[3] = (byte) 0;
    data[4] = (byte) 0;
    data[5] = (byte) 1;
    data[6] = (byte) 0;
    data[7] = (byte) led_strip;
    CANAPIJNI.writeCANPacket(can_handle, data, TEJUINO_LED_API_ID);
  }

  public void stingbot_effect(int led_strip) {
    byte[] data = new byte[8];
    data[0] = (byte) 0;
    data[1] = (byte) 0;
    data[2] = (byte) 0;
    data[3] = (byte) 0;
    data[4] = (byte) 0;
    data[5] = (byte) 2;
    data[6] = (byte) 0;
    data[7] = (byte) led_strip;
    CANAPIJNI.writeCANPacket(can_handle, data, TEJUINO_LED_API_ID);
  }

    public void escuderia_effect(int led_strip) {
    byte[] data = new byte[8];
    data[0] = (byte) 0;
    data[1] = (byte) 0;
    data[2] = (byte) 0;
    data[3] = (byte) 0;
    data[4] = (byte) 0;
    data[5] = (byte) 3;
    data[6] = (byte) 0;
    data[7] = (byte) led_strip;
    CANAPIJNI.writeCANPacket(can_handle, data, TEJUINO_LED_API_ID);
  }



}
