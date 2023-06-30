// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.subsystems;

import org.team2168.Constants.PneumaticsDevices;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.annotations.Log;

public class LEDs extends SubsystemBase {

  private static Solenoid redLED;
  private static Solenoid greenLED;
  private static Solenoid blueLED;

  private double timecount = 0.0;

  static LEDs instance = null;

  public LEDs() {
    redLED = new Solenoid(PneumaticsDevices.MODULE_TYPE, PneumaticsDevices.RED_LED);
    greenLED = new Solenoid(PneumaticsDevices.MODULE_TYPE, PneumaticsDevices.GREEN_LED);
    blueLED = new Solenoid(PneumaticsDevices.MODULE_TYPE, PneumaticsDevices.BLUE_LED);
  }

  /**
   * Sets the red LED on/off
   * @param isOn whether the red LED should be on (true) or off (false)
   */
  public static void red(boolean isOn) {
    redLED.set(isOn);
  }

  /**
   * Sets the green LED on/off
   * @param isOn whether the green LED should be on (true) or off (false)
   */
  public static void green(boolean isOn) {
    greenLED.set(isOn);
  }

  /**
   * Sets the blue LED on/off
   * @param isOn whether the blue LED should be on (true) or off (false)
   */
  public static void blue(boolean isOn) {
    blueLED.set(isOn);
  }

  @Log(name = "Red On?", rowIndex = 0, columnIndex = 0)
  public boolean getRedState() {
    return redLED.get();
  }

  @Log(name = "Green On?", rowIndex = 0, columnIndex = 1)
  public boolean getGreenState() {
    return greenLED.get();
  }

  @Log(name = "Blue On?", rowIndex = 0, columnIndex = 2)
  public boolean getBlueState() {
    return blueLED.get();
  }

  public static LEDs getInstance() {
    if (instance == null) 
      instance = new LEDs();
    return instance;
  }

  public static void setLED(boolean redOn, boolean greenOn, boolean blueOn) {
    red(redOn);
    green(greenOn);
    blue(blueOn);
  }

  public void redWhiteBlueLED() {

    if (timecount >= 0 && timecount <= 10) {
    setLED(true, false, false);
  } else if (timecount >= 10 && timecount <= 20) {
    setLED(true, true, true);
    } else if (timecount >= 20 && timecount <= 30) {
      setLED(false, false, true);
  }
}

  @Override
  public void periodic() {
    ++timecount;

    if(timecount > 30)
      timecount = 0.0;
    // This method will be called once per scheduler run
  }
}
