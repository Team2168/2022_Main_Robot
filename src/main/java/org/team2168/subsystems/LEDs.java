// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.subsystems;

import org.team2168.Constants;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.annotations.Log;

public class LEDs extends SubsystemBase {
  
  public DigitalOutput redLED;
  public DigitalOutput greenLED;
  public DigitalOutput blueLED;

  static LEDs instance = null;

  public LEDs() {
    redLED = new DigitalOutput(Constants.DIO.RED_LED);
    greenLED = new DigitalOutput(Constants.DIO.GREEN_LED);
    blueLED = new DigitalOutput(Constants.DIO.BLUE_LED);
  }

  /**
   * Sets the red LED on/off
   * @param isOn whether the red LED should be on (true) or off (false)
   */
  public void red(boolean isOn) {
    redLED.set(!isOn);
  }

  /**
   * Sets the green LED on/off
   * @param isOn whether the green LED should be on (true) or off (false)
   */
  public void green(boolean isOn) {
    greenLED.set(!isOn);
  }

  /**
   * Sets the blue LED on/off
   * @param isOn whether the blue LED should be on (true) or off (false)
   */
  public void blue(boolean isOn) {
    blueLED.set(!isOn);
  }

  @Log(name = "Red On?", rowIndex = 0, columnIndex = 0)
  public boolean getRedState() {
    return !redLED.get();
  }

  @Log(name = "Green On?", rowIndex = 0, columnIndex = 1)
  public boolean getGreenState() {
    return !greenLED.get();
  }

  @Log(name = "Blue On?", rowIndex = 0, columnIndex = 2)
  public boolean getBlueState() {
    return !blueLED.get();
  }

  public static LEDs getInstance() {
    if (instance == null) 
      instance = new LEDs();
    return instance;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
