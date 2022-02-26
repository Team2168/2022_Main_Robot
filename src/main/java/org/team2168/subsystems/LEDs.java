// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.subsystems;

import org.team2168.Constants;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDs extends SubsystemBase {
  
  public DigitalOutput redLED;
  public DigitalOutput greenLED;
  public DigitalOutput blueLED;

  public LEDs() {
    redLED = new DigitalOutput(Constants.DIO.RED_LED);
    greenLED = new DigitalOutput(Constants.DIO.GREEN_LED);
    blueLED = new DigitalOutput(Constants.DIO.BLUE_LED);
  }

  public void red(boolean isOn) {
    redLED.set(!isOn);
  }

  public void green(boolean isOn) {
    greenLED.set(!isOn);
  }

  public void blue(boolean isOn) {
    blueLED.set(!isOn);
  }

  public boolean getRedState() {
    return !redLED.get();
  }

  public boolean getGreenState() {
    return !greenLED.get();
  }

  public boolean getBlueState() {
    return !blueLED.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
