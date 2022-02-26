// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands.LEDs;

import org.team2168.subsystems.LEDs;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class SetGreenLED extends CommandBase {
  /** Creates a new SetGreenLED. */
  private LEDs leds;
  private boolean isOn;

  /**
   * Sets the green LED on/off
   * @param leds the LED instance
   * @param isOn whether the green LED should be on (true) or off (false)
   */
  public SetGreenLED(LEDs leds, boolean isOn) {
    this.leds = leds;
    this.isOn = isOn;

    addRequirements(leds);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    leds.green(isOn);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
