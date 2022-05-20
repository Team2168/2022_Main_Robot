// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands.LEDs;

import org.team2168.subsystems.LEDs;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class SetAllLEDs extends CommandBase {
  /** Creates a new SetAllLEDs. */
  private LEDs leds;
  private boolean redOn;
  private boolean blueOn;
  private boolean greenOn;
/**
 * Single method for setting all of the LED colors on/off
 * @param leds the LED instance
 * @param redOn whether the red LED should be on
 * @param blueOn whether the blue LED should be on
 * @param greenOn whether the green LED should be on
 */
  public SetAllLEDs(LEDs leds, boolean redOn, boolean blueOn, boolean greenOn) {
    this.leds = leds;
    this.redOn = redOn;
    this.blueOn = blueOn;
    this.greenOn = greenOn;

    addRequirements(leds);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    leds.red(redOn);
    leds.blue(blueOn);
    leds.green(greenOn);
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
