// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands.LEDs;

import org.team2168.subsystems.LEDs;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class RedWhiteBlueLEDs extends CommandBase {
  /** Creates a new RedWhiteBlueLEDs. */
  private LEDs leds;
  private double timecount = 0.0;
  public RedWhiteBlueLEDs(LEDs leds) {
    this.leds = leds;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ++timecount;
    leds.redWhiteBlueLED();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
