// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands.LEDs;

import org.team2168.subsystems.LEDs;
import org.team2168.subsystems.Shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ShowShooterAtSpeed extends CommandBase {
  /** Creates a new ShowShooterAtSpeed. */
  private LEDs leds;
  private Shooter shooter;

  public ShowShooterAtSpeed(LEDs leds, Shooter shooter) {
    this.leds = leds;
    this.shooter = shooter;

    addRequirements(leds);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(shooter.isAtSpeed(15)) {
      leds.red(false);
      leds.blue(false);
      leds.green(true);
    }
    else {
      leds.blue(false);
      leds.green(false);
      leds.red(true);
    }
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
