// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands.LEDs;

import org.team2168.subsystems.Climber;
import org.team2168.subsystems.LEDs;
import org.team2168.subsystems.Shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ShowShooterAtSpeed extends CommandBase {
  /** Creates a new ShowShooterAtSpeed. */
  private LEDs leds;
  private Shooter shooter;
  private Climber climber;

  public ShowShooterAtSpeed(LEDs leds, Shooter shooter, Climber climber) {
    this.leds = leds;
    this.shooter = shooter;
    this.climber = climber;

    addRequirements(leds);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (climber.isClimberHookAttached()) {
      leds.red(false);
      leds.green(false);
      leds.blue(true);
    } else if (shooter.isAtSpeed()) {
      leds.red(false);
      leds.green(true);
      leds.blue(false);
    } else {
      leds.red(true);
      leds.blue(false);
      leds.green(false);
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
