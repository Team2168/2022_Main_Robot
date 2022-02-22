// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands.shooter;

import org.team2168.subsystems.Shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveShooterToSpeed extends CommandBase {
  Shooter shooter;
  double target_speed;

  /**
   * 
   * @param s the shooter instance
   * @param speed_rpm target speed in RPMs
   */
  public DriveShooterToSpeed(Shooter s, double speed_rpm) {
    shooter = s;
    target_speed = speed_rpm;
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.setSpeed(target_speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.setVelocityAdjustment(0.0);
    shooter.setSpeed(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
