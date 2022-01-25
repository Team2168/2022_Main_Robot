// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.team2168.subsystems.Shooter;

public class DriveShooterWithConstant extends CommandBase {
  /** Creates a new DriveShooterWithConstant. */

  private double Speed;
  private Shooter Shooter;

  public DriveShooterWithConstant(double k_speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    Shooter = Shooter.getInstance();
    addRequirements(Shooter);
    Speed = k_speed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Shooter.setSpeed(Speed);
    //this doesn't drive yet because i haven't made the drive method yet
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if interrupted == true;{
      isFinished();
    }
    else{
      Shooter.driveShooterMotors(0);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
