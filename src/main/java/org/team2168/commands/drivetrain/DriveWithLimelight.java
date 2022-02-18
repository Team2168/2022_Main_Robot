// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands.drivetrain;

import org.team2168.subsystems.Drivetrain;
import org.team2168.subsystems.Limelight;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveWithLimelight extends CommandBase {
  /** Creates a new DriveWithLimelight. */
  private Drivetrain dt;
  private Limelight lime;

  private double errorToleranceAngle = 1.0; // in degrees
  private double limeAngle;
  
  //limelight gains
  private double limelightkP = 0.25;
  private double limelightkFeedForward = 0.1; // both completely arbitrary values

  private double driveLimeTurnSpeed;
  
  public DriveWithLimelight(Drivetrain drivetrain, Limelight limelight) {
    addRequirements(limelight);
    lime = limelight;
    dt = drivetrain;

    limeAngle = lime.getPositionX();
    driveLimeTurnSpeed = ((limeAngle/lime.MAX_POSITIVE_ANGLE) * limelightkP) + limelightkFeedForward;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (!lime.isLimelightEnabled()) {
      lime.enableLimelight();
    }
    lime.setPipeline(lime.getDrivePipeline());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    dt.arcadeDrive(0, driveLimeTurnSpeed);
    // System.out.println(driveLimeTurnSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    dt.arcadeDrive(0.0, 0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (limeAngle >= 0) {
      return (limeAngle <= errorToleranceAngle);
    }
    else {
      return (limeAngle >= -errorToleranceAngle);
    }
  }
}
