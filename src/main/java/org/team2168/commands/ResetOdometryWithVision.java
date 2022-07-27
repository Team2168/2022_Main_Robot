// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands;

import org.team2168.Constants;
import org.team2168.subsystems.Drivetrain;
import org.team2168.subsystems.Limelight;
import org.team2168.subsystems.Turret;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ResetOdometryWithVision extends CommandBase {
  /** Creates a new ResetOdometryWithVision. */
  Turret turret;
  Drivetrain drive;
  Limelight lime;

  double limeAdjustPoseX;
  double limeAdjustPoseY;
  double limeAdjustPoseAngle;
  double angleOffset = 0.0;

  double poseDistFromHubX;

  double errorToleranceAngle;
  public ResetOdometryWithVision(Drivetrain drive, Turret turret, Limelight lime) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.turret = turret;
    this.drive = drive;
    this.lime = lime;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    limeAdjustPoseY = Constants.FieldPositions.HUB_Y_METERS - Math.sin(Units.degreesToRadians(-turret.getPositionDegrees() + drive.getHeading())) * lime.getDistanceMetersToCenterHub();
    errorToleranceAngle = 1.0;

    poseDistFromHubX = Math.cos(Units.degreesToRadians(-turret.getPositionDegrees() + drive.getHeading())) * lime.getDistanceMetersToCenterHub();

    if (limeAdjustPoseY < Constants.FieldPositions.HUB_Y_METERS) {
      limeAdjustPoseX = Constants.FieldPositions.HUB_X_METERS - poseDistFromHubX;
    }
    else {
      limeAdjustPoseX = Constants.FieldPositions.HUB_X_METERS + poseDistFromHubX;
    }

    if (limeAdjustPoseAngle > 180.0) {
      angleOffset = angleOffset - 360.0;
    }
    else if (limeAdjustPoseAngle < -180.0) {
      angleOffset = angleOffset + 360.0;
    }

    limeAdjustPoseAngle = drive.getHeading() + angleOffset;

    if (lime.getPositionX() < errorToleranceAngle) {
      drive.resetOdometry(new Pose2d(limeAdjustPoseX, limeAdjustPoseY, new Rotation2d(Units.degreesToRadians(limeAdjustPoseAngle))), new Rotation2d(Units.degreesToRadians(limeAdjustPoseAngle)));
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
