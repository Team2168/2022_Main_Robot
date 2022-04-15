// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands.shootingpositions;

import org.team2168.Constants;
import org.team2168.RobotContainer;
import org.team2168.subsystems.Drivetrain;
import org.team2168.subsystems.Hood;
import org.team2168.subsystems.Limelight;
import org.team2168.subsystems.Shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ShootBasedOnDistance extends CommandBase {
  /** Creates a new ShootBasedOnDistance. */
  Limelight lime;
  Shooter shooter;
  Hood hood;
  Drivetrain drive;

  double limelightDistance;
  double pastLimelightDist = 0.0;
  double predictedTravelDistY = 0.0;
  double predictedTravelDistX = 0.0;
  double predictAddedDistFromHub = 0.0;
  double signOfDistanceChangeY = 1.0;
  double signofDistanceChangeX = 1.0;

  Pose2d pastPose;
  Pose2d currentPose;

  double shooterRPM;
  double hoodAngle;

  final double LOOP_TIME_SECS = 0.02;
  final double SHOT_TAKEN_TIME = 0.1;
  public ShootBasedOnDistance(Shooter shooter, Hood hood, Limelight lime, Drivetrain drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter, hood);
    this.lime = lime;
    this.shooter = shooter;
    this.hood = hood;
    this.drive = drive;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentPose = drive.getPose();
    limelightDistance = lime.calcDistanceMeters();
    shooterRPM = shooter.getRPMfromDistance(limelightDistance + predictAddedDistFromHub);
    hoodAngle = hood.getHoodAnglefromDistance(limelightDistance + predictAddedDistFromHub);

    predictedTravelDistY = (currentPose.getY() - pastPose.getY()) * (SHOT_TAKEN_TIME/LOOP_TIME_SECS);
    predictedTravelDistX = (currentPose.getX() - pastPose.getX()) * (SHOT_TAKEN_TIME/LOOP_TIME_SECS);

    if (Math.abs(currentPose.getY() + predictedTravelDistY - Constants.FieldPositions.HUB_Y_METERS) >= Math.abs(currentPose.getY() - Constants.FieldPositions.HUB_Y_METERS)) {
      signOfDistanceChangeY = 1.0;
    }
    else {
      signOfDistanceChangeY = -1.0;
    }

    if (Math.abs(currentPose.getX() + predictedTravelDistX - Constants.FieldPositions.HUB_X_METERS) >= Math.abs(currentPose.getX() - Constants.FieldPositions.HUB_X_METERS)) {
      signofDistanceChangeX = 1.0;
    }
    else {
      signofDistanceChangeX = -1.0;
    }

    predictAddedDistFromHub = Math.sqrt((Math.pow(predictedTravelDistY, 2) * signOfDistanceChangeY) + (Math.pow(predictedTravelDistX , 2) * signofDistanceChangeX));

    if (!RobotContainer.getInstance().isFiring()) {
      shooter.setSpeed(shooterRPM);
      hood.setPosition(hoodAngle);
    }
    shooter.setWaitForShooterAtSpeed(false);
    pastPose = currentPose;
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
