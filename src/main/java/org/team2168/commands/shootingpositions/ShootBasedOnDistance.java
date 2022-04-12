// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands.shootingpositions;

import org.team2168.RobotContainer;
import org.team2168.subsystems.Hood;
import org.team2168.subsystems.Limelight;
import org.team2168.subsystems.Shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ShootBasedOnDistance extends CommandBase {
  /** Creates a new ShootBasedOnDistance. */
  Limelight lime;
  Shooter shooter;
  Hood hood;

  double limelightDistance;
  double pastLimelightDist = 0.0;
  double shooterRPM;
  double hoodAngle;
  public ShootBasedOnDistance(Shooter shooter, Hood hood, Limelight lime) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter, hood);
    this.lime = lime;
    this.shooter = shooter;
    this.hood = hood;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    limelightDistance = lime.calcDistanceMeters();
    shooterRPM = shooter.getRPMfromDistance(limelightDistance);
    hoodAngle = hood.getHoodAnglefromDistance(limelightDistance);

    if (limelightDistance < 4.0) {
      lime.setPipeline(Limelight.PIPELINE_TARMAC_LINE);
    }
    else if (limelightDistance >= 4.0 && limelightDistance < 6.0) {
      lime.setPipeline(Limelight.PIPELINE_LAUNCHPAD_LINE);
    }
    else {
      lime.setPipeline(Limelight.PIPELINE_TERMINAL);
    }

    if (RobotContainer.getInstance().isFiring()) {
      shooter.setSpeed(shooterRPM);
      hood.setPosition(hoodAngle);
    }
    shooter.setWaitForShooterAtSpeed(false);
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
