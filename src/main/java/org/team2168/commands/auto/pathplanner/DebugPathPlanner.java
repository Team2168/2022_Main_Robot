// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands.auto.pathplanner;

import java.io.File;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.security.Timestamp;
import java.util.UUID;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import org.team2168.Constants;
import org.team2168.subsystems.Drivetrain;
import org.team2168.utils.PathUtil;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;

public class DebugPathPlanner extends CommandBase {
  Drivetrain drivetrain;
  RamseteCommand rCommand;
  double initialTimestep;
  Pose2d initialPose;

  double accel;
  double vel;
  double curvature;
  double time;
  String pathname;
  StringBuilder out = new StringBuilder("time,expected velocity,actual velocity,expected_curvature,actualturningrate\n");

  /** Creates a new DebugPath. */
  public DebugPathPlanner(Drivetrain drivetrain, String pathname) {
    this.drivetrain = drivetrain; 
    this.pathname = pathname;

    try {
    var trajectory = PathUtil.getPathPlannerTrajectory(pathname, true);
    initialPose = trajectory.getInitialPose();
    

    rCommand = new RamseteCommand(
      trajectory,
      drivetrain::getPose,
      new RamseteController(Constants.Drivetrain.kRamseteB, Constants.Drivetrain.kRamseteZeta),
      new SimpleMotorFeedforward(
              Constants.Drivetrain.ksVolts,
              Constants.Drivetrain.kvVoltSecondsPerMeter,
              Constants.Drivetrain.kaVoltSecondsSquaredPerMeter),
      Constants.Drivetrain.kDriveKinematics,
      drivetrain::getWheelSpeeds,
      new PIDController(Constants.Drivetrain.kPDriveVel, 0, 0),
      new PIDController(Constants.Drivetrain.kPDriveVel, 0, 0),
      // RamseteCommand passes volts to the callback
      (l, r) -> {

        // Sample data to be puked out to nt by oblog
        double time = Timer.getFPGATimestamp() - initialTimestep;
        this.time = time;
        var data = trajectory.sample(time);
        vel = data.velocityMetersPerSecond;
        accel = data.accelerationMetersPerSecondSq;
        curvature = data.curvatureRadPerMeter;

        drivetrain.tankDriveVolts(l, r);
      },
      drivetrain);
    } catch (IOException e) {
      initialPose = new Pose2d();
    }
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drivetrain.resetOdometry(initialPose, false);
    initialTimestep = Timer.getFPGATimestamp();
    rCommand.schedule();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    final String ENTRY = "%f,%f,%f,%f,%f%n";
    SmartDashboard.putNumber("commanded velocity", vel);
    SmartDashboard.putNumber("commanded accelleration", accel);
    SmartDashboard.putNumber("commanded curvature", curvature);
    //print values used to plot commanded vs actual velocity 
    out.append(String.format(ENTRY, time, vel, (drivetrain.getLeftEncoderRate() + drivetrain.getRightEncoderRate())/2.0, drivetrain.getTurnRate(), curvature));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    rCommand.end(interrupted);
    System.out.println(out.toString());
//    try {
//      var fileName = System.currentTimeMillis() + ".csv";
//      var file = Filesystem.getOperatingDirectory().toPath().resolve(fileName);
//      Files.writeString(file, out);
//      System.out.println("wrote to " + fileName);
//    } catch (IOException e) {
//      DriverStation.reportError("Failed to write path logs!", e.getStackTrace());
//    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return rCommand.isFinished();
  }
}
