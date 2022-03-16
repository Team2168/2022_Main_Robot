// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands.drivetrain;

import org.team2168.Constants;
import org.team2168.OI;
import org.team2168.subsystems.Drivetrain;
import org.team2168.subsystems.Limelight;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;

import java.util.function.DoubleSupplier;

public class DriveWithLimelight extends CommandBase implements Loggable {
  /** Creates a new DriveWithLimelight. */
  private Drivetrain dt;
  private Limelight lime;
  private PIDController pid;
  private DoubleSupplier joystickInput;

  private static double DEFAULT_MAXANGLE = 0.5;
  private double errorToleranceAngle; // in degrees
  private double limeAngle;
  private int withinThresholdLoops = 0;
  private int acceptableLoops = 10;

  private static final double P_FAR;
  private static final double P_NEAR;
  private static final double I_FAR;
  private static final double I_NEAR;
  private static final double MINIMUM_COMMAND;   // TODO normalize for battery voltage
  private static final double MAX_INTEGRAL;
  
  static {
    if(Constants.IS_COMPBOT) {
      P_FAR = 0.0021;
      P_NEAR = 0.0041;
      I_FAR = 0.001;
      I_NEAR = 0.001;
      MINIMUM_COMMAND = 0.05;
      MAX_INTEGRAL = 1.0;
    } else {
      P_FAR = 0.02;
      P_NEAR = 0.01;
      I_FAR = 0.002;
      I_NEAR = 0.001;
      MINIMUM_COMMAND = 0.25;
      MAX_INTEGRAL = 1.0;
    }
  }

  //limelight gains
  private double P;
  private double I;
  private double D = 0.0;

  @Config
  void setLimeP(int P) {
    this.P = P;
  }
  @Config
  void setLimeI(int I) {
    this.I = I;
  }
  @Config
  void setLimeD(int D) {
    this.D = D;
  }

  //determines whether joystick should be used or not
  private boolean manualControl;

  //speed of drivetrain rotation
  @Log(name = "Turn Speed")
  private double driveLimeTurnSpeed;

  public DriveWithLimelight(Drivetrain drivetrain, Limelight limelight) {
    this(drivetrain, limelight, () -> 0.0);
    manualControl = false;
  }

  public DriveWithLimelight(Drivetrain drivetrain, Limelight limelight, double acceptableAngle, boolean near) {
    this(drivetrain, limelight, acceptableAngle, () -> 0.0, near);
    manualControl = false;
  }

  public DriveWithLimelight(Drivetrain drivetrain, Limelight limelight, DoubleSupplier joystickInput) {
    this(drivetrain, limelight, DEFAULT_MAXANGLE, joystickInput, true);
  }
  public DriveWithLimelight(Drivetrain drivetrain, Limelight limelight, double acceptableAngle, DoubleSupplier joystickInput, boolean near) {
    lime = limelight;
    dt = drivetrain;
    this.errorToleranceAngle = acceptableAngle;
    this.joystickInput = joystickInput;
    if (near) {
      P = P_NEAR;
      I = I_NEAR;
    } else {
      P = P_FAR;
      I = I_FAR;
    }
    manualControl = true;

    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pid = new PIDController(P, I, D);
    lime.enableLimelight();

    pid.setTolerance(errorToleranceAngle);
    pid.setIntegratorRange(-MAX_INTEGRAL, MAX_INTEGRAL);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    limeAngle = lime.getPositionX();

    if (Math.abs(limeAngle) < errorToleranceAngle) {
      ++withinThresholdLoops;
    }
    else {
      withinThresholdLoops = 0;
    }

    if (limeAngle < -errorToleranceAngle) {
      driveLimeTurnSpeed = -(pid.calculate(limeAngle) + MINIMUM_COMMAND);
    }
    else if (limeAngle > errorToleranceAngle) {
      driveLimeTurnSpeed = -(pid.calculate(limeAngle) - MINIMUM_COMMAND);
    }
    else {
      driveLimeTurnSpeed = 0.0;
    }

    if (withinThresholdLoops < acceptableLoops) {
      dt.arcadeDrive(joystickInput.getAsDouble(), driveLimeTurnSpeed);
    } else if (manualControl) {
      dt.arcadeDrive(joystickInput.getAsDouble(), 0.0);  // still drive fwd & back if you're within threshold
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (manualControl) {
      lime.pauseLimelight();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (withinThresholdLoops >= acceptableLoops && !manualControl); // command does not need to finish if bound to a button
  }
}
