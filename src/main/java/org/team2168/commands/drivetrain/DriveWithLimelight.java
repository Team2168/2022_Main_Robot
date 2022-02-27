// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands.drivetrain;

import org.team2168.OI;
import org.team2168.subsystems.Drivetrain;
import org.team2168.subsystems.Limelight;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;

public class DriveWithLimelight extends CommandBase implements Loggable {
  /** Creates a new DriveWithLimelight. */
  private Drivetrain dt;
  private Limelight lime;
  private OI oi;
  private PIDController pid;

  private double errorToleranceAngle = 0.5; // in degrees
  private double limeAngle;
  private int withinThresholdLoops = 0;
  private int acceptableLoops = 10;

  private static final double MINIMUM_COMMAND = 2.02/12;
  
  //limelight gains
  @Log(name = "P")
  private double P = 0.02;
  @Log(name = "I")
  private double I = 0.0;
  @Log(name = "D")
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
  private boolean useJoystick;

  //speed of drivetrain rotation
  private double driveLimeTurnSpeed;

  @Log(name = "Turn Speed")
  private double getLimeTurnSpeed() {
    return driveLimeTurnSpeed;
  }
  
  public DriveWithLimelight(Drivetrain drivetrain, Limelight limelight) {
    addRequirements(limelight);
    lime = limelight;
    dt = drivetrain;
  }

  public DriveWithLimelight(Drivetrain drivetrain, Limelight limelight, boolean useJoystick) {
    addRequirements(limelight);
    lime = limelight;
    dt = drivetrain;
    this.useJoystick = useJoystick;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    oi = OI.getInstance();
    pid = new PIDController(P, I, D);
    if (!lime.isLimelightEnabled()) {
      lime.enableLimelight();
    }

    pid.setTolerance(errorToleranceAngle);
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
      if (!useJoystick) {
        dt.arcadeDrive(0.0, driveLimeTurnSpeed);
      }
      else if (useJoystick) {
        dt.arcadeDrive(oi.getGunStyleTrigger(), driveLimeTurnSpeed);
      }
    }
    
    System.out.println(driveLimeTurnSpeed);
    SmartDashboard.putNumber("driveLimeTurnSpeed", driveLimeTurnSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    lime.pauseLimelight();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() { 
    return (Math.abs(limeAngle) < errorToleranceAngle && !useJoystick); // command does not need to finish if bound to a button
  }
}
