// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class TurnInPlace extends CommandBase {

 public static org.team2168.subsystems.Drivetrain turnInPlace;


 private double targetPos = 0.0;
 private double targetAngle;

 private static double DEFAULT_TOLERANCE_ERROR = 1.0;
 private double errorTolerancePosition = 0.5; 
 private double errorToleranceAngle;  
 private double loopsToSettle = 5;
 private int withinThresholdLoops = 0;

 public TurnInPlace(double setPoint){
   this(turnInPlace, setPoint, DEFAULT_TOLERANCE_ERROR);
 }


  public TurnInPlace(org.team2168.subsystems.Drivetrain turnInPlace, double targetAngle, double errorToleranceAngle) {
    this.turnInPlace = turnInPlace;
    this.targetAngle = targetAngle;
    this.errorToleranceAngle = errorToleranceAngle;
    addRequirements(turnInPlace);
     
  }
    


  // Called when the command is initially scheduled.
  @Override

  public void initialize() {
    withinThresholdLoops = 0;
    turnInPlace.zeroHeading();
    turnInPlace.resetEncoders();
    turnInPlace.switchGains(false);
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    turnInPlace.setPointPosition(targetPos, targetAngle);
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
   turnInPlace.tankDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
