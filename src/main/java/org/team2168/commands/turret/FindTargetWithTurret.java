// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands.turret;

import org.team2168.subsystems.Limelight;
import org.team2168.subsystems.Turret;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class FindTargetWithTurret extends CommandBase {
  /** Creates a new FindTargetWithTurret. */
  private Turret turret;
  private Limelight limelight;

  private double startingPos;
  private boolean rotateToRight;
  private boolean reachedLimit;
  private boolean targetFound = false;

  private long startTime;

  public FindTargetWithTurret(Turret turret, Limelight limelight) {
    this.turret = turret;
    this.limelight = limelight;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    limelight.enableLimelight();

    startingPos = turret.getEncoderPosition();

    if (startingPos <= 0)
      rotateToRight = false;
    else 
      rotateToRight = true;

    startTime = System.currentTimeMillis();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    targetFound = limelight.hasTarget();

    //rotates while the target isn't in view or it hasn't turned to the closest soft limit
    while (targetFound == false || reachedLimit == false) {
      // rotates the turret +/- 1 degree and checks if there is a target
      if (rotateToRight == false)
        //turns until reaches soft limit
        turret.setRotationDegrees(-1);
      else 
        turret.setRotationDegrees(1);

      targetFound = limelight.hasTarget();
      if (turret.atSoftLimit())
        reachedLimit = true;
    }

    while (targetFound == false && reachedLimit == true) {
      // goes the opposite direction it was going in
      if (rotateToRight == false) 
        turret.setRotationDegrees(1);
      else 
        turret.setRotationDegrees(-1);
      targetFound = limelight.hasTarget();
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    /* if the target is found, the turret stops moving
       if the target wasn't found, the turret goes to zero */
    if (targetFound)
      turret.setVelocity(0.0);
    else
      turret.zeroTurret();
    limelight.pauseLimelight();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //if the target is found or it has been longer than 10 seconds
    return (targetFound || (System.currentTimeMillis() >= startTime + 10 * 1000)); 
  }
}
