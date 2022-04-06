// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands;

import org.team2168.commands.limelight.WaitForLimelightInPosition;
import org.team2168.commands.shooter.WaitForShooterAtSpeed;
import org.team2168.subsystems.Limelight;
import org.team2168.subsystems.Shooter;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;



public class WaitUntilFireBalls extends ParallelCommandGroup {
 
  public WaitUntilFireBalls(Shooter shooter, Limelight limelight) {
    
    addCommands(
      new WaitForShooterAtSpeed(shooter),
      new WaitForLimelightInPosition(limelight)
    );
  }
}
