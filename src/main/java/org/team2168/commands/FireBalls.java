// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import org.team2168.Constants;
import org.team2168.commands.hopper.DriveHopperUntilNoBall;
import org.team2168.commands.hopper.DriveHopperWithPercentOutput;
import org.team2168.commands.indexer.DriveIndexerUntilBall;
import org.team2168.commands.indexer.DriveIndexerUntilNoBall;
import org.team2168.commands.shooter.WaitForShooterAtSpeed;
import org.team2168.subsystems.Hopper;
import org.team2168.subsystems.Indexer;
import org.team2168.subsystems.Shooter;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FireBalls extends SequentialCommandGroup {
  /** Creates a new FireBalls. */

  public FireBalls(Shooter shooter, Indexer indexer, Hopper hopper) {
      
    addCommands(
      new ConditionalCommand(
          new WaitForShooterAtSpeed(shooter), new PrintCommand("No Wait"), shooter::shouldWaitForShooterAtSpeed),
      race( // Make sure the ball is queued up near the top before starting sequence 
        new DriveHopperWithPercentOutput(hopper, () -> Constants.MotorSpeeds.HOPPER_SPEED),
        new DriveIndexerUntilBall(indexer, () -> Constants.MotorSpeeds.INTAKE_SPEED)
      ),
      race( // Run indexer until the ball is gone
        new DriveHopperWithPercentOutput(hopper, () -> Constants.MotorSpeeds.HOPPER_SPEED),
        new DriveIndexerUntilNoBall(indexer, Constants.MotorSpeeds.INTAKE_SPEED)
      )
    );
  }

}
