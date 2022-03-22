// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands.auto;

import org.team2168.commands.FireBalls;
import org.team2168.commands.QueueBallsForShotNoStop;
import org.team2168.commands.drivetrain.DriveWithLimelight;
import org.team2168.commands.drivetrain.DriveXDistance;
import org.team2168.commands.indexer.DriveIndexer;
import org.team2168.commands.intakeraiseandlower.IntakeLower;
import org.team2168.commands.shooter.WaitForShooterAtSpeed;
import org.team2168.commands.shootingpositions.auto.AutoTarmacLine;
import org.team2168.subsystems.*;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SimpleTwoBall extends SequentialCommandGroup {
  /** Creates a new TwoBall. */
  public SimpleTwoBall(
            Drivetrain drivetrain,
            IntakeRaiseAndLower intakeRaiseAndLower,
            IntakeRoller intakeRoller,
            Hopper hopper,
            Indexer indexer,
            Hood hood,
            Shooter shooter,
            Pooper pooper,
            ColorSensor colorSensor,
            Limelight lime) {
    addCommands(
      new AutoTarmacLine(hood, shooter, lime).withTimeout(0.2),
      new IntakeLower(intakeRaiseAndLower),
      race (  // run group until path ends
              new QueueBallsForShotNoStop(hopper, indexer, pooper, colorSensor, intakeRoller),
              new DriveXDistance(drivetrain, -42.0) //Drive backwards to get first ball
      ),
      // new RetractAndStopIntake(intakeRaiseAndLower, intakeRoller).withTimeout(0.1),
      new DriveIndexer(indexer, () -> 0.0).withTimeout(0.1),

      new InstantCommand(() -> System.out.println("reving up shooter!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")),
      new WaitForShooterAtSpeed(shooter),
      new InstantCommand(() -> System.out.println("shooter done!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!11")),
      new DriveWithLimelight(drivetrain, lime, 1.5, true),
      new InstantCommand(() -> System.out.println("Limelight done!!!!!!!!!!!!!!!!!!!!!!!!!!!")),
      new FireBalls(shooter, indexer, hopper),
      new WaitForShooterAtSpeed(shooter),
      new FireBalls(shooter, indexer, hopper)
    );
  }
}
