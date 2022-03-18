// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands.auto;

import org.team2168.commands.FireBalls;
import org.team2168.commands.QueueBallsForShotNoStop;
import org.team2168.commands.Sleep;
import org.team2168.commands.drivetrain.DriveWithLimelight;
import org.team2168.commands.drivetrain.DriveXDistance;
import org.team2168.commands.drivetrain.TurnXDegrees;
import org.team2168.commands.intakeraiseandlower.IntakeLower;
import org.team2168.commands.intakeraiseandlower.IntakeRaise;
import org.team2168.commands.shooter.WaitForShooterAtSpeed;
import org.team2168.commands.shootingpositions.auto.AutoTarmacLine;
import org.team2168.subsystems.ColorSensor;
import org.team2168.subsystems.Drivetrain;
import org.team2168.subsystems.Hood;
import org.team2168.subsystems.Hopper;
import org.team2168.subsystems.Indexer;
import org.team2168.subsystems.IntakeRaiseAndLower;
import org.team2168.subsystems.IntakeRoller;
import org.team2168.subsystems.Limelight;
import org.team2168.subsystems.Pooper;
import org.team2168.subsystems.Shooter;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TerminalToTarmac extends SequentialCommandGroup {
  /** Creates a new TerminalToTarmac. */
  public TerminalToTarmac(
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
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new IntakeLower(intakeRaiseAndLower),
      new QueueBallsForShotNoStop(hopper, indexer, pooper, colorSensor, intakeRoller).withTimeout(2.0),
      new IntakeRaise(intakeRaiseAndLower),
      new DriveXDistance(drivetrain, Units.metersToInches(6.0)),
      new TurnXDegrees(drivetrain, -92.0),
      new AutoTarmacLine(hood, shooter, lime),
      new DriveWithLimelight(drivetrain, lime),
      new WaitForShooterAtSpeed(shooter),
      new FireBalls(shooter, indexer, hopper),
      new WaitForShooterAtSpeed(shooter),
      new FireBalls(shooter, indexer, hopper)
    );
  }
}
