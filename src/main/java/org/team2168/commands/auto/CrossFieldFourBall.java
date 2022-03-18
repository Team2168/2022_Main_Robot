// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands.auto;

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

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CrossFieldFourBall extends SequentialCommandGroup {
  /** Creates a new CrossFieldFourBall. */
  public CrossFieldFourBall(
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
      new SimpleOppositeSideToTerminal(drivetrain, intakeRaiseAndLower, intakeRoller, hopper, indexer, hood, shooter, pooper, colorSensor, lime),
      new TerminalToTarmac(drivetrain, intakeRaiseAndLower, intakeRoller, hopper, indexer, hood, shooter, pooper, colorSensor, lime)
    );
  }
}
