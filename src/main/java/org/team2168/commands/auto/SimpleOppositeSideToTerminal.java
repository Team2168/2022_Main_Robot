// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands.auto;

import org.team2168.commands.drivetrain.DriveXDistance;
import org.team2168.commands.drivetrain.TurnXDegrees;
import org.team2168.subsystems.*;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SimpleOppositeSideToTerminal extends SequentialCommandGroup {
  /** Creates a new SimpleOppositeSideToTerminal. */
  public SimpleOppositeSideToTerminal(
    Drivetrain drivetrain,
    IntakeRaiseAndLower intakeRaiseAndLower,
    IntakeRoller intakeRoller,
    Hopper hopper,
    Indexer indexer,
    Hood hood,
    Shooter shooter,
    Turret turret,
    Pooper pooper,
    ColorSensor colorSensor,
    Limelight lime) {
    addCommands(
      new SimpleTwoBall(drivetrain, intakeRaiseAndLower, intakeRoller, hopper, indexer, hood, shooter, turret, pooper, colorSensor, lime),

      new TurnXDegrees(drivetrain, 96.0), //Rotate butt toward Terminal
      new DriveXDistance(drivetrain, Units.metersToInches(-6.0)) // drive back towards Terminal
    );
  }
}
