// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands;

import org.team2168.commands.drivetrain.ArcadeDrive;
import org.team2168.commands.hopper.DriveHopperWithPercentOutput;
import org.team2168.commands.indexer.DriveIndexer;
import org.team2168.commands.intakeroller.SetIntakeSpeed;
import org.team2168.subsystems.Drivetrain;
import org.team2168.subsystems.Hopper;
import org.team2168.subsystems.Indexer;
import org.team2168.subsystems.IntakeRoller;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class StopMechanisms extends ParallelCommandGroup {

  public StopMechanisms(
      Hopper hopper,
      Indexer indexer,
      IntakeRoller intakeRoller,
      Drivetrain drivetrain) {

    addCommands(
        new DriveHopperWithPercentOutput(hopper, () -> 0.0).withTimeout(0.1),
        new DriveIndexer(indexer, () -> 0.0).withTimeout(0.1),
        new SetIntakeSpeed(intakeRoller, 0.0).withTimeout(0.1),
        new ArcadeDrive(drivetrain, () -> 0.0, () -> 0.0));
  }

  public StopMechanisms(
      Hopper hopper,
      Indexer indexer,
      IntakeRoller intakeRoller) 
      {
    addCommands(
        new DriveHopperWithPercentOutput(hopper, () -> 0.0).withTimeout(0.1),
        new DriveIndexer(indexer, () -> 0.0).withTimeout(0.1),
        new SetIntakeSpeed(intakeRoller, 0.0).withTimeout(0.1));
       }
}
