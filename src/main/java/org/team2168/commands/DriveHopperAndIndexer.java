// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands;

import java.util.function.DoubleSupplier;

import org.team2168.Constants;
import org.team2168.commands.hopper.DriveHopperWithPercentOutput;
import org.team2168.commands.indexer.DriveIndexer;
import org.team2168.subsystems.Hopper;
import org.team2168.subsystems.Indexer;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveHopperAndIndexer extends ParallelCommandGroup {

  /**
   * Drives both hopper and indexer at speed set in RobotContainer
   * 
   * @param hopper Hopper subsystem
   * @param indexer Indexer subsystem
   * @param speed Doublesupplier for speed value
   */
  public DriveHopperAndIndexer(Hopper hopper, Indexer indexer, double hopperSpeed, double indexerSpeed) {

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new DriveHopperWithPercentOutput(hopper, () -> hopperSpeed),
      new DriveIndexer(indexer, () -> indexerSpeed)
    );
  }
}
