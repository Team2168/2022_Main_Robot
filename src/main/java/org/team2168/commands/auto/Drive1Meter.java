// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands.auto;

import org.team2168.subsystems.Drivetrain;
import org.team2168.utils.PathUtil;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;


public class Drive1Meter extends SequentialCommandGroup {
  /** Creates a new Drive1Meter. */
  public Drive1Meter(Drivetrain drivetrain) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      PathUtil.getPathPlannerCommand("Drive1Meter", drivetrain)
    );
  }
}
