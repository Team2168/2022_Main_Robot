// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands.auto;

import org.team2168.subsystems.Drivetrain;
import org.team2168.utils.PathUtil;
import org.team2168.utils.PathUtil.InitialPathState;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class MultipartNonZeroVel extends SequentialCommandGroup {
  /** Creates a new MultipartNonZeroVel. */
  public MultipartNonZeroVel(Drivetrain drivetrain) {
    Paths paths = Paths.getInstance();
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      PathUtil.getPathCommand(paths.path_canweturn, drivetrain, InitialPathState.DISCARDHEADING),
      PathUtil.getPathCommand(paths.path_wecanturn, drivetrain, InitialPathState.PRESERVEHEADING)
    );
  }
}
