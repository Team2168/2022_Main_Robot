// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands.auto.pathplanner;

import org.team2168.subsystems.Drivetrain;
import org.team2168.utils.PathUtil;
import org.team2168.utils.PathUtil.InitialPathState;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class TwoballTopToTerm extends SequentialCommandGroup {
  /** Creates a new TwoballTopToTerm. */
  public TwoballTopToTerm(Drivetrain drivetrain) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    Paths paths = Paths.getInstance();
    addCommands(
      // new ExtendIntake(intake),
      new ParallelRaceGroup(
              // new DriveIntake(intake, () -> 0.5)
              new SequentialCommandGroup(
                PathUtil.getPathCommand(paths.path_4BALL_0, drivetrain, InitialPathState.DISCARDHEADING),
                // new DriveShooter(shooter, () -> 0.5).withTimeout(3),
                PathUtil.getPathCommand(paths.path_2BALL_1, drivetrain, InitialPathState.PRESERVEODOMETRY)
              )
      )
    );
  }
}
