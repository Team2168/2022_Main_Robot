// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands.auto;

import org.team2168.commands.FireBalls;
import org.team2168.commands.drivetrain.DriveXDistance;
import org.team2168.commands.shooter.WaitForShooterAtSpeed;
import org.team2168.subsystems.Drivetrain;
import org.team2168.subsystems.Hopper;
import org.team2168.subsystems.Indexer;
import org.team2168.subsystems.Shooter;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Backward2Ball extends SequentialCommandGroup {
  /** Creates a new Backward2Ball. */
  public Backward2Ball(Shooter shooter, Drivetrain drivetrain, Indexer indexer, Hopper hopper) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new DriveXDistance(drivetrain, -2),
      new WaitForShooterAtSpeed(shooter),
      new FireBalls(shooter, indexer, hopper),
      new WaitForShooterAtSpeed(shooter),
      new FireBalls(shooter, indexer, hopper)
    );
  }
}
