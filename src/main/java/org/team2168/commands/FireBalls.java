// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands;

import org.team2168.commands.indexer.DriveUntilBall;
import org.team2168.commands.indexer.DriveUntilNoBall;
import org.team2168.commands.shooter.WaitForShooterAtSpeed;
import org.team2168.subsystems.Indexer;
import org.team2168.subsystems.Shooter;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FireBalls extends SequentialCommandGroup {
  /** Creates a new FireBalls. */
  private Shooter shooter;
  private Indexer indexer;
  private double indexerSpeed = 0.8;

  public FireBalls(Shooter shooter, Indexer indexer) {
    this.shooter = shooter;
    this.indexer = indexer;

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    addCommands(
      new WaitForShooterAtSpeed(shooter),
      new DriveUntilNoBall(indexer, indexerSpeed),
      new DriveUntilBall(indexer, indexerSpeed)
    );
  }
}
