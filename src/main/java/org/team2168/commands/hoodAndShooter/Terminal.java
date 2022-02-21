// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands.hoodAndShooter;

import org.team2168.commands.hood.HoodToAngle;
import org.team2168.commands.shooter.SetSpeed;
import org.team2168.subsystems.Hood;
import org.team2168.subsystems.Shooter;
import org.team2168.subsystems.Hood.HoodPosition;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Terminal extends ParallelCommandGroup {
  /** Creates a new Terminal. */
  private Hood hood;
  private Shooter shooter;

  public Terminal(Hood h, Shooter s) {
    hood = h;
    shooter = s;

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new HoodToAngle(hood, HoodPosition.TERMINAL.position_degrees),
      new SetSpeed(shooter, 2500.0)
    );
  }
}