// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands.turret;

import org.team2168.subsystems.Limelight;
import org.team2168.subsystems.Turret;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FindTargetAndCenterTurret extends SequentialCommandGroup {
  /** Creates a new FindTargetAndCenterTurret. */
  private Turret turret;
  private Limelight limelight;

  /**
   * Drives the turret to find a target and lines it up
   * @param turret the turret instance
   * @param limelight the limelight instance
   */
  public FindTargetAndCenterTurret(Turret turret, Limelight limelight) {
    this.turret = turret;
    this.limelight = limelight;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new FindTargetWithTurret(turret, limelight),
      new DriveTurretWithLimelight(turret, limelight)
    );
  }
}
