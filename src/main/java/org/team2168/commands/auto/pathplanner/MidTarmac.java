// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands.auto.pathplanner;

import org.team2168.commands.QueueBallsForShotNoStop;
import org.team2168.commands.intakeraiseandlower.IntakeLower;
import org.team2168.commands.turret.DriveTurretWithLimelight;
import org.team2168.commands.turret.RotateTurret;
import org.team2168.subsystems.ColorSensor;
import org.team2168.subsystems.Drivetrain;
import org.team2168.subsystems.Hood;
import org.team2168.subsystems.Hopper;
import org.team2168.subsystems.Indexer;
import org.team2168.subsystems.IntakeRaiseAndLower;
import org.team2168.subsystems.IntakeRoller;
import org.team2168.subsystems.Limelight;
import org.team2168.subsystems.Pooper;
import org.team2168.subsystems.Shooter;
import org.team2168.subsystems.Turret;
import org.team2168.utils.PathUtil;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class MidTarmac extends SequentialCommandGroup {
  /** Creates a new MidTarmac. */
  public MidTarmac(
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
            Limelight lime
  ) {
    Paths paths = Paths.getInstance();
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new RotateTurret(turret, 0.0).withTimeout(0.5),
      new InstantCommand(() -> System.out.println("done turning turret")),
      parallel(
              new DriveTurretWithLimelight(turret, lime),
              new InstantCommand(() -> System.out.println("beginning sequence")),
              sequence(
                new IntakeLower(intakeRaiseAndLower),
                race(
                  new QueueBallsForShotNoStop(hopper, indexer, pooper, colorSensor, intakeRoller),
                  PathUtil.getPathCommand(paths.path_MIDFIELD_0, drivetrain, PathUtil.InitialPathState.DISCARDHEADING)
                )
              )
      )
    );
  }
}
