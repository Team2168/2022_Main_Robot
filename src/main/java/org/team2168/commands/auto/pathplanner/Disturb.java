// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands.auto.pathplanner;

import java.nio.file.Path;

import org.team2168.commands.FireBalls;
import org.team2168.commands.QueueBallsForShotNoStop;
import org.team2168.commands.drivetrain.TankDrive;
import org.team2168.commands.hood.HoodToAngle;
import org.team2168.commands.hopper.DriveHopperWithPercentOutput;
import org.team2168.commands.indexer.DriveIndexer;
import org.team2168.commands.intakeraiseandlower.IntakeLower;
import org.team2168.commands.intakeroller.SetIntakeSpeed;
import org.team2168.commands.limelight.WaitForLimelightInPosition;
import org.team2168.commands.shooter.SetShooterSpeed;
import org.team2168.commands.shooter.WaitForShooterAtSpeed;
import org.team2168.commands.shootingpositions.auto.AutoTarmacLine;
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
import org.team2168.subsystems.Hood.HoodPosition;
import org.team2168.subsystems.Shooter.ShooterRPM;
import org.team2168.utils.PathUtil;
import org.team2168.utils.PathUtil.InitialPathState;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Disturb extends SequentialCommandGroup {
  /** Creates a new Disturb. */
  public Disturb(
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
      Limelight limelight) {

    Paths paths = Paths.getInstance();

    addCommands(
        new RotateTurret(turret, 0.0).withTimeout(0.1),
        new InstantCommand(() -> shooter.setWaitForShooterAtSpeed(false)),
        // Collects ball and shoots it and preloaded ball
          race(
              new DriveTurretWithLimelight(turret, limelight),
              sequence(
                  new AutoTarmacLine(hood, shooter, limelight),
                  new IntakeLower(intakeRaiseAndLower),
                  race(
                      new QueueBallsForShotNoStop(hopper, indexer, pooper, colorSensor, intakeRoller),
                      PathUtil.getPathCommand(paths.path_Disturb_1, drivetrain, PathUtil.InitialPathState.DISCARDHEADING)),
                  // Hopper, indexer, intake, and drivtrain have 0.1 seconds to stop
                  parallel(
                      new DriveHopperWithPercentOutput(hopper, () -> 0.0),
                      new DriveIndexer(indexer, () -> 0.0),
                      new SetIntakeSpeed(intakeRoller, 0.0),
                      new TankDrive(drivetrain, () -> 0.0, () -> 0.0)).withTimeout(0.1),
                  parallel(
                      new WaitForShooterAtSpeed(shooter),
                      new WaitForLimelightInPosition(limelight)),

                  new FireBalls(shooter, indexer, hopper),
                  new FireBalls(shooter, indexer, hopper),

                  parallel(
                      new SetShooterSpeed(shooter, ShooterRPM.STOP),
                      new HoodToAngle(hood, HoodPosition.ZERO.position_degrees),
                      new RotateTurret(turret, 0.0),
                      new DriveHopperWithPercentOutput(hopper, () -> 0.0),
                      new DriveIndexer(indexer, () -> 0.0)).withTimeout(0.1))),

        // Collects oppo. ball and poops it in hangar pt 1
        sequence(
          PathUtil.getPathCommand(paths.path_Disturb_2, drivetrain, InitialPathState.PRESERVEODOMETRY),
          race(
            
          )
        ),
        // Collects oppo. ball and poops it in hangar pt 2
        sequence());
  }
}
