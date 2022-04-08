// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands.auto.pathplanner;

import org.team2168.commands.FireBalls;
import org.team2168.commands.QueueBallsForShotNoStop;
import org.team2168.commands.drivetrain.ArcadeDrive;
import org.team2168.commands.hood.HoodToAngle;
import org.team2168.commands.hopper.DriveHopperWithPercentOutput;
import org.team2168.commands.indexer.DriveIndexer;
import org.team2168.commands.intakeraiseandlower.IntakeLower;
import org.team2168.commands.intakeroller.SetIntakeSpeed;
import org.team2168.commands.limelight.SetPipeline;
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
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SimpleFourBall extends SequentialCommandGroup {
  public SimpleFourBall(Drivetrain drivetrain, 
                        IntakeRaiseAndLower intakeRaiseAndLower, 
                        IntakeRoller intakeRoller, 
                        Hopper hopper, 
                        Indexer indexer, 
                        Turret turret,
                        Hood hood, 
                        Shooter shooter,
                        Limelight limelight,
                        Pooper pooper,
                        ColorSensor colorSensor) {

    Paths paths = Paths.getInstance();

    addCommands(
      new RotateTurret(turret, 0.0).withTimeout(0.2),
      new InstantCommand(() -> shooter.setWaitForShooterAtSpeed(false)),

      race(
        new DriveTurretWithLimelight(turret, limelight),
        sequence(
          new HoodToAngle(hood, HoodPosition.AUTO_TARMAC_LINE.position_degrees),
          new SetShooterSpeed(shooter, ShooterRPM.AUTO_BALL3),
          new SetPipeline(limelight, Limelight.PIPELINE_TARMAC_LINE),
          new IntakeLower(intakeRaiseAndLower),
          race(
            new QueueBallsForShotNoStop(hopper, indexer, pooper, colorSensor, intakeRoller),
            PathUtil.getPathCommand(paths.path_simple_4_ball_1, drivetrain, InitialPathState.DISCARDHEADING)),
          parallel(
            new DriveHopperWithPercentOutput(hopper, () -> 0.0),
            new DriveIndexer(indexer, () -> 0.0),
            new SetIntakeSpeed(intakeRoller, 0.0),
            new ArcadeDrive(drivetrain, () -> 0.0, () -> 0.0)).withTimeout(0.1),
          parallel(
            new WaitForShooterAtSpeed(shooter),
            new WaitForLimelightInPosition(limelight)),
          new FireBalls(shooter, indexer, hopper),
          new FireBalls(shooter, indexer, hopper), //shoots two balls

          parallel(
            new HoodToAngle(hood, HoodPosition.ZERO.position_degrees),
            new SetShooterSpeed(shooter, ShooterRPM.STOP)).withTimeout(0.1),
          race(
            new QueueBallsForShotNoStop(hopper, indexer, pooper, colorSensor, intakeRoller),
            PathUtil.getPathCommand(paths.path_simple_4_ball_2, drivetrain, InitialPathState.PRESERVEODOMETRY)),
          new WaitUntilCommand(hopper::isBallPresent).withTimeout(2.0), //2 seconds for a 2nd ball to be rolled in
          new AutoTarmacLine(hood, shooter, limelight),
          PathUtil.getPathCommand(paths.path_simple_4_ball_3, drivetrain, InitialPathState.PRESERVEODOMETRY),
          parallel(
            new DriveHopperWithPercentOutput(hopper, () -> 0.0),
            new DriveIndexer(indexer, () -> 0.0),
            new SetIntakeSpeed(intakeRoller, 0.0),
            new ArcadeDrive(drivetrain, () -> 0.0, () -> 0.0)).withTimeout(0.1),
          parallel(
            new WaitForShooterAtSpeed(shooter),
            new WaitForLimelightInPosition(limelight)),
          new FireBalls(shooter, indexer, hopper),
          new FireBalls(shooter, indexer, hopper)
            
          ))                
    );
  }
}
