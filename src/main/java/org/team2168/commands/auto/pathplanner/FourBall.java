// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands.auto.pathplanner;

import org.team2168.commands.FireBalls;
import org.team2168.commands.QueueBallsForShotNoStop;
import org.team2168.commands.drivetrain.ArcadeDrive;
import org.team2168.commands.hopper.DriveHopperWithPercentOutput;
import org.team2168.commands.indexer.DriveIndexer;
import org.team2168.commands.intakeraiseandlower.IntakeLower;
import org.team2168.commands.intakeroller.SetIntakeSpeed;
import org.team2168.commands.limelight.WaitForLimelightInPosition;
import org.team2168.commands.shooter.WaitForShooterAtSpeed;
import org.team2168.commands.shootingpositions.ShootBasedOnDistance;
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
import org.team2168.utils.PathUtil.InitialPathState;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FourBall extends SequentialCommandGroup {


        public FourBall(
                Drivetrain drivetrain,
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
                            // constantly tracks the hub
                            new DriveTurretWithLimelight(turret, limelight),
                            new ShootBasedOnDistance(shooter, hood, limelight),

                    sequence(
                            new IntakeLower(intakeRaiseAndLower),    
                            //picks up ball
                            race(
                                    new QueueBallsForShotNoStop(hopper, indexer, pooper, colorSensor, intakeRoller),
                                    PathUtil.getPathCommand(paths.path_simple_4_ball_1, drivetrain,
                                            InitialPathState.DISCARDHEADING)),
                            parallel(
                                    new DriveHopperWithPercentOutput(hopper,() -> 0.0),
                                    new DriveIndexer(indexer, () -> 0.0),
                                    new SetIntakeSpeed(intakeRoller, 0.0),
                                    new ArcadeDrive(drivetrain, () -> 0.0,() -> 0.0)).withTimeout(0.1),
                            parallel(
                                    new WaitForShooterAtSpeed(shooter),
                                    new WaitForLimelightInPosition(limelight)),
                            new FireBalls(shooter, indexer, hopper),
                            new WaitForShooterAtSpeed(shooter),
                            new FireBalls(shooter, indexer, hopper),

                            //goes to terminal
                            race(
                                    new QueueBallsForShotNoStop(hopper, indexer, pooper, colorSensor, intakeRoller),
                                    PathUtil.getPathCommand(paths.path_simple_4_ball_2,drivetrain,
                                        InitialPathState.PRESERVEODOMETRY)),
                            //waits for two balls
                            race(
                                    new QueueBallsForShotNoStop(hopper, indexer, pooper, colorSensor, intakeRoller),
                                    parallel(
                                            new WaitUntilCommand(indexer::isBallPresent)),
                                            new WaitUntilCommand(hopper::isBallPresent).withTimeout(3.0)),
                            //drives up to tarmac
                            PathUtil.getPathCommand(paths.path_simple_4_ball_3, drivetrain,
                                    InitialPathState.PRESERVEODOMETRY),
                            parallel(
                                    new DriveHopperWithPercentOutput(hopper, () -> 0.0),
                                    new DriveIndexer(indexer, () -> 0.0),
                                    new SetIntakeSpeed(intakeRoller, 0.0),
                                    new ArcadeDrive(drivetrain, () -> 0.0, () -> 0.0)).withTimeout(0.1),
                            parallel(
                                    new WaitForShooterAtSpeed(shooter),
                                    new WaitForLimelightInPosition(limelight)),

                            new FireBalls(shooter, indexer, hopper),
                            new WaitForShooterAtSpeed(shooter),
                            new FireBalls(shooter, indexer, hopper)
                ))
            );
        }
}