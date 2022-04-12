package org.team2168.commands.auto.pathplanner;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import org.team2168.commands.FireBalls;
import org.team2168.commands.QueueBallsForShotNoStop;
import org.team2168.commands.drivetrain.ArcadeDrive;
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
import org.team2168.commands.turret.RotateTurretNoFinish;
import org.team2168.subsystems.*;
import org.team2168.subsystems.Hood.HoodPosition;
import org.team2168.subsystems.Shooter.ShooterRPM;
import org.team2168.utils.PathUtil;
import org.team2168.utils.PathUtil.InitialPathState;

public class FiveBall extends SequentialCommandGroup {


        public FiveBall(
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
                Limelight lime) {
    
            Paths paths = Paths.getInstance();
    
            addCommands(
                    new RotateTurret(turret, 0.0).withTimeout(0.2),
                    new InstantCommand(() -> shooter.setWaitForShooterAtSpeed(false)),
                    race(
                            new DriveTurretWithLimelight(turret, lime),
                            sequence(
                                    //two ball auto
                                    //collects balls
                                    new AutoTarmacLine(hood, shooter, lime).withTimeout(0.2),
                                    new IntakeLower(intakeRaiseAndLower),
                                    race( // run group until path ends
                                            new QueueBallsForShotNoStop(hopper, indexer, pooper, colorSensor, intakeRoller),
                                            //first ball
                                            PathUtil.getPathCommand(paths.path_4BALL_0, drivetrain,
                                                    PathUtil.InitialPathState.DISCARDHEADING)),
                                    // new RetractAndStopIntake(intakeRaiseAndLower, intakeRoller).withTimeout(0.1),
                                    //stops subsystems and shoots
                                    parallel(
                                            new DriveHopperWithPercentOutput(hopper, () -> 0.0).withTimeout(0.1),
                                            new DriveIndexer(indexer, () -> 0.0).withTimeout(0.1),
                                            new SetIntakeSpeed(intakeRoller, 0.0).withTimeout(0.1),
                                            new InstantCommand(() -> drivetrain.tankDrive(0.0, 0.0))).withTimeout(0.1),
                                    parallel(
                                            new WaitForShooterAtSpeed(shooter, 5),
                                            new WaitForLimelightInPosition(lime)),
                                    new FireBalls(shooter, indexer, hopper),
                                    new FireBalls(shooter, indexer, hopper),
    
                                    //gets third ball and shoots it
                                    new IntakeLower(intakeRaiseAndLower),
                                    // new Launchpad(hood, shooter, lime),
                                    new SetShooterSpeed(shooter, ShooterRPM.AUTO_BALL3),
                                    race(
                                            new QueueBallsForShotNoStop(hopper, indexer, pooper, colorSensor, intakeRoller),
                                            sequence(
                                                    PathUtil.getPathCommand(paths.path_4BALL_1, drivetrain,
                                                            PathUtil.InitialPathState.PRESERVEODOMETRY))),
                                    parallel(
                                            new DriveHopperWithPercentOutput(hopper, () -> 0.0).withTimeout(0.1),
                                            new DriveIndexer(indexer, () -> 0.0).withTimeout(0.1),
                                            new SetIntakeSpeed(intakeRoller, 0.0).withTimeout(0.1),
                                            new InstantCommand(() -> drivetrain.tankDrive(0.0, 0.0))).withTimeout(0.1),
                                    parallel(
                                            new WaitForShooterAtSpeed(shooter, 10),
                                            new WaitForLimelightInPosition(lime)),
                                    new FireBalls(shooter, indexer, hopper))
                    ),
                    //gets fourth ball at terminal
                    race(
                        new RotateTurretNoFinish(turret, 0.0),
                        sequence(
                            new IntakeLower(intakeRaiseAndLower),
                            race(
                                    new QueueBallsForShotNoStop(hopper, indexer, pooper, colorSensor, intakeRoller),
                                    PathUtil.getPathCommand(paths.path_4BALL_2, drivetrain,
                                            PathUtil.InitialPathState.PRESERVEODOMETRY)
                            ),
                            // new RetractAndStopIntake(intakeRaiseAndLower, intakeRoller).withTimeout(0.1),
                            new InstantCommand(() -> System.out.println("path finished!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"))
    
                            // new SetIntakeSpeed(intakeRoller, Constants.MotorSpeeds.INTAKE_SPEED),
                        )
                    ),
                            
                    new SetShooterSpeed(shooter, ShooterRPM.AUTO_TARMAC_LINE),
                    new HoodToAngle(hood, HoodPosition.AUTO_TARMAC_LINE.position_degrees),               
    
                            race(
                                    new DriveTurretWithLimelight(turret, lime),
                                    new QueueBallsForShotNoStop(hopper, indexer, pooper, colorSensor, intakeRoller),                                                                                            
                                    //drives up to edge of tarmac
                                    PathUtil.getPathCommand(paths.path_4BALL_3, drivetrain, InitialPathState.PRESERVEODOMETRY)
                            ),
                                
                            parallel(
                                    new DriveTurretWithLimelight(turret, lime),
                                    sequence(
                                            parallel(
                                                    new WaitForShooterAtSpeed(shooter, 20),
                                                    new WaitForLimelightInPosition(lime)
                                            ),
                                            
                                            new ArcadeDrive(drivetrain, () -> 0.0, () -> 0.0).withTimeout(0.1),
                                            new FireBalls(shooter, indexer, hopper),
                                            new FireBalls(shooter, indexer, hopper)
                                    )
                            )
            );
        }
}