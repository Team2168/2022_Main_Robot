package org.team2168.commands.auto.pathplanner;


import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import org.team2168.Constants;
import org.team2168.commands.FireBalls;
import org.team2168.commands.QueueBallsForShotNoStop;
import org.team2168.commands.RetractAndStopIntake;
import org.team2168.commands.drivetrain.DriveWithLimelight;
import org.team2168.commands.indexer.DriveIndexer;
import org.team2168.commands.intakeraiseandlower.IntakeLower;
import org.team2168.commands.intakeroller.SetIntakeSpeed;
import org.team2168.commands.limelight.WaitForLimelightInPosition;
import org.team2168.commands.shooter.WaitForShooterAtSpeed;
import org.team2168.commands.shootingpositions.Launchpad;
import org.team2168.commands.shootingpositions.TarmacLine;
import org.team2168.commands.shootingpositions.auto.AutoTarmacLine;
import org.team2168.commands.turret.DriveTurretWithLimelight;
import org.team2168.commands.turret.RotateTurret;
import org.team2168.commands.turret.StopTurret;
import org.team2168.subsystems.*;
import org.team2168.utils.PathUtil;

public class ThreeBall extends SequentialCommandGroup {
    public ThreeBall(
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
                race(
                        new DriveTurretWithLimelight(turret, lime),
                        sequence(
                                new AutoTarmacLine(hood, shooter, lime).withTimeout(0.2),
                                new IntakeLower(intakeRaiseAndLower),
                                race(  // run group until path ends
                                        new QueueBallsForShotNoStop(hopper, indexer, pooper, colorSensor, intakeRoller),
                                        PathUtil.getPathCommand(paths.path_4BALL_0, drivetrain, PathUtil.InitialPathState.DISCARDHEADING)
                                ),
                                // new RetractAndStopIntake(intakeRaiseAndLower, intakeRoller).withTimeout(0.1),
                                new DriveIndexer(indexer, () -> 0.0).withTimeout(0.1),

                                new InstantCommand(() -> System.out.println("reving up shooter!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")),
                                new WaitForShooterAtSpeed(shooter),
                                new InstantCommand(() -> System.out.println("shooter done!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!11")),
                                new WaitForLimelightInPosition(lime),
                                new InstantCommand(() -> System.out.println("Limelight done!!!!!!!!!!!!!!!!!!!!!!!!!!!")),
                                new FireBalls(shooter, indexer, hopper),
                                new WaitForShooterAtSpeed(shooter),
                                new FireBalls(shooter, indexer, hopper)
                        )),
                new RotateTurret(turret, 0.0).withTimeout(0.2), // zero turret after first set
                new StopTurret(turret),

                sequence(
                        new IntakeLower(intakeRaiseAndLower),
                        // new Launchpad(hood, shooter, lime),
                        new AutoTarmacLine(hood, shooter, lime).withTimeout(0.2),
                        race(
                                new QueueBallsForShotNoStop(hopper, indexer, pooper, colorSensor, intakeRoller),
                                sequence(
                                        PathUtil.getPathCommand(paths.path_3Ball_1, drivetrain, PathUtil.InitialPathState.PRESERVEODOMETRY)
                                        // PathUtil.getPathCommand(paths.path_3Ball_2, drivetrain, PathUtil.InitialPathState.PRESERVEODOMETRY),
                                        //new DriveWithLimelight(drivetrain, lime, 1.5, false)
                                )
                        ),

                        // new RetractAndStopIntake(intakeRaiseAndLower, intakeRoller).withTimeout(0.1),
                        parallel(
                                new DriveTurretWithLimelight(turret, lime),
                                sequence(

                                        new InstantCommand(() -> System.out.println("path finished!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")),
                                        race(
                                                new SetIntakeSpeed(intakeRoller, Constants.MotorSpeeds.INTAKE_SPEED),
                                                sequence(
                                                        new WaitForLimelightInPosition(lime),
                                                        new FireBalls(shooter, indexer, hopper)
                                                )
                                        ),

                                        new InstantCommand(() -> System.out.println("fired!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")),
                                        race(
                                                new SetIntakeSpeed(intakeRoller, Constants.MotorSpeeds.INTAKE_SPEED),
                                                new FireBalls(shooter, indexer, hopper)
                                        )
                                )
                        )
                )
        );
    }
}