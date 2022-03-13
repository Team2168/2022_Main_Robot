package org.team2168.commands.auto;


import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.team2168.commands.FireBalls;
import org.team2168.commands.QueueBallsForShotNoStop;
import org.team2168.commands.drivetrain.DriveWithLimelight;
import org.team2168.commands.drivetrain.DriveXDistance;
import org.team2168.commands.drivetrain.TurnXAngle;
import org.team2168.commands.indexer.DriveIndexer;
import org.team2168.commands.intakeraiseandlower.IntakeLower;
import org.team2168.commands.shooter.WaitForShooterAtSpeed;
import org.team2168.commands.shootingpositions.auto.AutoTarmacLine;
import org.team2168.subsystems.*;

public class ThreeBall extends SequentialCommandGroup {
    public ThreeBall(Drivetrain drivetrain,
                             IntakeRaiseAndLower intakeRaiseAndLower,
                             IntakeRoller intakeRoller,
                             Hopper hopper,
                             Indexer indexer,
                             Hood hood,
                             Shooter shooter,
                             Pooper pooper,
                             ColorSensor colorSensor,
                             Limelight lime) {
        addCommands(
                new AutoTarmacLine(hood, shooter, lime).withTimeout(0.2),
                new IntakeLower(intakeRaiseAndLower),
                race(
                        new QueueBallsForShotNoStop(hopper, indexer, pooper, colorSensor, intakeRoller),
                        new DriveXDistance(drivetrain, -3.28084 * 12)
                ),
                new DriveIndexer(indexer, () -> 0.0).withTimeout(0.1),

                new InstantCommand(() -> System.out.println("reving up shooter!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")),
                new WaitForShooterAtSpeed(shooter),
                new InstantCommand(() -> System.out.println("shooter done!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!11")),
                new DriveWithLimelight(drivetrain, lime, 1.5, true),
                new InstantCommand(() -> System.out.println("Limelight done!!!!!!!!!!!!!!!!!!!!!!!!!!!")),
                new FireBalls(shooter, indexer, hopper),
                new WaitForShooterAtSpeed(shooter),
                new FireBalls(shooter, indexer, hopper),
                new TurnXAngle(drivetrain, 135),
                new IntakeLower(intakeRaiseAndLower),
                race(
                        new QueueBallsForShotNoStop(hopper, indexer, pooper, colorSensor, intakeRoller),
                        new DriveXDistance(drivetrain, 9*12)
                ),
                new DriveIndexer(indexer, () -> 0.0).withTimeout(0.1),

                new InstantCommand(() -> System.out.println("reving up shooter!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")),
                new WaitForShooterAtSpeed(shooter),
                new InstantCommand(() -> System.out.println("shooter done!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!11")),
                new DriveWithLimelight(drivetrain, lime, 1.5, true),
                new InstantCommand(() -> System.out.println("Limelight done!!!!!!!!!!!!!!!!!!!!!!!!!!!")),
                new FireBalls(shooter, indexer, hopper)
        );
}