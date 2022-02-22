package org.team2168.commands.ShootingPositions;


import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import org.team2168.commands.hood.DriveHoodToAngle;
import org.team2168.commands.shooter.DriveShooterToSpeed;
import org.team2168.subsystems.Hood;
import org.team2168.subsystems.Shooter;

public class FenderLowShooterPos extends ParallelCommandGroup {
    public FenderLowShooterPos(Hood hood, Shooter shooter) {
        addCommands(
                new DriveHoodToAngle(hood, Hood.HoodPosition.FENDER_LOW.position_degrees),
                new DriveShooterToSpeed(shooter, Shooter.ShooterRPM.FENDER_LOW.rpm)
        );
    }
}