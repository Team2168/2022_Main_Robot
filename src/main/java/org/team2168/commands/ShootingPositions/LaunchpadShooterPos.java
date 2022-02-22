package org.team2168.commands.ShootingPositions;


import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import org.team2168.commands.hood.DriveHoodToAngle;
import org.team2168.commands.shooter.DriveShooterToSpeed;
import org.team2168.subsystems.Hood;
import org.team2168.subsystems.Shooter;

public class LaunchpadShooterPos extends ParallelCommandGroup {
    public LaunchpadShooterPos(Hood hood, Shooter shooter) {
        addCommands(
                new DriveHoodToAngle(hood, Hood.HoodPosition.LAUNCHPAD.position_degrees),
                new DriveShooterToSpeed(shooter, Shooter.ShooterRPM.LAUNCHPAD.rpm)
        );
    }
}