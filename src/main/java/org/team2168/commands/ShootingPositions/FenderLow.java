package org.team2168.commands.ShootingPositions;


import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import org.team2168.commands.hood.HoodToAngle;
import org.team2168.commands.shooter.SetShooterSpeed;
import org.team2168.subsystems.Hood;
import org.team2168.subsystems.Shooter;

public class FenderLow extends ParallelCommandGroup {
    public FenderLow(Hood hood, Shooter shooter) {
        addCommands(
                new HoodToAngle(hood, Hood.HoodPosition.FENDER_LOW.position_degrees),
                new SetShooterSpeed(shooter, Shooter.ShooterRPM.FENDER_LOW.rpm)
        );
    }
}