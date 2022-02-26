package org.team2168.commands.shootingpositions;


import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import org.team2168.commands.hood.HoodToAngle;
import org.team2168.commands.shooter.SetShooterSpeed;
import org.team2168.subsystems.Hood;
import org.team2168.subsystems.Shooter;

public class Terminal extends ParallelCommandGroup {
    public Terminal(Hood hood, Shooter shooter) {
        addCommands(
            new HoodToAngle(hood, Hood.HoodPosition.TERMINAL.position_degrees),
                new SetShooterSpeed(shooter, Shooter.ShooterRPM.TERMINAL.rpm)
        );
    }
}