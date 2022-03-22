package org.team2168.commands;


import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import org.team2168.commands.hood.HoodToAngle;
import org.team2168.commands.shooter.SetShooterSpeed;
import org.team2168.subsystems.Hood;
import org.team2168.subsystems.Shooter;

public class StowEverything extends ParallelCommandGroup {
    public StowEverything(Hood hood, Shooter shooter) {
        addCommands(
                new HoodToAngle(hood, Hood.HoodPosition.ZERO.position_degrees),
                new SetShooterSpeed(shooter, 0.0)
        );
    }
}