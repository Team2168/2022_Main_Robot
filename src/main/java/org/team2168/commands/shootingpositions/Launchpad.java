package org.team2168.commands.shootingpositions;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import org.team2168.commands.hood.HoodToAngle;
import org.team2168.commands.limelight.SetPipeline;
import org.team2168.commands.shooter.SetShooterSpeed;
import org.team2168.subsystems.Hood;
import org.team2168.subsystems.Limelight;
import org.team2168.subsystems.Shooter;

public class Launchpad extends ParallelCommandGroup {
    public Launchpad(Hood hood, Shooter shooter, Limelight lime) {
        addCommands(
                new HoodToAngle(hood, Hood.HoodPosition.LAUNCHPAD.position_degrees),
                new SetShooterSpeed(shooter, Shooter.ShooterRPM.LAUNCHPAD.rpm),
                new SetPipeline(lime, Limelight.PIPELINE_LAUNCHPAD_LINE)
        );
    }
}