package org.team2168.commands.shootingpositions;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import org.team2168.commands.hood.HoodToAngle;
import org.team2168.commands.limelight.SetPipeline;
import org.team2168.commands.shooter.SetShooterSpeed;
import org.team2168.subsystems.Hood;
import org.team2168.subsystems.Limelight;
import org.team2168.subsystems.Shooter;

public class TarmacLine extends ParallelCommandGroup {
    public TarmacLine(Hood hood, Shooter shooter, Limelight lime) {
        addCommands(
                new HoodToAngle(hood, Hood.HoodPosition.TARMAC_LINE.position_degrees),
                new SetShooterSpeed(shooter, Shooter.ShooterRPM.TARMAC_LINE),
                new SetPipeline(lime, Limelight.PIPELINE_TARMAC_LINE)
        );
    }
}