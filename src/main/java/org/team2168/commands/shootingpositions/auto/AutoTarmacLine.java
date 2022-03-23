package org.team2168.commands.shootingpositions.auto;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import org.team2168.commands.hood.HoodToAngle;
import org.team2168.commands.limelight.SetPipeline;
import org.team2168.commands.shooter.SetShooterSpeed;
import org.team2168.subsystems.Hood;
import org.team2168.subsystems.Limelight;
import org.team2168.subsystems.Shooter;

public class AutoTarmacLine extends ParallelCommandGroup {
    public AutoTarmacLine(Hood hood, Shooter shooter, Limelight lime) {
        addCommands(
                new HoodToAngle(hood, Hood.HoodPosition.AUTO_TARMAC_LINE.position_degrees),
                new SetShooterSpeed(shooter, Shooter.ShooterRPM.AUTO_TARMAC_LINE),
                new SetPipeline(lime, Limelight.PIPELINE_TARMAC_LINE)
        );
    }
}