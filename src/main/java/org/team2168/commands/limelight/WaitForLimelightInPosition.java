package org.team2168.commands.limelight;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.team2168.subsystems.Limelight;


public class WaitForLimelightInPosition extends CommandBase {
    private static final double DEFAULT_ANGLE = 1.0;
    private static final double DEFAULT_THRESHOLD_LOOPS = 10;

    Limelight lime;
    double threshold;
    double angle;
    double withinThresholdLoops = 0;


    public WaitForLimelightInPosition(Limelight limelight, double angle, double loops) {
        lime = limelight;
        this.angle = angle;
        threshold = loops;
    }

    public WaitForLimelightInPosition(Limelight limelight) {
        this(limelight, DEFAULT_ANGLE, DEFAULT_THRESHOLD_LOOPS);
    }

    public WaitForLimelightInPosition(Limelight limelight, double angle) {
        this(limelight, angle, DEFAULT_THRESHOLD_LOOPS);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        if (lime.getPositionX() < angle) {
            withinThresholdLoops++;
        } else {
            withinThresholdLoops = 0;
        }
    }

    @Override
    public boolean isFinished() {
        return (withinThresholdLoops > threshold);
    }

    @Override
    public void end(boolean interrupted) {

    }
}
