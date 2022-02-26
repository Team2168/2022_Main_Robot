package org.team2168.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.team2168.subsystems.Shooter;


public class WaitForShooterAtSpeed extends CommandBase {

    Shooter shooter;
    boolean atSpeed;

    private static final double THRESHOLD = 20.0;
    public WaitForShooterAtSpeed(Shooter shooter) {
        this.shooter = shooter;
        addRequirements();
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        if (shooter.getSetPoint() - shooter.getVelocity() > THRESHOLD)
            atSpeed = true;
    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return atSpeed;
    }

    @Override
    public void end(boolean interrupted) {

    }
}
