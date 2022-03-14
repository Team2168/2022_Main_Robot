package org.team2168.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.team2168.subsystems.Drivetrain;

import java.util.function.DoubleSupplier;


public class ArcadeDriveCTRE extends CommandBase {

    Drivetrain drivetrain;
    DoubleSupplier fwd, turn;

    public ArcadeDriveCTRE(Drivetrain drivetrain, DoubleSupplier fwd, DoubleSupplier turn) {
        this.drivetrain = drivetrain;
        this.fwd = fwd;
        this.turn = turn;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        drivetrain.setSetPointHeadingTeleop(fwd.getAsDouble(), turn.getAsDouble());
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {

    }
}
