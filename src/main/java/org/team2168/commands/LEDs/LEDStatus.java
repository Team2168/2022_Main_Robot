// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands.LEDs;

import org.team2168.subsystems.Climber;
import org.team2168.subsystems.LEDs;
import org.team2168.subsystems.Limelight;
import org.team2168.subsystems.Shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class LEDStatus extends CommandBase {
  /** Creates a new ShowShooterAtSpeed. */
  private LEDs leds;
  private Shooter shooter;
  private Climber climber;
  private Limelight lime;

  public LEDStatus(LEDs leds, Shooter shooter, Climber climber, Limelight lime) {
    this.leds = leds;
    this.shooter = shooter;
    this.climber = climber;
    this.lime = lime;

    addRequirements(leds);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (climber.isClimberHookAttached()) {
      leds.red(false);
      leds.green(false);
      leds.blue(true);
    } else {
        leds.blue(false);

        // if the shooter is at speed, the turret is locked on, 
        // and the limelight sees a target (turret will pretend 
        // to be locked on if there is no target), LEDs are green
        if (lime.getPositionX() < 5.0 && lime.hasTarget() == true) {
          leds.red(false);
          leds.green(true);
        }
        // if the shooter is at speed but there is no target, LEDs are yellow
        else if (shooter.isAtSpeed() && lime.hasTarget() == false) {
          leds.red(true);
          leds.green(true);
        }
        // if the shooter isn't at speed, LEDs are red  
        // regardless of whether there is a target/we are locked on or not
        else {
          leds.red(true);
          leds.green(false);
        }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
