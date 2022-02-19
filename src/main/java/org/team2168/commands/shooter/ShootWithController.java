// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands.shooter;

import java.util.function.DoubleSupplier;
import org.team2168.subsystems.Shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ShootWithController extends CommandBase {
  /** Creates a new ShootWithController. */

  private Shooter shooter;
  private DoubleSupplier speed;
  /**
   * @param k_shooter this is where you plug in your shooter, 
   * @param d this is the speed DoubleSupplier you can plug in it 
   * @return it sets the Shooter and the speed by using parameters
   */

  public ShootWithController(Shooter k_shooter, DoubleSupplier d) {
    shooter = k_shooter;
    speed = d;
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.shoot(speed.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.shoot(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
