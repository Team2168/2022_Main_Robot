// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.team2168.subsystems.Shooter;

public class DriveShooterWithConstant extends CommandBase {
  /** Creates a new DriveShooterWithConstant. */

  private double speed;
  private double rotation;
  private Shooter st;

  public DriveShooterWithConstant(double k_speed, double k_rotation, Shooter shooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    st = shooter;
    speed = k_speed;
    rotation = k_rotation;
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    st.drive(speed, rotation);
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
