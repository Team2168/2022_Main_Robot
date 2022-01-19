// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands.hopper;

import org.team2168.OI;
import org.team2168.RobotContainer;
import org.team2168.subsystems.Hopper;

import edu.wpi.first.wpilibj2.command.CommandBase;



public class DriveHopperWithJoystick extends CommandBase {

  private Hopper hopper;
  private OI oi;

  /** Creates a new DriveHopperWithJoystick. */
  public DriveHopperWithJoystick() {
    // Use addRequirements() here to declare subsystem dependencies.
    hopper = Hopper.getInstance();
    addRequirements(hopper);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    oi = OI.getInstance();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    hopper.driveHopper(RobotContainer.getHopperJoystickValue());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    hopper.driveHopper(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
