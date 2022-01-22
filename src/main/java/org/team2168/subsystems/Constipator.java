// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Constipator extends SubsystemBase {
  /** Creates a new Constipator. */
  private DoubleSolenoid poopBlocker;
  private static Constipator _instance;
  public Constipator() {}

  public static Constipator getInstance() {
    if(_instance == null) {
      _instance = new Constipator();
    }
    return _instance;
  }

  public void block() {
    poopBlocker.set(DoubleSolenoid.Value.kForward);
  }

  public void unblock() {
    poopBlocker.set(DoubleSolenoid.Value.kReverse);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
