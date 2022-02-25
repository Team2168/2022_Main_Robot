// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.subsystems;

import org.team2168.Constants.PneumaticsDevices;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.annotations.Log;
import io.github.oblarg.oblog.Loggable;

public class IntakeRaiseAndLower extends SubsystemBase implements Loggable {
  /** Creates a new IntakeRaiseAndLower. */
  private DoubleSolenoid upAndDown;
  private static IntakeRaiseAndLower _instance = null;
  private IntakeRaiseAndLower() {
    upAndDown = new DoubleSolenoid(PneumaticsDevices.MODULE_TYPE, PneumaticsDevices.INTAKE_LOWER,
                                   PneumaticsDevices.INTAKE_RAISE);
  }

  public static IntakeRaiseAndLower getInstance() {
    if(_instance == null) {
      _instance = new IntakeRaiseAndLower();
    }
    return _instance;
  }

  public void lower() {
    upAndDown.set(DoubleSolenoid.Value.kForward);
  }

  public void raise() {
    upAndDown.set(DoubleSolenoid.Value.kReverse);
  }

  @Log(name = "Intake Lowered?")
  public boolean isIntakeLowered() {
    return upAndDown.get() == DoubleSolenoid.Value.kForward;
  }

  @Log(name = "Intake Raised?")
  public boolean isIntakeRaised() {
    return upAndDown.get() == DoubleSolenoid.Value.kReverse;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
