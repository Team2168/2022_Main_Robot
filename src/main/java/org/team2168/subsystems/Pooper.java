// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.subsystems;

import org.team2168.Constants;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Pooper extends SubsystemBase {

  public static Pooper instance = null;

  private DoubleSolenoid peptobismol;

  public static Pooper getInstance() {
    if (instance == null) {
      instance = new Pooper();
    }
    return instance;
  }

  

  public void excrete() {
    peptobismol.set(DoubleSolenoid.Value.kForward);
  }

  public void absorb() {
    peptobismol.set(DoubleSolenoid.Value.kReverse);
  }

  /** Creates a new Pooper. */
  private Pooper() {
    peptobismol = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 
    Constants.Pneumatics.POOPER_DOUBLE_SOLENOID_FORWARDS, 
    Constants.Pneumatics.POOPER_DOUBLE_SOLENOID_BACKWARDS);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
