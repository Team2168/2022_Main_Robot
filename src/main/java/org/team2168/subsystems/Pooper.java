// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.subsystems;

import org.team2168.Constants.PneumaticsDevices;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Pooper extends SubsystemBase {

  public static Pooper instance = null;

  private DoubleSolenoid pooperSolenoid;

  public static Pooper getInstance() {
    if (instance == null) {
      instance = new Pooper();
    }
    return instance;
  }

  private Pooper() {
    pooperSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 
    PneumaticsDevices.POOPER_DOUBLE_SOLENOID_EXTEND, 
    PneumaticsDevices.POOPER_DOUBLE_SOLENOID_RETRACT);
  }

  

  public void forward() {
    pooperSolenoid.set(DoubleSolenoid.Value.kForward);
  }

  public void backwards() {
    pooperSolenoid.set(DoubleSolenoid.Value.kReverse);
  }

  public void normal() {
    pooperSolenoid.set(DoubleSolenoid.Value.kOff);
  }

  /** Creates a new Pooper. */
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
