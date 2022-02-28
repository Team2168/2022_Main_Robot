// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.subsystems;

import org.team2168.Constants.PneumaticsDevices;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

public class Pooper extends SubsystemBase implements Loggable {

  public static Pooper instance = null;
  private DoubleSolenoid pooperSolenoid;

  public static Pooper getInstance() {
    if (instance == null) {
      instance = new Pooper();
    }
    return instance;
  }

  private Pooper() {
    pooperSolenoid = new DoubleSolenoid(PneumaticsDevices.MODULE_TYPE, 
      PneumaticsDevices.POOPER_DOUBLE_SOLENOID_EXTEND, 
      PneumaticsDevices.POOPER_DOUBLE_SOLENOID_RETRACT);
  }

  /**
   * Extends the pooperSolenoid to the extended position 
   */
  @Log(name = "Poop", rowIndex = 0, columnIndex = 1)
  public void extend() {
    pooperSolenoid.set(DoubleSolenoid.Value.kForward);
  }


  /**
   * Retracts the pooperSolenoid to the retracted
   */
  @Log(name = "Unpoop", rowIndex = 0, columnIndex = 2)
  public void retract() {
    pooperSolenoid.set(DoubleSolenoid.Value.kReverse);
  }

  /**
   * 
   * @return true when pooper is extened
   */
  @Log(name = "extended?", rowIndex = 1, columnIndex = 1)
  public boolean isExtended() {
    return pooperSolenoid.get() == DoubleSolenoid.Value.kForward;
  }

  /**
   * 
   * @return true when pooper is retracted
   */
  @Log(name = "retracted?", rowIndex = 1, columnIndex = 2)
  public boolean isRetracted() {
    return pooperSolenoid.get() == DoubleSolenoid.Value.kReverse;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
