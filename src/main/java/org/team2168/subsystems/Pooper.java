// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.subsystems;

import org.team2168.Constants.PneumaticsDevices;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

public class Pooper extends SubsystemBase implements Loggable {

  public static Pooper instance = null;

  /**
   * Pooper solenoid, it does the pooping
   */
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

  
  /**
   * Extends the pooperSolenoid forward
   */
  public void forward() {
    pooperSolenoid.set(DoubleSolenoid.Value.kForward);
  }


  /**
   * Retracts the pooperSolenoid backwards
   */
  public void backwards() {
    pooperSolenoid.set(DoubleSolenoid.Value.kReverse);
  }

  /**
   * Moves the pooperSolenoid to a normal position
   */
  public void off() {
    pooperSolenoid.set(DoubleSolenoid.Value.kOff);
  }

  /**
   * 
   * @return true when pooper is extened
   */
  @Log(name = "extended?", rowIndex = 1, columnIndex = 1)
  public boolean pooperExtended() {
    return pooperSolenoid.get() == DoubleSolenoid.Value.kForward;
  }


/**
 * 
 * @return true when pooper is retracted
 */
  @Log(name = "retracted?", rowIndex = 1, columnIndex = 2)
  public boolean pooperRetracted() {
    return pooperSolenoid.get() == DoubleSolenoid.Value.kReverse;
  }


  /**
   * 
   * @return true when pooper is off
   */
  @Log(name = "off?", rowIndex = 1, columnIndex = 3)
  public boolean pooperOff() {
    return pooperSolenoid.get() == DoubleSolenoid.Value.kOff;
  }

  /** Creates a new Pooper. */
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
