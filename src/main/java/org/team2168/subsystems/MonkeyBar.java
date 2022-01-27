// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.subsystems;

import org.team2168.Constants.DIO;
import org.team2168.Constants.PneumaticsDevices;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

public class MonkeyBar extends SubsystemBase implements Loggable{
  private DoubleSolenoid solenoid;
  private DigitalInput limitSwitch;

  private static MonkeyBar instance;

  /** Creates a new MonkeyBar. */
  private MonkeyBar() {
    solenoid = new DoubleSolenoid(PneumaticsDevices.MODULE_TYPE, PneumaticsDevices.MONKEYBAR_EXTEND,
        PneumaticsDevices.MONKEYBAR_RETRACT);
    limitSwitch = new DigitalInput(DIO.MONKEYBAR_LIMIT_SWITCH);
  }

  public static MonkeyBar getInstance() {
    if (instance == null)
      instance = new MonkeyBar();
    return instance;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void extend() {
    solenoid.set(DoubleSolenoid.Value.kForward);
  }

  public void retract() {
    solenoid.set(DoubleSolenoid.Value.kReverse);
  }

  @Log(name="Limit Switch")
  public boolean getLimitSwitch() {
    return limitSwitch.get();
  }

  /**
   * Gets whether the solenoid is extended. This is the value <b>as tracked by the
   * object, not as it is in real life.</b> There is no way of tracking the actual
   * position of the solenoid, other than taking a guess off of where the air is
   * being directed.
   * 
   * @return is solenoid extended?
   */
  @Log(name="extended?")
  public boolean isExtended() {
    return solenoid.get() == DoubleSolenoid.Value.kForward;
  }

  /**
   * Gets whether the solenoid is extended. This is the value <b>as tracked by the
   * object, not as it is in real life.</b> There is no way of tracking the actual
   * position of the solenoid, other than taking a guess off of where the air is
   * being directed.
   * 
   * @return is solenoid retracted?
   */
  @Log(name="retracted?")
  public boolean isRetracted() {
    return solenoid.get() == DoubleSolenoid.Value.kReverse;
  }
}
