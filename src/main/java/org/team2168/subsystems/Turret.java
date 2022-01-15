// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.subsystems;

import com.revrobotics.SparkMaxLimitSwitch;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Turret extends SubsystemBase {
  /** Creates a new Turret. */
  private SparkMaxLimitSwitch hallEffectSensor;
  //the name; below was by conor's request
  private TalonFX turtle;
  private static Turret instance = null;

  public Turret() {
    //0 is a placeholder, move to constants later
    turtle = new TalonFX(0);
  }

  public static Turret getInstance() {
    if(instance == null) 
      instance = new Turret();
    return instance;
  }

  /**
* Is forward limit switch closed.
*
* @return  '1' iff forward limit switch input is closed, 0 iff switch is open. This function works
*          regardless if limit switch feature is enabled.  Remote limit features do not impact this routine.
*/
//closed position = turret is 0 degrees rotated(?)
public int isFwdLimitSwitchClosed() {
  return turtle.isFwdLimitSwitchClosed();
}

public boolean isLimitSwitchEnabled() {
  return hallEffectSensor.isLimitSwitchEnabled();
}


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
