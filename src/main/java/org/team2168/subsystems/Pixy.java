// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.pseudoresonance.pixy2api.Pixy2;
import io.github.pseudoresonance.pixy2api.Pixy2CCC;
import io.github.pseudoresonance.pixy2api.links.SPILink;

public class Pixy extends SubsystemBase {

  private static Pixy instance;
  private static Pixy2 cam;

  /** Creates a new Pixy. */
  private Pixy() {
    cam = Pixy2.createInstance(new SPILink());
    cam.init();
    cam.setLED(255, 255, 255); // Sets the RGB LED to full white
  }

  public static Pixy getInstance() {
    if (instance == null) {
      instance = new Pixy();
    }
    
    return instance;
  }

  public void turnLEDsOn() {
    cam.setLamp((byte) 1, (byte) 1);
  }

  public void turnLEDsOff() {
    cam.setLamp((byte) 0, (byte) 0);
  }

  public Pixy2CCC getCCC() {
    return cam.getCCC();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
