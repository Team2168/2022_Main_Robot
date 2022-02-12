// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.subsystems;

import java.util.zip.CRC32;
import java.util.zip.Checksum;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ColorSensor extends SubsystemBase {
    private I2C i2c;
    private static ColorSensor instance = null;

    private static final I2C.Port I2C_PORT = I2C.Port.kOnboard; // port on the roborio
    private static final int I2C_ADDRESS = 2; // just a place holder, depends on what we give the teensy slave

    byte[] data = new byte[3];
    byte[] date = new byte[4];

    private boolean connected;

    private ColorSensor() {
        i2c = new I2C(I2C_PORT, I2C_ADDRESS);
    }

    /**
     * Constructor for ColorSensor subsystem
     * 
     * @return instance of ColorSensor subsystem
     */
    public static ColorSensor getInstance() {
        if (instance == null)
            instance = new ColorSensor();
        return instance;
    }

    public boolean isConnected() {
        connected = i2c.verifySensor(I2C_ADDRESS, 3, data);
        return connected;
    }

    public byte[] readSensor() {
        i2c.readOnly(data, 3);
        return data;
    }

    public static boolean validateSensor(byte[] data) {
         return (data[1] ^ data[2] ^ data[3]) == data[4];
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
       // isConnected();
       // if (connected) {
            readSensor();
       // }
        System.out.print(" Color Sensor test ");
       // System.out.print(connected);
        System.out.print(" R:"+data[0]);
        System.out.print(" G:"+data[1]);
        System.out.print(" B:"+data[2]);
    }
}
