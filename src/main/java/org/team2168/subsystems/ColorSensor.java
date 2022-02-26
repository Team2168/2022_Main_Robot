// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

public class ColorSensor extends SubsystemBase implements Loggable {
    private SerialPort serialPort;
    private static ColorSensor instance = null;
    public boolean isDataValid;

    private Thread valueUpdateThread = new Thread(
            () -> {
                while (true) {
                    this.readSensor();
                    try {
                        Thread.sleep(20);
                    } catch (InterruptedException e) {
                        DriverStation.reportError("valueUpdateThread interrupted during sleep", e.getStackTrace());
                    }
                }
            });

    private static final SerialPort.Port SERIAL_PORT_PORT = SerialPort.Port.kOnboard; // port on the roborio
    // private static final int SERIAL_PORT_ADDRESS = 2; // just a place holder,
    // depends on what we give the teensy slave

    private volatile int[] data = new int[4]; // {r, g, b}

    private ColorSensor() {
        serialPort = new SerialPort(9600, SERIAL_PORT_PORT);
        serialPort.setReadBufferSize(4);
        serialPort.setWriteBufferSize(0);
        valueUpdateThread.start();
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

    /**
     * A method to get rgb values from the teensy.
     * 
     * This will block until it recieves values, so be careful with where you use
     * it.
     * 
     */
    public void readSensor() {
        serialPort.reset();
        byte[] serialOutput = null;

        System.out.println("trying to read serial!");
        short counter = 0;
       
        while (serialOutput == null) {
            if (serialPort.getBytesReceived() >= 4) {
                serialOutput = serialPort.read(4);
                if ((serialOutput[0] ^ serialOutput[1] ^ serialOutput[2]) != serialOutput[3]) {
                    System.out.println("Received garbled data from teensy!");
                    serialOutput = null;
                    isDataValid = false;
                }
            }
                if (counter%100 == 0)
                    System.out.println("Stalling while trying to read data.  Is teensy connected?");
                counter++;
        }
        System.out.println("successfully read from serial!");
        isDataValid = true;

        // convert values to integers
        for (int i = 0; i < serialOutput.length; i++)
            data[i] = Byte.toUnsignedInt(serialOutput[i]); // Value on teensy will be unsigned int; byte is signed 2^7
    }


    @Log(name = "Color Sensor Alliance", methodName = "toString")
    public Alliance getColor() {
        if (getRed() > getBlue())
            return Alliance.Red;
        else
            return Alliance.Blue;
    }

    @Log
    public boolean isTeamColor() {
        return DriverStation.getAlliance() == getColor();
    }

    @Log(name = "Pooper Petitioner")
    public String decider() {
        if (isTeamColor() == true) {
            return "Keep the ball";
        }
        else {
            return "Poop the ball";
        }
    }

    @Log(name = "Color Sensor Checksum")
    public String checker() {
        if (isDataValid == true) {
            return "Data is valid";
        }
        else {
            return "Data is unvalid";
        }
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run

         System.out.println(
         String.format("R: %d G: %d B: %d", getRed(), getGreen(), getBlue())
         );

    }

    public int getRed() {
        try {
            return data[0];
        } catch (ArrayIndexOutOfBoundsException e) {
            DriverStation.reportError("Error while getting colorsensor red value!  Index out of range",
                    e.getStackTrace());
            return 0;
        }
    }

    public int getGreen() {
        try {
            return data[1];
        } catch (ArrayIndexOutOfBoundsException e) {
            DriverStation.reportError("Error while getting colorsensor green value!  Index out of range",
                    e.getStackTrace());
            return 0;
        }
    }

    public int getBlue() {
        try {
            return data[2];
        } catch (ArrayIndexOutOfBoundsException e) {
            DriverStation.reportError("Error while getting colorsensor blue value!  Index out of range",
                    e.getStackTrace());
            return 0;
        }
    }

    public int[] getData() {
        return data;
    }
}
