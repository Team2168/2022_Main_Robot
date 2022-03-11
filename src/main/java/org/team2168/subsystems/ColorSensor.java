// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.subsystems;

import javax.lang.model.util.ElementScanner6;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;



public class ColorSensor extends SubsystemBase implements Loggable {
    private SerialPort serialPort;
    private static ColorSensor instance = null;
    private static final double DATA_THRESHOLD = 2.0;  // Time after which to consider data stale
   
    static int rNorm = 1; //init to non-zero to avoid divide by zero error when no comms to sensor
    static int gNorm = 1;
    static int bNorm = 1;
    static int MaxVal = 1;
    static int valScal = 255;

    private Thread valueUpdateThread = new Thread(
            () -> {
                serialPort.reset();
                while (true) {
                    this.readSensor();
                    try {
                        Thread.sleep(20);
                    } catch (InterruptedException e) {
//                        DriverStation.reportError("valueUpdateThread interrupted during sleep", e.getStackTrace());
                    }
                }
            });

    private static final SerialPort.Port SERIAL_PORT_PORT = SerialPort.Port.kOnboard; // port on the roborio

    private volatile Color8Bit data = new Color8Bit(1, 1, 1); // avoid divide by zero erros
    private volatile double timestamp = Timer.getFPGATimestamp();

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
    private void readSensor() {
        byte[] serialOutput = null;

        if (serialPort.getBytesReceived() >= 4) {
            
            serialOutput = serialPort.read(4);
            // convert values to integers
            var intValue = new int[serialOutput.length];
            for (int i = 0; i < serialOutput.length; i++)
                intValue[i] = Byte.toUnsignedInt(serialOutput[i]); // Value on teensy will be unsigned int; byte is signed 2^7

            //
            if ((intValue[0] ^ intValue[1] ^ intValue[2]) != intValue[3]) {
                // serialOutput = null;
            } else {
                data = new Color8Bit(intValue[0], intValue[1], intValue[2]);
                timestamp = Timer.getFPGATimestamp();
            }
        }
    }

    private void normRaw(){
        int Norm_scale = valScal;
        MaxVal = data.red;
        if(data.green>MaxVal){
            MaxVal=data.green;
        }
        if(data.blue>MaxVal){
            MaxVal=data.blue;
        }
       rNorm=(Norm_scale*data.red)/MaxVal;
       gNorm=(Norm_scale*data.green)/MaxVal;
       bNorm=(Norm_scale*data.blue)/MaxVal;
    }

    @Log(name = "Color Sensor Alliance")
    public String getColorName() {
        return getColor().name();
    }
    
    public Alliance getColor() {
        Alliance color;
        normRaw();      
    //    if ((rNorm==255) && Math.abs(bNorm - gNorm) < 40 && bNorm > 90)
    //         color = Alliance.Invalid;
    //    else if(rNorm==255)
    //         color = Alliance.Red;
    //     else
    //         color = Alliance.Blue;//   return Alliance.Invalid;
    //     return color;

        if((rNorm==255) && (bNorm + gNorm) < 200) 
            color = Alliance.Red;
        else if ((bNorm==255) && (rNorm + gNorm) < 200)
            color = Alliance.Blue;
        else
            color = Alliance.Invalid;
        return color;
    
        
    }

    @Log(name = "Is team color?")
    public boolean isTeamColor() {
        Alliance alliance = DriverStation.getAlliance();
        return alliance == getColor() && alliance != Alliance.Invalid;
    }

    @Log(name = "Is data stale?")
    public boolean isDataStale() {
        return getTimeSinceLastRead() > DATA_THRESHOLD;
    }

    @Log(name = "time since last read")
    public double getTimeSinceLastRead() {
        return Timer.getFPGATimestamp() - timestamp;
    }

    @Log(name = "Red Raw Value")
    public int rawRed() {
        return data.red;
    }

    @Log(name = "Blue Raw Value")
    public int rawBlue() {
        return data.blue;
    }

    @Log(name = "Green Raw Value")
    public int rawGreen() {
        return data.green;
    }

    

    @Override
    public void periodic() {
        // This method will be called once per scheduler run

        // System.out.println(
        // String.format("R: %d G: %d B: %d", data.red, data.green, data.blue)
        // );

    }

    public Color8Bit getData() {
        return data;
    }
}
