// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package org.team2168.subsystems;


import java.util.Arrays;


import java.util.zip.CRC32;
import java.util.zip.Checksum;
import org.team2168.subsystems.Pooper;
import com.fasterxml.jackson.databind.node.ObjectNode;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
public class ColorSensor extends SubsystemBase {
    private SerialPort serialPort;
    private static ColorSensor instance = null;
    private static Pooper pooper;
    private static final SerialPort.Port SERIAL_PORT_PORT = SerialPort.Port.kOnboard; // port on the roborio
    //private static final int SERIAL_PORT_ADDRESS = 2; // just a place holder, depends on what we give the teensy slave
    byte[] data = new byte[3];
   // byte[] date = new byte[4];
    private ColorSensor() {
        serialPort = new SerialPort(9600, SERIAL_PORT_PORT); 
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
    public byte[] readSensor() {
        serialPort.write(new byte[] {0x12}, 1);// once we write to teensy we receive data from teensy
        return data=serialPort.read(3);
    }
    public static boolean validateSensor(byte[] data) {
         return (data[1] ^ data[2] ^ data[3]) == data[4];
    }


    

    final byte compareRGB[] = {data[0]++,data[1]++,data[2]++};
    final int finalRGB[];
    public final int highestRGBValue;
    public int rgbNormalizer(int highRGBValue){
        Arrays.sort(compareRGB);
        return compareRGB.length - 1 -> highestRGBValue;
        }
 public void divisor(){
    
 int finalRgbRed = (compareRGB[0]/highestRGBValue) * 255;
 int finalRgbGreen = (compareRGB[1]/highestRGBValue) * 255;
 int finalRgbBlue = (compareRGB[2]/highestRGBValue) * 255;
  int finalRGB[] = {finalRgbRed, finalRgbGreen, finalRgbBlue};
 }
   

        
  
    public static boolean red = true; //when jio makes the normalization code, he will say wheter the ball color is true or false
    public static boolean blue = true; //im just setting it as true/false for testing purposes


    public static boolean Red = true; //when jio makes the normalization code, he will say wheter the ball color is true or false
    public static boolean Blue = true; //im just setting it as true/false for testing purposes

    public static void onRedTeam(boolean isTrue){
        if (isTrue == true && red == true){
            pooper.absorb();
            System.out.println("Red Team"); //testing purposes
        }
        else if (isTrue == false || red == false){
            pooper.excrete();
        }
    }
    public static void onBlueTeam(boolean isTrue){
        if (isTrue == true && blue == true){
            pooper.absorb();
            System.out.println("Blue Team");
        }
        else if (isTrue == false || blue == false){
            pooper.excrete();
        }
    }
    public void onStart(){
        onBlueTeam(true);
        onRedTeam(false);
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
        rgbNormalizer(highestRGBValue);
    }
}