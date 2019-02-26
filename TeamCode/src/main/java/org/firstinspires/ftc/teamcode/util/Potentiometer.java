package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Potentiometer {
    AnalogInput potentiometer;

    public Potentiometer(HardwareMap map){
        potentiometer = map.analogInput.get("pot");
    }

    public double getVoltage(){
        return potentiometer.getVoltage();
    }

    public double getMaxVoltage(){
        return potentiometer.getMaxVoltage();
    }
}
