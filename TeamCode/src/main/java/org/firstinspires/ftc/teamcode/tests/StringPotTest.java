package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.Potentiometer;

@Disabled
@Autonomous(name = "StringPotTest", group = "misc")
public class StringPotTest extends OpMode {
    private Potentiometer potentiometer;

    @Override
    public void init() {
        potentiometer = new Potentiometer(hardwareMap);

    }

    @Override
    public void loop() {
        telemetry.addData("Voltage: ", potentiometer.getVoltage());
        telemetry.addData("Max Voltage: ", potentiometer.getMaxVoltage());
        telemetry.update();
    }
}
