package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.util.Constants;

@Autonomous(name = "Encoder Pot Test")
public class TestEncoder extends OpMode {
    DcMotor x;

    @Override
    public void init() {
        x = hardwareMap.dcMotor.get("3");
    }

    @Override
    public void loop() {
        telemetry.addData("Position", x.getCurrentPosition());
        telemetry.update();
    }
}
