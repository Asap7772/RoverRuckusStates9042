package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.GoldAlignUtil;

@Autonomous(name = "Vision Testing", group = "")
public class VisionTesting extends OpMode {
    GoldAlignUtil util;

    @Override
    public void init() {
        util = new GoldAlignUtil(hardwareMap);
        util.init();
    }

    @Override
    public void loop() {
        telemetry.addData("X Position of the Gold mineral is ", util.getXPosition());
        telemetry.addData("is Aligned?", util.isAligned());
    }

    @Override
    public void stop() {
        super.stop();
        util.stop();
    }
}
