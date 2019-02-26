package org.firstinspires.ftc.teamcode.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.util.GoldAlignUtil;
import org.firstinspires.ftc.teamcode.util.Logging;

@Autonomous(name = "Gold Align Test")
public class GoldAlignTest extends OpMode {

    private GoldAlignUtil util;
    public int offset = 250;

    @Override
    public void init() {
        util = new GoldAlignUtil(hardwareMap);
        util.init();
    }

    @Override
    public void loop() {
        double xpos = util.getXPosition();
        boolean isAligned = util.isAligned();
        Logging.log("X Position", xpos,telemetry);
        Logging.log("offset", offset,telemetry);
        telemetry.update();
        telemetry.update();
    }

    @Override
    public void stop() {
        super.stop();
        util.stop();
    }
}
