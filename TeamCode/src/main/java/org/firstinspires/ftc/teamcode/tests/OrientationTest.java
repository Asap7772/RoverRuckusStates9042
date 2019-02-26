package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.util.Gyro;
import org.firstinspires.ftc.teamcode.util.Logging;

@Disabled
@Autonomous(name = "Orientation Test", group = "turning")
public class OrientationTest extends OpMode {

    Gyro gyro;

    @Override
    public void init() {
        gyro = new Gyro(hardwareMap);
    }

    @Override
    public void loop() {
        Logging.log("Roll", gyro.getRoll(), telemetry);
        Logging.log("Pitch", gyro.getPitch(), telemetry);
        Logging.log("Yaw", gyro.getYaw(), telemetry);
        telemetry.update();
    }
}
