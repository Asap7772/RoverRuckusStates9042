package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.DriveTrain;

@TeleOp(name = "Motor Test")
public class MotorTest extends OpMode {
    DriveTrain driveTrain;

    @Override
    public void init() {
        driveTrain = new DriveTrain(this.hardwareMap);
    }

    @Override
    public void loop() {
        driveTrain.setPower(gamepad1.left_stick_y, gamepad1.right_stick_y, gamepad2.left_stick_y, gamepad2.right_stick_y);
    }
}
