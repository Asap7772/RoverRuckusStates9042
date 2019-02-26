package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Disabled
@Autonomous(name = "Lifter Test", group = "landing")
public class LifterTest  extends OpMode {
    DcMotor extend;
    DcMotor left_lift, right_lift;
    private DcMotor intake;

    double power = 0;

    @Override
    public void init() {
        extend = hardwareMap.dcMotor.get("0");
        left_lift = hardwareMap.dcMotor.get("1");
        right_lift = hardwareMap.dcMotor.get("2");
        intake = hardwareMap.dcMotor.get("3");

        extend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_lift.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop() {
        extend.setPower(-gamepad1.left_stick_y);
        left_lift.setPower(gamepad1.right_stick_y);
        right_lift.setPower(gamepad1.right_stick_y);
        intake.setPower(power);
        if(gamepad1.left_bumper){
            power = 1;
        }

        if(gamepad1.right_bumper){
            power = -1;
        }
    }
}
