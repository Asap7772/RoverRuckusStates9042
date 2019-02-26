package org.firstinspires.ftc.teamcode.teleop;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.DriveTrain;

@TeleOp(name = "ArcadeTeleop")
public class ArcadeTeleop extends OpMode {

    DriveTrain driveTrain;
    DcMotor extend;
    DcMotor left_lift, right_lift;
    DcMotor intake;

    Servo dump;
    private double power = 0;


    @Override
    public void init() {
        driveTrain = new DriveTrain(this.hardwareMap);
        extend = hardwareMap.dcMotor.get("extend");
        left_lift = hardwareMap.dcMotor.get("left");
        right_lift = hardwareMap.dcMotor.get("right");
        intake = hardwareMap.dcMotor.get("intake");
        dump = hardwareMap.servo.get("dump");

        extend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_lift.setDirection(DcMotorSimple.Direction.REVERSE);

        dump.scaleRange(0,1);
    }

    @Override
    public void loop() {
        float yval = gamepad1.left_stick_y;
        float xval = gamepad1.right_stick_x;


        float lpwr = (float) Math.pow(((yval - xval)), 3);
        float rpwr = (float) Math.pow((yval + xval), 3);

        driveTrain.setPower(lpwr,rpwr);

        Servo dump = hardwareMap.servo.get("dump");

        if (gamepad2.right_bumper){
            dump.setPosition(0);
        }
        if(gamepad2.left_bumper) {
            dump.setPosition(1);
        }

        extend.setPower(gamepad2.left_stick_y);
        left_lift.setPower(Math.pow(gamepad2.right_stick_y,3));
        right_lift.setPower(Math.pow(gamepad2.right_stick_y,3));

        if(gamepad2.right_trigger > 0){
            power = gamepad2.right_trigger;
        }else if(gamepad2.left_trigger > 0){
            power = -gamepad2.left_trigger;
        }else{
            power = 0;
        }

        if(gamepad2.right_bumper) {
            dump.setPosition(0);
        }

        if(gamepad2.left_bumper){
            dump.setPosition(1);
        }

        intake.setPower(Math.pow(power, 1/5.0));

    }
}