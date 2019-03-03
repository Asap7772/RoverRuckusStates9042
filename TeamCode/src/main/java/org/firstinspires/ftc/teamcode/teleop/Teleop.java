package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.DriveTrain;

@TeleOp(name = "Teleop")
public class Teleop extends OpMode {
    DriveTrain driveTrain;
    DcMotor extend;
    DcMotor left_lift, right_lift;
    DcMotor intake;

    Servo dump;
    private double power = 0;
    private boolean toggle = false;

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
        dump.setPosition(0);

        intake.setPower(0.01);
    }

    @Override
    public void loop() {
        if(gamepad1.right_trigger >0.5){
            driveTrain.setPower(this.gamepad1.left_stick_y*0.3, this.gamepad1.right_stick_y*0.3);
        }else {
            driveTrain.setPower(Math.pow(this.gamepad1.left_stick_y,3), Math.pow(this.gamepad1.right_stick_y,3));
        }

        if(gamepad2.b){
            if(toggle){
                toggle = false;
            }else{
                toggle = true;
            }
        }

        if(toggle){
            if(gamepad2.left_stick_y==0) {
                extend.setPower(-0.15);
            }else{
                extend.setPower(gamepad2.left_stick_y);
                toggle = false;
            }
        }else{
            extend.setPower(gamepad2.left_stick_y);
        }


        if(gamepad2.right_trigger > 0){
            power = 0;
        }else if(gamepad2.left_trigger > 0){
            power = -1;
        }else if(gamepad2.y){
            power = 1;
        }

        //>1 up
        if(gamepad2.a){
            left_lift.setPower(-0.2);
            right_lift.setPower(-0.2);
        }else{
            left_lift.setPower(Math.pow(gamepad2.right_stick_y,3));
            right_lift.setPower(Math.pow(gamepad2.right_stick_y,3));
        }

        if(gamepad2.right_bumper) {
            dump.setPosition(0);
        }

        if(gamepad2.left_bumper){
            dump.setPosition(1);
        }

        intake.setPower(power);

        telemetry.addData("Toggle", toggle);
        telemetry.update();
    }
}
