package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Disabled
@Autonomous(name = "Individual Motor Test", group = "Driving Forward")
public class MotorTest extends OpMode{

    DcMotor motor1, motor2;

    @Override
    public void init() {
        motor1 = this.hardwareMap.dcMotor.get("0");
        motor2 = this.hardwareMap.dcMotor.get("1");
    }

    @Override
    public void loop() {
        if(gamepad1.a){
            motor1.setPower(1);
            motor2.setPower(1);
        }else{
            motor1.setPower(gamepad1.left_stick_y);
            motor2.setPower(gamepad1.right_stick_y);
        }
    }
}