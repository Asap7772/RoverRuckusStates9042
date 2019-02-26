package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled
@Autonomous(name = "Hook Test", group = "landing")
public class hookTest extends OpMode {

    Servo left, right;
    @Override
    public void init() {
        left = hardwareMap.servo.get("left");
        right = hardwareMap.servo.get("right");
    }

    @Override
    public void loop() {
        if(gamepad2.left_bumper){
            left.setPosition(1);
            right.setPosition(0);
        }else{
            left.setPosition(0);
            right.setPosition(1);
        }
    }
}
