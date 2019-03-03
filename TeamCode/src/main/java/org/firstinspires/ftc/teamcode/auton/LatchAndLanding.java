package org.firstinspires.ftc.teamcode.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.util.DriveTrain;
import org.firstinspires.ftc.teamcode.util.GoldAlignUtil;
import org.firstinspires.ftc.teamcode.util.Gyro;

@Autonomous(name = "Auton in Progress", group = "landing")

public class LatchAndLanding extends LinearOpMode {
    DriveTrain driveTrain;
    DcMotor left_lift;
    DcMotor right_lift;
    GoldAlignUtil util;
    private DcMotor extend;

    public void runOpMode() throws InterruptedException {
        init_motors();
        init_vision();

        left_lift.setPower(-0.2);
        right_lift.setPower(-0.2);

        waitForStart();

        left_lift.setPower(0.2);
        right_lift.setPower(0.2);

        sleep(2000);

        left_lift.setPower(0);
        right_lift.setPower(0);

        driveTrain.setPower(0.4);

        sleep(2000);

        driveTrain.setPower(0);

        driveTrain.setPower(-1, 1);

        sleep(2000);

        driveTrain.setPower(0);

        Gyro g = new Gyro(hardwareMap);
        int targetTurn = 90;
        int tolerance = 1;
        double power = 0.2;
        boolean found = false;

        while(Math.abs(Math.abs(g.getHeading())-targetTurn) > tolerance && !found){
            //while the error is greater than the tolerance
            driveTrain.setPower(power,-power);
            if(util.isAligned()){
                found = true;
            }

        }
        driveTrain.stop();

        targetTurn = 0;
        while(Math.abs(Math.abs(g.getHeading())-targetTurn) > tolerance && !found){
            //while the error is greater than the tolerance
            driveTrain.setPower(-power,power);
            if(util.isAligned()){
                found = true;
            }
        }
        driveTrain.stop();

        driveTrain.setPower(-0.5);
        sleep(1000);
        driveTrain.stop();

    }

    private void init_vision() {
        util = new GoldAlignUtil(hardwareMap);
        util.init();
    }

    private void init_motors() {
        left_lift = hardwareMap.dcMotor.get("left");
        right_lift = hardwareMap.dcMotor.get("right");
        extend = hardwareMap.dcMotor.get("extend");
        driveTrain = new DriveTrain(hardwareMap);

        extend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_lift.setDirection(DcMotorSimple.Direction.REVERSE);

        extend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

}
