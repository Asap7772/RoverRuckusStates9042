package org.firstinspires.ftc.teamcode.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.DriveTrain;
import org.firstinspires.ftc.teamcode.util.GoldAlignUtil;
import org.firstinspires.ftc.teamcode.util.Gyro;
import org.firstinspires.ftc.teamcode.util.GyroProportional;
import org.firstinspires.ftc.teamcode.util.Logging;

@Autonomous(name = "Depot Auton", group = "testing")
public class DepotAuton extends LinearOpMode {

    private DcMotor left_lift, right_lift;
    private DriveTrain driveTrain;
    private GoldAlignUtil util;

    @Override
    public void runOpMode() throws InterruptedException {
        left_lift = hardwareMap.dcMotor.get("left");
        right_lift = hardwareMap.dcMotor.get("right");
        driveTrain = new DriveTrain(hardwareMap);

        left_lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_lift.setDirection(DcMotorSimple.Direction.REVERSE);

        left_lift.setPower(-0.2);
        right_lift.setPower(-0.2);

        waitForStart();
        //auton
        left_lift.setPower(0.4);
        right_lift.setPower(0.4);

        sleep(2500);

        left_lift.setPower(0);
        right_lift.setPower(0);


        Gyro gyro = new Gyro(hardwareMap);

        driveTrain.setPower(0.4);
        sleep(2000);
        driveTrain.stop();

        driveTrain.setPower(-1,1);
        sleep(500);
        driveTrain.stop();

        driveTrain.setPower(-0.4);
        sleep(700);
        driveTrain.stop();

        util = new GoldAlignUtil(hardwareMap);
        util.init();

        //TODO try 30% power
        double power = 0.40;
        ElapsedTime elapsedTime = new ElapsedTime();
        elapsedTime.reset();
        boolean first = true;
        while (opModeIsActive() && util.isAligned() != true) {
            double xpos = util.getXPosition();
            driveTrain.setPower(power, -power);
            Logging.log("is Aligned", util.isAligned(), telemetry);
            Logging.log("X Position", xpos, telemetry);
            telemetry.update();
            if (elapsedTime.seconds() > 6) {
                if (first) {
                    power = -power;
                    first = false;
                }
            }
            if (elapsedTime.seconds() > 12) {
                break;
            }
        }

        driveTrain.stop();
        util.stop();

        driveTrain.setPower(-0.8);
        sleep(1000);
        driveTrain.stop();

        //TODO create depot auton // unlatching + sampling
        //1. Test in all position (left, center and right and check if there are no functionality errors)
        //2. figure out the logic for depot (make a plan, psuedocode) save and upload to drive
        //a. gyro sensor (read up on that again), Proportional turning, game manual for the math (how much to turn and drive forward)
        //3. know when to initialize the gyro sensor


        if(Math.abs(gyro.getHeading())  < 15){ // center so we can drive straight
            driveTrain.setPower(0.4);
            sleep(1000);
            driveTrain.stop();
        }else if(gyro.getHeading() < 0){
            Gyro gyro2 = new Gyro(hardwareMap, "imu2");
            double target = 90;
            while(opModeIsActive() && Math.abs(Math.abs(gyro2.getHeading()) - target) < 1){
                double powerTurn = Constants.P_CONSTANT_TURNING * Math.abs(Math.abs(gyro2.getHeading()) - target);
                driveTrain.setPower(powerTurn, -powerTurn);
            }
        }else{
            Gyro gyro2 = new Gyro(hardwareMap, "imu2");
            double target = 90;
            while(opModeIsActive() && Math.abs(Math.abs(gyro2.getHeading()) - target) < 1){
                double powerTurn = Constants.P_CONSTANT_TURNING * Math.abs(Math.abs(gyro2.getHeading()) - target);
                driveTrain.setPower(-powerTurn, powerTurn);
            }
        }
    }
}
