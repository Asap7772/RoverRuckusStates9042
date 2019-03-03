package org.firstinspires.ftc.teamcode.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.DriveTrain;
import org.firstinspires.ftc.teamcode.util.GoldAlignUtil;
import org.firstinspires.ftc.teamcode.util.Logging;

@Autonomous(name = "Landing Example", group = "testing")
public class Landing extends LinearOpMode{
    DriveTrain driveTrain;
    DcMotor left_lift, right_lift;
    GoldAlignUtil util;


    @Override
    public void runOpMode() throws InterruptedException {
        //init
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

        sleep(2000);

        left_lift.setPower(0);
        right_lift.setPower(0);

        driveTrain.setPower(0.2);
        sleep(2000);
        driveTrain.stop();

        driveTrain.setPower(-1,1);
        sleep(400);
        driveTrain.stop();

        driveTrain.setPower(-0.4);
        sleep(500);
        driveTrain.stop();

        util = new GoldAlignUtil(hardwareMap);
        util.init();

        //TODO try 30% power
        double power = -0.4 ;
//        double power = -0.3 ;
        ElapsedTime elapsedTime = new ElapsedTime();
        elapsedTime.reset();
        boolean first = true;
        while (opModeIsActive() && util.isAligned() != true) {
            double xpos = util.getXPosition();
            driveTrain.setPower(-power, power);
            Logging.log("is Aligned", util.isAligned(), telemetry);
            Logging.log("X Position", xpos, telemetry);
//            Logging.log("offset", offset, telemetry);
            telemetry.update();
//            if (elapsedTime.seconds() > 3) {
            if (elapsedTime.seconds() > 4) {
                if (first) {
                    power = -power;
                    first = false;
                }
            }
//            if (elapsedTime.seconds() > 6) {
            if (elapsedTime.seconds() > 8) {
                break;
            }
        }
        driveTrain.stop();
        util.stop();

    }
}