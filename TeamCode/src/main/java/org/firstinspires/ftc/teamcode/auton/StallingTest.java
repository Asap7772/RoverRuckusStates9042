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

@Autonomous(name = "Motor Stalling Test")
public class StallingTest extends LinearOpMode {
    private DcMotor right_lift;
    private DcMotor left_lift;
    private DriveTrain driveTrain;
    private GoldAlignUtil util;
    private int offset = 250;

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

        util = new GoldAlignUtil(hardwareMap);
        util.init();
        waitForStart();

        left_lift.setPower(0.2);
        right_lift.setPower(0.2);


        ElapsedTime t = new ElapsedTime();
        t.reset();

        while (opModeIsActive() && t.seconds() < 2) {

        }
        t.reset();
        left_lift.setPower(0);
        right_lift.setPower(0);
        while (opModeIsActive() && t.seconds() < 1) {

        }
//        t.reset();
//        while (opModeIsActive() && t.seconds() < 1) {
//            driveTrain.setPower(0.2, 0.2);
//        }

        t.reset();
        while (opModeIsActive() && t.seconds() < 0.5) {
            driveTrain.setPower(-1, 1);
        }

        t.reset();
        while (opModeIsActive() && t.seconds() < 0.50) {
            driveTrain.setPower(-0.4, -0.4);
        }

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
            Logging.log("offset", offset, telemetry);
            telemetry.update();
            if (elapsedTime.seconds() > 3) {
//            if (elapsedTime.seconds() > 4) {
                if (first) {
                    power = -power;
                    first = false;
                }
            }
            if (elapsedTime.seconds() > 6) {
//            if (elapsedTime.seconds() > 8) {
                break;
            }
        }
        driveTrain.stop();
        util.stop();

        double target = 180;
        GyroProportional proportional = new GyroProportional(Constants.P_CONSTANT_TURNING,target);
        Gyro gyro = new Gyro(hardwareMap);

        driveTrain.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        waitForStart();
        driveTrain.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        while(opModeIsActive()){
            double heading = gyro.getHeading();

            if(proportional.reachedTarget(Math.abs(heading))){
                break;
            }

            power = proportional.getValue(Math.abs(heading));

            //to go forward
            driveTrain.setPower(power,-power);

            telemetry.addData("heading", heading);
            telemetry.addData("power", power);
            telemetry.update();
        }

        driveTrain.setPower(0);

//        elapsedTime.reset();
//        while(opModeIsActive() && elapsedTime.seconds()<2){
//            hardwareMap.dcMotor.get("extend").setPower(-1);
//        }
//        hardwareMap.dcMotor.get("extend").setPower(0);
        hardwareMap.dcMotor.get("intake").setPower(-1);
        elapsedTime.reset();
        while(opModeIsActive() && elapsedTime.seconds()<1.5) {
//            left_lift.setPower(1);
//            right_lift.setPower(1);
//            hardwareMap.dcMotor.get("extend").setPower(-0.15);

            if(elapsedTime.seconds()>1){
                driveTrain.setPower(0.5);
            }
        }
        left_lift.setPower(0);
        right_lift.setPower(0);

        elapsedTime.reset();
        while(opModeIsActive() && elapsedTime.seconds()<2) {
        }
    }

}
