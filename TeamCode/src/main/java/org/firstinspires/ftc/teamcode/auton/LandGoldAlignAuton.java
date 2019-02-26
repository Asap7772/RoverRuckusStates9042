package org.firstinspires.ftc.teamcode.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.DriveTrain;
import org.firstinspires.ftc.teamcode.util.GoldAlignUtil;
import org.firstinspires.ftc.teamcode.util.Gyro;
import org.firstinspires.ftc.teamcode.util.Logging;

@Disabled
@Autonomous(name = "Land + Gold Align", group = "landing")
public class LandGoldAlignAuton extends LinearOpMode {
    DcMotor extend;
    DcMotor left_lift, right_lift;
    private DriveTrain driveTrain;
    private Gyro gyro;
    private GoldAlignUtil util;
    public int offset = 250;

    @Override
    public void runOpMode() throws InterruptedException {
        driveTrain = new DriveTrain(hardwareMap);
        gyro = new Gyro(hardwareMap);

        driveTrain.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        extend = hardwareMap.dcMotor.get("extend");
        left_lift = hardwareMap.dcMotor.get("left");
        right_lift = hardwareMap.dcMotor.get("right");

        extend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_lift.setDirection(DcMotorSimple.Direction.REVERSE);
        left_lift.setDirection(DcMotorSimple.Direction.REVERSE);

        extend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        util = new GoldAlignUtil(hardwareMap);
        util.init();

        left_lift.setPower(0.7);
        right_lift.setPower(0.7);
        extend.setPower(0.4);


        waitForStart();

        left_lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right_lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driveTrain.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left_lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right_lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driveTrain.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        ElapsedTime t = new ElapsedTime();
        t.reset();
        while(opModeIsActive() && t.seconds()<0.5){
            left_lift.setPower(0.4);
            right_lift.setPower(0.4);
        }
        left_lift.setPower(0);
        right_lift.setPower(0);

        t.reset();
        while(opModeIsActive() && t.seconds()<1){
            driveTrain.setPower(1,1);
        }
        t.reset();
        while(opModeIsActive() && t.seconds()<1){
            driveTrain.setPower(1,-1);
        }

        t.reset();
        while(opModeIsActive() && t.seconds()<1){
            driveTrain.setPower(-0.2,-0.2);
        }

        double power = 0.2;
        ElapsedTime elapsedTime = new ElapsedTime();
        elapsedTime.reset();
        boolean first = true;
        while (opModeIsActive() && util.isAligned() != true){
            double xpos = util.getXPosition();
            driveTrain.setPower(-power,power);
            Logging.log("is Aligned", util.isAligned(),telemetry);
            Logging.log("X Position", xpos,telemetry);
            Logging.log("offset", offset,telemetry);
            telemetry.update();
            if(elapsedTime.seconds()>3){
                if(first) {
                    power = -power;
                    first = false;
                }
            }
            if(elapsedTime.seconds()>6){
                break;
            }
        }
        driveTrain.stop();
        util.stop();

        ElapsedTime time = new ElapsedTime();
        time.reset();
        while(time.seconds()<2 && opModeIsActive()) {
            driveTrain.setPower(-0.5);
        }
        driveTrain.stop();
    }
}
