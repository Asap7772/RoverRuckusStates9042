package org.firstinspires.ftc.teamcode.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.DriveTrain;
import org.firstinspires.ftc.teamcode.util.GoldAlignUtil;
import org.firstinspires.ftc.teamcode.util.Gyro;
import org.firstinspires.ftc.teamcode.util.GyroProportional;
import org.firstinspires.ftc.teamcode.util.Logging;

@Autonomous(name = "Gold Align Auton")
public class GoldAlignAuton extends LinearOpMode {

    public int offset = 250;
    public DriveTrain driveTrain;
    DcMotor extend;
    DcMotor left_lift, right_lift;
    DcMotor intake;
    private GoldAlignUtil util;

    @Override
    public void runOpMode() {
        driveTrain = new DriveTrain(hardwareMap);
        extend = hardwareMap.dcMotor.get("extend");
        left_lift = hardwareMap.dcMotor.get("left");
        right_lift = hardwareMap.dcMotor.get("right");
        intake = hardwareMap.dcMotor.get("intake");

        extend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_lift.setDirection(DcMotorSimple.Direction.REVERSE);

        util = new GoldAlignUtil(hardwareMap);
        util.init();

        waitForStart();

        ElapsedTime a = new ElapsedTime();

        a.reset();
        while(a.seconds()<0.5&&opModeIsActive()){
            driveTrain.setPower(-0.2);
        }
        driveTrain.stop();

        a.reset();
        while(a.seconds()<0.2&&opModeIsActive()){
            left_lift.setPower(1);
            right_lift.setPower(1);
        }
        left_lift.setPower(0);
        right_lift.setPower(0);

        double target = 45;
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

            double power = proportional.getValue(Math.abs(heading));

            //to go forward
            driveTrain.setPower(-power,power);

            telemetry.addData("heading", heading);
            telemetry.addData("power", power);
            telemetry.update();
        }

        driveTrain.setPower(0);

        double power = 0.2;
        ElapsedTime elapsedTime = new ElapsedTime();
        elapsedTime.reset();
        boolean first = true;
        while (opModeIsActive() && util.isAligned() != true){
            double xpos = util.getXPosition();
            driveTrain.setPower(power,-power);
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

    public void alignToGold(double dir){
        if(dir - offset > 0){
            driveTrain.setPower(-0.05, 0.05);
            telemetry.addData("Rotation","turning clockwise");
        }
        if(dir - offset < 0){
            driveTrain.setPower(0.05, -0.05);
            telemetry.addData("Rotation","turning counterclockwise");
        }
    }
}
