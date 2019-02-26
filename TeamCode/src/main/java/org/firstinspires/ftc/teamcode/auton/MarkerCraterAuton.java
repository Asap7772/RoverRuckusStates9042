package org.firstinspires.ftc.teamcode.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.DriveTrain;
import org.firstinspires.ftc.teamcode.util.Gyro;

@Disabled
@Autonomous(name = "Marker Crater Auton")
public class MarkerCraterAuton extends LinearOpMode{

    int distance1 = 85; //85 inches from lander to the marker pit
    int angle = 135; // 135 degree turn to the crater
    int distance2 = 96; //96 inches from marker pit to crater
    DriveTrain driveTrain;
    private Gyro gyro;

    @Override
    public void runOpMode() throws InterruptedException {
        driveTrain = new DriveTrain(hardwareMap);
        gyro = new Gyro(hardwareMap);

        driveTrain.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //resetting encoders
        waitForStart();
        driveTrain.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); //usual mode

        driveTrain.setPower(-0.5); //going forward at half power

        double ticksd1 = distance1 * Constants.TICKS_INCH; //number of encoder ticks to travel to marker pit
        while(opModeIsActive()){
            double leftPos = Math.abs(driveTrain.getAveragePositionLeft());
            double rightPos = Math.abs(driveTrain.getAveragePositionRight());

            //telemetry
            telemetry.addData("Left Position", leftPos);
            telemetry.addData("Right Position", rightPos);
            telemetry.addData("Target",ticksd1);
            telemetry.update();

            if(Math.abs(ticksd1-leftPos) < Constants.TOLERANCE){
                driveTrain.setPower(0);
                break;
            }

            if(Math.abs(ticksd1-rightPos) < Constants.TOLERANCE){
                driveTrain.setPower(0);
                break;
            }
        }
        driveTrain.setPower(0);

        hardwareMap.servo.get("marker").setPosition(0); //check if correct (0 or 1) drop marker

        //two second pause
        ElapsedTime t = new ElapsedTime();
        t.reset();
        while(t.seconds()<2){
            telemetry.addData("pause", "2 seconds");
            telemetry.update();
        }

        hardwareMap.servo.get("marker").setPosition(1); //check if correct (0 or 1) lift servo back up

        //two second pause
        t = new ElapsedTime();
        t.reset();
        while(t.seconds()<2){
            telemetry.addData("pause", "2 seconds");
            telemetry.update();
        }

        //turning 135 degrees
        //is it yaw?
        while(opModeIsActive()){
            telemetry.addData("turning", "to Crater");
            telemetry.addData("yaw", gyro.getYaw());
            telemetry.addData("pitch", gyro.getPitch());
            telemetry.addData("roll", gyro.getRoll());
            telemetry.addData("target",135);
            telemetry.update();
            driveTrain.setPower(-0.3, 0.3);

            if(Math.abs(gyro.getHeading()) >= 135 ){
                driveTrain.setPower(0);
                break;
            }
        }
        driveTrain.setPower(0);

        //two second pause
        t = new ElapsedTime();
        t.reset();
        while(t.seconds()<2){
            telemetry.addData("pause", "2 seconds");
            telemetry.update();
        }

        driveTrain.setPower(-0.5); //going forward at half power

        //starting encoder values (since we did not reset again)
        double leftPosStart = Math.abs(driveTrain.getAveragePositionLeft());
        double rightPosStart = Math.abs(driveTrain.getAveragePositionRight());

        double ticksd2 = distance2 * Constants.TICKS_INCH; //number of encoder ticks to travel to crater
        while(opModeIsActive()){
            double leftPos = Math.abs(driveTrain.getAveragePositionLeft());
            double rightPos = Math.abs(driveTrain.getAveragePositionRight());

            //displacement calculated (final-initial position) since initial position is not 0
            double leftDisplacement = Math.abs(leftPos-leftPosStart);
            double rightDisplacement = Math.abs(rightPos-rightPosStart);

            telemetry.addData("Left Displacement", leftDisplacement);
            telemetry.addData("Right Displacement", rightDisplacement);
            telemetry.addData("Target",ticksd2);
            telemetry.update();

            if(Math.abs(ticksd2-leftDisplacement) < Constants.TOLERANCE){
                driveTrain.setPower(0);
                break;
            }

            if(Math.abs(ticksd2-rightDisplacement) < Constants.TOLERANCE){
                driveTrain.setPower(0);
                break;
            }
        }
        driveTrain.setPower(0);
    }
}
