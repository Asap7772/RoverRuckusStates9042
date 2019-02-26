package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.DriveTrain;
import org.firstinspires.ftc.teamcode.util.DrivetrainProportional;

@Disabled
@Autonomous(name = "Drive Distance Test", group = "Driving Forward")
public class DriveDistanceTest extends LinearOpMode{
    private DriveTrain driveTrain;

    @Override
    public void runOpMode() throws InterruptedException {
        driveTrain = new DriveTrain(this.hardwareMap);

        double target = 25;

        DrivetrainProportional proportional = new DrivetrainProportional(Constants.P_CONSTANT_DRIVING, target);

        driveTrain.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        waitForStart();
        driveTrain.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while(opModeIsActive()){
            double leftPos = Math.abs(driveTrain.getAveragePositionLeft());
            double rightPos = Math.abs(driveTrain.getAveragePositionRight());

            if(proportional.reachedTarget(leftPos, rightPos)){
                break;
            }

            double leftPower = proportional.getValue(leftPos);
            double rightPower = proportional.getValue(rightPos);

            //to go forward
            driveTrain.setPower(-leftPower,-rightPower);

            telemetry.addData("Left Position", leftPos);
            telemetry.addData("Right Position", rightPos);
            telemetry.addData("Left Power", leftPower);
            telemetry.addData("Right Power", rightPower);
            telemetry.update();
        }

        driveTrain.setPower(0);

        while(opModeIsActive()){
            telemetry.addData("Left Position", driveTrain.getAveragePositionLeft());
            telemetry.addData("Right Position", driveTrain.getAveragePositionRight());
            telemetry.update();
        }
    }
}
