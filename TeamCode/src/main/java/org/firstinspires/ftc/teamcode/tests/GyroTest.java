package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.DriveTrain;
import org.firstinspires.ftc.teamcode.util.Gyro;
import org.firstinspires.ftc.teamcode.util.GyroProportional;

@Disabled
@Autonomous(name = "Gyro Test", group = "turning")
public class GyroTest extends LinearOpMode {
    private DriveTrain driveTrain;

    @Override
    public void runOpMode() throws InterruptedException {
        driveTrain = new DriveTrain(this.hardwareMap);

        double target = 90;
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
            driveTrain.setPower(power,-power);

            telemetry.addData("heading", heading);
            telemetry.addData("power", power);
            telemetry.update();
        }

        driveTrain.setPower(0);
    }



    //small up is up
    //big up is down
}
