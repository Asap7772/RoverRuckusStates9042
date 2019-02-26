package org.firstinspires.ftc.teamcode.vision;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.DriveTrain;
import org.firstinspires.ftc.teamcode.util.Gyro;

@Disabled
@Autonomous(name = "Align Gold Brunswick")
public class GoldAlignBrunswick extends LinearOpMode {
    // Detector object
    private GoldAlignDetector detector;
    DriveTrain driveTrain;
    private Gyro gyro;

    @Override
    public void runOpMode() throws InterruptedException {
        // Set up detector
        detector = new GoldAlignDetector();
        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        detector.useDefaults();

        detector.alignSize = 225;
        detector.alignPosOffset = 0;
        detector.downscale = 0.4;

        detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA;
        detector.maxAreaScorer.weight = 0.005;

        detector.ratioScorer.weight = 5;
        detector.ratioScorer.perfectRatio = 1.0;

        detector.enable();

        driveTrain = new DriveTrain(hardwareMap);
        gyro = new Gyro(hardwareMap);

        driveTrain.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        waitForStart();
        driveTrain.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        ElapsedTime t = new ElapsedTime();
        t.reset();
        driveTrain.setPower(-0.5);
        //make sure it is close enough to detect the mineral (change time)
        while(t.seconds()<1 && opModeIsActive()){
            telemetry.addData("Driving", "Forward");
            telemetry.update();
        }
;
        driveTrain.setPower(0);

        t.reset();
        driveTrain.setPower(-0.3,0.3);
        //align with the first mineral (change time)
        while(t.seconds()<0.5 && opModeIsActive()){
            telemetry.addData("Turning", "Left");
            telemetry.update();
        }

        driveTrain.setPower(0);

        t.reset();
        double rot = 1;
        driveTrain.setPower(0.3,-0.3);
        //by default hit the last mineral (change time) make sure can detect mineral (change turn power)
        while(opModeIsActive() && t.seconds()<1){
            telemetry.addData("Turning", "Mineral");
            telemetry.update();
            if(detector.getAligned()){
                rot = t.seconds();
                break;
            }
        }
        driveTrain.setPower(0);

        t.reset();
        driveTrain.setPower(-0.7);
        //drive just far enough to hit the mineral
        while (opModeIsActive() && t.seconds()<1){
            telemetry.addData("Hitting", "Mineral");
        }
        driveTrain.setPower(0);

        t.reset();
        driveTrain.setPower(-0.3,0.3);
        Gyro g = new Gyro(hardwareMap);
        //turn back so aligned to marker pit(check if needed)
        while(opModeIsActive() && t.seconds()<rot/2){
            telemetry.addData("Turning", "Back");
            telemetry.update();
            if(Math.abs(Math.abs(g.getYaw())-0) <0.3){
                break;
            }
        }
        driveTrain.setPower(0);

        t.reset();
        driveTrain.setPower(-0.7);
        //drive to the marker area
        while (opModeIsActive() && t.seconds()<1){
            telemetry.addData("Drive", "to marker drop");
        }
        driveTrain.setPower(0);

        //drop the marker
        hardwareMap.servo.get("marker").setPosition(0); //check if correct (0 or 1)
    }
}
