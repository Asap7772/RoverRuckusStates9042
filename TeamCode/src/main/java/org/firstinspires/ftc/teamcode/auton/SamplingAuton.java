package org.firstinspires.ftc.teamcode.auton;

import com.disnodeteam.dogecv.detectors.roverrukus.SamplingOrderDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.MineralDetection;

@Disabled
@Autonomous(name = "Sampling Test")
public class SamplingAuton extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        SamplingOrderDetector.GoldLocation location = SamplingOrderDetector.GoldLocation.UNKNOWN;
        MineralDetection detection = new MineralDetection(hardwareMap);
        detection.init();

        while(!opModeIsActive()){
            location = detection.getMineralPos();
            telemetry.addData("Position", location.toString());
            telemetry.update();
        }

        waitForStart();

        if(opModeIsActive()){
            ElapsedTime t = new ElapsedTime();
            t.reset();
            while(location.compareTo(SamplingOrderDetector.GoldLocation.UNKNOWN) == 0){
                telemetry.addData("Position", location.toString());
                telemetry.update();
                if(!opModeIsActive()){
                    break;
                }
                if(t.seconds()>5){
                    break;
                }
            }
        }

        while (opModeIsActive()){
            telemetry.addData("Detected Position", location.toString());
        }

        //TODO combine
        //if it can't fine at 34 inches try detecting at 17 (that is definitely possible)
        //headings
        //========
        //Center go forward 34 inches, stop 2 seconds, go forward 34 inches
        //left go forward 17 inches, turn 45 degrees ccw, go forward 24 inches, go back 24 inches, turn 45 degrees cw
        //right go forward 17 inches, turn 45 degrees cw, go forward 24 inches, go back 24 inches, turn 45 degrees ccw

    }
}
