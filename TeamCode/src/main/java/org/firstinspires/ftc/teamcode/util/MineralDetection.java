package org.firstinspires.ftc.teamcode.util;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.SamplingOrderDetector;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class MineralDetection{

    private SamplingOrderDetector detector;
    private HardwareMap hardwareMap;


    public MineralDetection(HardwareMap hardwareMap){

        this.hardwareMap = hardwareMap;


    }

    public void init(){

        detector = new SamplingOrderDetector();

        //front or back facing
        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance(), 0, false);
        //detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance(), 1, false);
        detector.useDefaults();

        detector.downscale = 0.4; // How much to downscale the input frames

        // Optional Tuning
        detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
        //detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
        detector.maxAreaScorer.weight = 0.001;

        detector.ratioScorer.weight = 15;
        detector.ratioScorer.perfectRatio = 1.0;

        detector.enable();

    }

    public SamplingOrderDetector.GoldLocation getMineralPos(){
        //UNKNOWN,
        // LEFT,
        // CENTER,
        // RIGHT
        return detector.getCurrentOrder();
    }

    public void stop(){

        detector.disable();

    }

}
