package org.firstinspires.ftc.teamcode.util;

import android.util.Log;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Logging {

    public static void log(String caption, Object value, Telemetry telemetry){
        Log.e(caption, value.toString()); //logcat
        telemetry.addData(caption, value);
    }
}