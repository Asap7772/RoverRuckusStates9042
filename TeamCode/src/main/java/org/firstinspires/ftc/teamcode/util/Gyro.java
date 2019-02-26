package org.firstinspires.ftc.teamcode.util;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class Gyro {
    // The IMU sensor object
    public BNO055IMU imu;

    // State used for updating telemetry
    Orientation angles;

    public Gyro(HardwareMap map){
        initGyro(map);
    }

    public void initGyro(HardwareMap hardwareMap)
    {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

    }

    public void initGyro(HardwareMap hardwareMap, String name)
    {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";

        imu = hardwareMap.get(BNO055IMU.class, name);
        imu.initialize(parameters);

    }

    //TODO: CHANGE THE AXES BASED ON WHERE THE HUB IS PLACED
    private void updateAngles() {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC,AxesOrder.ZYX,AngleUnit.DEGREES);
    }



    public double getRoll(){
        updateAngles();
        return angles.secondAngle;
    }
    public double getPitch(){
        updateAngles();
        return angles.thirdAngle;
    }
    public double getYaw() {
        updateAngles();
        return angles.firstAngle;
    }

    public double getHeading() {
        return getYaw();
    }
}
