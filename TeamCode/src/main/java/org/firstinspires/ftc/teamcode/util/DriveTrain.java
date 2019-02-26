package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DriveTrain {
    private DcMotor motor1, motor2; //Left Motors
    private DcMotor motor3, motor4; //Right Motors

    private boolean nova = false;

    //TODO consider using the DcMotorEx class
    public DriveTrain(HardwareMap map){
        motor1 = map.dcMotor.get(Constants.DRIVE_MOTOR_TL);
        motor2 = map.dcMotor.get(Constants.DRIVE_MOTOR_BL);
        motor3 = map.dcMotor.get(Constants.DRIVE_MOTOR_TR);
        motor4 = map.dcMotor.get(Constants.DRIVE_MOTOR_BR);

        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor4.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setMode(DcMotor.RunMode mode) {
        motor1.setMode(mode);
        motor2.setMode(mode);
        motor3.setMode(mode);
        motor4.setMode(mode);
    }

    public void setPower(double l1, double l2, double r1, double r2){
        motor1.setPower(l1);
        motor2.setPower(l2);
        motor3.setPower(r1);
        motor4.setPower(r2);
    }

    public void setPower(double left, double right){
        motor1.setPower(left);
        motor2.setPower(left);
        motor3.setPower(right);
        motor4.setPower(right);
    }

    public void setPower(double power){
        setPower(power,power);
    }

    public DcMotor getMotor1() {
        return motor1;
    }

    public DcMotor getMotor2() {
        return motor2;
    }

    public DcMotor getMotor3() {
        return motor3;
    }

    public DcMotor getMotor4() {
        return motor4;
    }

    public double[] getPosition(){
        double[] arr = new double[4];

        arr[0] = motor1.getCurrentPosition();
        arr[1] = motor2.getCurrentPosition();
        arr[2] = motor3.getCurrentPosition();
        arr[3] = motor4.getCurrentPosition();

        return arr;
    }

    public double getAveragePositionLeft(){
        double arr[] = getPosition();
        double avg = (arr[0] + arr[1])/2.0;
        return avg;
    }

    public double getAveragePositionRight(){
        double arr[] = getPosition();
        double avg = (arr[2] + arr[3])/2.0;
        return avg;
    }

    public void stop() {
        setPower(0);
    }
}
