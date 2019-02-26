package org.firstinspires.ftc.teamcode.util;

public class GyroProportional {
    public static double P_Constant, target;

    public GyroProportional(double P, double target){
        this.P_Constant = P;
        this.target = target;
    }

    public double getValue(double angle){
        double displacement = target - angle;
        double power = displacement*P_Constant;
        return limit(power);
    }

    private double limit(double power) {
        if(power > 1){
            return 1;
        }
        else if(power < -1){
            return -1;
        }
        return power;
    }

    public boolean reachedTarget(double angle){
        return Math.abs(angle-target) <Constants.TOLERANCE_TURN;
    }
}
