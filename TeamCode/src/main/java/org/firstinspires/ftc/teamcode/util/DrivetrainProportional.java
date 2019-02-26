package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.teamcode.util.Constants;

public class DrivetrainProportional {
    public static double P_Constant, target;

    public DrivetrainProportional(double P, double target){
        this.P_Constant = P;
        this.target = target;
    }

    public double getValue(double pos){
        double displacement = target - (pos/Constants.TICKS_INCH);
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

    public boolean reachedTarget(double posLeft, double posRight){
        boolean left = Math.abs(target * Constants.TICKS_INCH - Math.abs(posLeft)) < Constants.TOLERANCE;
        boolean right = Math.abs(target * Constants.TICKS_INCH - Math.abs(posRight)) < Constants.TOLERANCE;
        return left && right;
    }
}
