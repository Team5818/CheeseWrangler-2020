package org.rivierarobotics.util;

public class ShooterUtil {

    private ShooterUtil(){
    }

    public static double getTopHeight() {
        //meters
        return 1.63; //diff between robot height and top goal
    }

    public static double getDistanceFromOuterToInnerTarget() {
        //meters
        return 0.74295;
    }

    public static double getYVelocityConstant() {
        //  meters/second
        return Math.sqrt(getTopHeight() * 2 * 9.81);
    }

    public static double getBallMass() {
        //KG
        return 0.14;
    }

    public static double getTConstant() {
        //seconds
        return getYVelocityConstant() / 9.81;
    }

    public static double getMaxFlywheelVelocity() {
        //encoder value
        return 350;
    }

    public static double getMaxHoodAngle() {
        //degrees
        return 42;
    }

    public static double VelocityToTicks(double vel) {
        return ((vel - 0.86) / .003) * (1 / 600.0) * 4.4 * 12;
    }

    public static double getFieldLength() {
        return 7.31; //meters
    }

    public static double getLeftFieldToGoal() {
        return 2.3114; //meters
    }






}
