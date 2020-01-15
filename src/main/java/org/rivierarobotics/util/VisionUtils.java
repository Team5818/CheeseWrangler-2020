package org.rivierarobotics.util;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class VisionUtils {
    public static NetworkTable LIMELIGHT = NetworkTableInstance.getDefault().getTable("limelight");
    private static final double LLAngle = 0, LLHeight = 10, targetHeight = 20;

    public static double getLLValue(String key) {
        return LIMELIGHT.getEntry(key).getDouble(0);
    }

    public static double getDistanceToTarget() {
        if(getLLValue("tv") == 1) {
            return (targetHeight - LLHeight) / Math.tan(LLAngle + getLLValue("ty"));
        } else {
            return -1;
        }
    }
}
