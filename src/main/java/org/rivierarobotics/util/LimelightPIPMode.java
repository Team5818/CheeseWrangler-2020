package org.rivierarobotics.util;

public enum LimelightPIPMode {
    STANDARD(0), MAIN(1), SECONDARY(2);

    public final int set;

    LimelightPIPMode(int set) {
        this.set = set;
    }
}
