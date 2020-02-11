package org.rivierarobotics.util;

public enum LimelightLedState {
    FORCE_ON(3), FORCE_OFF(1), FORCE_BLINK(2), INHERIT(0);

    public final int set;

    LimelightLedState(int set) {
        this.set = set;
    }
}
