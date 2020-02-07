package org.rivierarobotics.subsystems;

public enum CheeseWheelMode {
    SHOOTING(0), COLLECT_FRONT(0), COLLECT_BACK(0), CLIMB(0), LAST(0);

    public final int offset;

    CheeseWheelMode(int offset) {
        this.offset = offset;
    }
}
