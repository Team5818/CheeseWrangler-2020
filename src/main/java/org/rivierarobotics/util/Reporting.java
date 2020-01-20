package org.rivierarobotics.util;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class Reporting {
    private Reporting() { }

    public static void setOutEntry(ShuffleboardTab tab, String key, double value) {
        addEntry(tab, key).setDouble(value);
    }

    public static void setOutEntry(ShuffleboardTab tab, String key, boolean value) {
        addEntry(tab, key).setBoolean(value);
    }
    private static NetworkTableEntry addEntry(ShuffleboardTab tab, String key) {
        return tab.add(key, 0).getEntry();
    }
}
