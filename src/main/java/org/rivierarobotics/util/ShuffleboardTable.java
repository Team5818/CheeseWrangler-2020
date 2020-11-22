package org.rivierarobotics.util;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardComponent;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.rivierarobotics.subsystems.MultiPID;

import java.util.Collection;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

public class ShuffleboardTable {

    private final ShuffleboardTab tab;
    private final String title;
    private final LinkedHashMap<String, NetworkTableEntry> entries;

    public ShuffleboardTable(String title, RobotShuffleboardTab tab) {
        this.tab = Shuffleboard.getTab(tab.getName());
        this.entries = new LinkedHashMap<>();
        this.title = title;
    }

    public void setEntry(String name, Object value) {
        if(entries.containsKey(name)) {
            entries.get(name).setValue(value);
        } else {
            entries.put(name, tab.add(title + "/" + name, value)
                    .withWidget("Network Table Tree")
                    .getEntry());
        }
    }

    public void addTab(RobotShuffleboardTab tab) {
        Collection<NetworkTableEntry> entryList = tab.getEntries().values();
        for(NetworkTableEntry e: entryList) {
            setEntry(e.getName().substring(13), e.getValue().getValue());
        }
    }

}
