/*
 * This file is part of CheeseWrangler-2020, licensed under the GNU General Public License (GPLv3).
 *
 * Copyright (c) Riviera Robotics <https://github.com/Team5818>
 * Copyright (c) contributors
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

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
        if (entries.containsKey(name)) {
            entries.get(name).setValue(value);
        } else {
            entries.put(name, tab.add(title + "/" + name, value)
                    .withWidget("Network Table Tree")
                    .getEntry());
        }
    }

    public void addTab(RobotShuffleboardTab tab) {
        Collection<NetworkTableEntry> entryList = tab.getEntries().values();
        for (NetworkTableEntry e: entryList) {
            setEntry(e.getName().substring(13), e.getValue().getValue());
        }
    }

}
