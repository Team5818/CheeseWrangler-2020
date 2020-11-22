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
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

import java.util.Collection;
import java.util.LinkedHashMap;

public class ShuffleboardTable {
    private final LinkedHashMap<String, NetworkTableEntry> entries;
    private final ShuffleboardTab tab;
    private final String tableName;

    public ShuffleboardTable(String tableName, RobotShuffleboardTab tab) {
        this.tableName = tableName;
        this.tab = tab.getAPITab();
        this.entries = new LinkedHashMap<>();
    }

    public void setEntry(String name, Object value) {
        if (entries.containsKey(name)) {
            entries.get(name).setValue(value);
        } else {
            entries.put(name, tab.add(tableName + "/" + name, value)
                    .withWidget("Network Table Tree")
                    .getEntry());
        }
    }

    public ShuffleboardTable addTabData(RobotShuffleboardTab tab) {
        return addEntries(tab.getEntries().values());
    }

    public ShuffleboardTable addEntries(Collection<NetworkTableEntry> ntEntries) {
        for (NetworkTableEntry entry : ntEntries) {
            setEntry(entry.getName().substring(13), entry.getValue().getValue());
        }
        return this;
    }

    public ShuffleboardTable copyToTab(RobotShuffleboardTab targetTab) {
        ShuffleboardTable table = new ShuffleboardTable(tableName, targetTab).addEntries(entries.values());
        targetTab.addTable(table);
        return table;
    }

    public String getName() {
        return tableName;
    }
}
