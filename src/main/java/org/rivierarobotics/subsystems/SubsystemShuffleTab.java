/*
 * This file is part of Placeholder-2020, licensed under the GNU General Public License (GPLv3).
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

package org.rivierarobotics.subsystems;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

import java.util.HashMap;
import java.util.Map;

public class SubsystemShuffleTab {
    private Map<String, NetworkTableEntry> entryMap;
    private ShuffleboardTab tab;

    public SubsystemShuffleTab(String name) {
        this.tab = Shuffleboard.getTab(name);
        this.entryMap = new HashMap<>();
    }

    public void setEntry(String key, Object value) {
        NetworkTableEntry entry = entryMap.get(key);
        if (entry == null) {
            entry = addEntry(key);
        }
        entry.setValue(value);
    }

    public NetworkTableEntry addEntry(String key) {
        NetworkTableEntry entry = tab.add(key, 0).getEntry();
        entryMap.put(key, entry);
        return entry;
    }
}
