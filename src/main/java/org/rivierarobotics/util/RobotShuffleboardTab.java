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

import java.util.HashMap;
import java.util.Map;

public class RobotShuffleboardTab {
    private final ShuffleboardTab tab;
    private final Map<String, NetworkTableEntry> entries;

    public RobotShuffleboardTab(String tab) {
        this.tab = Shuffleboard.getTab(tab);
        this.entries = new HashMap<>();
    }

    public <T> RobotShuffleboardTab setEntry(String key, T value) {
        if (!entries.containsKey(key)) {
            entries.put(key, tab.add(key, value).getEntry());
        } else {
            entries.get(key).setValue(value);
        }
        return this;
    }

    public RobotShuffleboardTab clear() {
        //TODO fix this
//        for (NetworkTableEntry entry : entries.values()) {
//            entry.delete();
//        }
//        entries.clear();
        return this;
    }

    public NetworkTableEntry getEntry(String title) {
        return entries.get(title);
    }
}
