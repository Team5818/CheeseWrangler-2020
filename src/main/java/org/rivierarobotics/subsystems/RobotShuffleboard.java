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

package org.rivierarobotics.subsystems;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SuppliedValueWidget;

import java.util.HashMap;
import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class RobotShuffleboard {
    private Map<String, RobotShuffleboardTab> tabs;

    public RobotShuffleboard(String... tabs) {
        this.tabs = new HashMap<>(tabs.length);
        addTab(tabs);
    }

    public RobotShuffleboardTab getTab(String tab) {
        return tabs.get(tab);
    }

    public void addTab(String... tabs) {
        for (String tab : tabs) {
            this.tabs.put(tab, new RobotShuffleboardTab(tab));
        }
    }

    public class RobotShuffleboardTab {
        private ShuffleboardTab tab;
        private Map<String, NetworkTableEntry> entries;

        public RobotShuffleboardTab(String tab) {
            this.tab = Shuffleboard.getTab(tab);
            this.entries = new HashMap<>();
        }

        public void addEntry(String... keys) {
            for (String key : keys) {
                NetworkTableEntry entry = tab.add(key, 0).getEntry();
                entries.put(key, entry);
            }
        }

        public RobotShuffleboardTab setEntry(String key, double value) {
            NetworkTableEntry entry = entries.get(key);
            if (entry == null) {
//                addEntry(key);
//                setEntry(key, value);
                return this;
            }
            entry.setNumber(value);
            return this;
        }

        public NetworkTableEntry getEntry(String title) {
            return entries.get(title);
        }
    }
}
