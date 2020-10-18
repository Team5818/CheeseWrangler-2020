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
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardComponent;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.smartdashboard.SendableRegistry;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class RobotShuffleboardTab {
    private final ShuffleboardTab tab;
    private final Map<String, NetworkTableEntry> entries;

    public RobotShuffleboardTab(String name) {
        this.tab = Shuffleboard.getTab(name);
        this.entries = new HashMap<>();
    }

    public RobotShuffleboardTab addSendable(Sendable sendable) {
        return setEntry(SendableRegistry.getName(sendable), sendable);
    }

    public <T> RobotShuffleboardTab setEntry(String title, T value) {
        if (!entries.containsKey(title)) {
            List<ShuffleboardComponent<?>> components = tab.getComponents();
            for (ShuffleboardComponent<?> comp : components) {
                if (comp.getTitle().equals(title) && comp instanceof SimpleWidget) {
                    NetworkTableEntry entry = ((SimpleWidget) comp).getEntry();
                    entry.setValue(value);
                    entries.put(title, entry);
                    return this;
                }
            }
            entries.put(title, tab.add(title, value).getEntry());
        } else {
            entries.get(title).setValue(value);
        }
        return this;
    }

    public NetworkTableEntry getEntry(String title) {
        return entries.get(title);
    }

    public RobotShuffleboardTab deleteEntry(String title) {
        entries.remove(title).delete();
        return this;
    }

    public RobotShuffleboardTab clear() {
        for (Map.Entry<String, NetworkTableEntry> entry : entries.entrySet()) {
            NetworkTableEntry value = entries.remove(entry.getKey());
            if (value != null && value.isValid() && value.exists()) {
                value.delete();
            }
        }
        return this;
    }
}
