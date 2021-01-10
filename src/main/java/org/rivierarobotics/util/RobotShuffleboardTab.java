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

import edu.wpi.cscore.VideoException;
import edu.wpi.cscore.VideoSource;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardComponent;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.smartdashboard.SendableRegistry;

import java.util.LinkedHashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Objects;

public class RobotShuffleboardTab {
    private final ShuffleboardTab tab;
    private final String name;
    private final Map<String, NetworkTableEntry> entries;
    private final Map<String, ShuffleboardTable> tables;

    public RobotShuffleboardTab(String name) {
        this.tab = Shuffleboard.getTab(name);
        this.name = name;
        this.entries = new LinkedHashMap<>();
        this.tables = new LinkedHashMap<>();

        for (ShuffleboardComponent<?> comp : tab.getComponents()) {
            if (comp instanceof SimpleWidget) {
                entries.put(comp.getTitle(), ((SimpleWidget) comp).getEntry());
            }
        }
    }

    public RobotShuffleboardTab setCamera(String name) throws VideoException {
        return setCamera(name, RSTOptions.DEFAULT);
    }

    public RobotShuffleboardTab setCamera(String name, RSTOptions options) throws VideoException {
        return setVideoSource(CameraServer.getInstance().getVideo(name).getSource(), options);
    }

    public RobotShuffleboardTab setVideoSource(VideoSource src) {
        return setVideoSource(src, RSTOptions.DEFAULT);
    }

    public RobotShuffleboardTab setVideoSource(VideoSource src, RSTOptions options) {
        options.applyToComplex(tab.add(src.getName(), src));
        return this;
    }

    public RobotShuffleboardTab setSendable(Sendable sendable) {
        return setSendable(sendable, RSTOptions.DEFAULT);
    }

    public RobotShuffleboardTab setSendable(Sendable sendable, RSTOptions options) {
        options.applyToComplex(tab.add(SendableRegistry.getName(sendable), sendable));
        return this;
    }

    public <T> RobotShuffleboardTab setEntry(String title, T value) {
        return setEntry(title, value, RSTOptions.DEFAULT);
    }

    public <T> RobotShuffleboardTab setEntry(String title, T value, RSTOptions options) {
        if (!entries.containsKey(title)) {
            entries.put(title, options.applyToSimple(tab.add(title, value)).getEntry());
        } else {
            entries.get(title).forceSetValue(value);
        }
        return this;
    }

    public NetworkTableEntry getEntry(String title) {
        NetworkTableEntry entry = entries.get(title);
        if (entry == null) {
            entry = tab.add(title, "").getEntry();
            entries.put(title, entry);
        }
        return entry;
    }

    public Map<String, NetworkTableEntry> getEntries() {
        return entries;
    }

    public ShuffleboardTable getTable(String tableName) {
        return getTable(tableName, RSTOptions.DEFAULT);
    }

    public ShuffleboardTable getTable(String tableName, RSTOptions options) {
        if (tables.get(tableName) == null) {
            addTable(tableName, options);
        }
        return tables.get(tableName);
    }

    public void addTable(String tableName) {
        addTable(tableName, RSTOptions.DEFAULT);
    }

    public void addTable(String tableName, RSTOptions options) {
        tables.put(tableName, new ShuffleboardTable(tableName, this, options));
    }

    public void addTable(ShuffleboardTable table) {
        tables.put(table.getName(), table);
    }

    public String getName() {
        return name;
    }

    public ShuffleboardTab getAPITab() {
        return tab;
    }
}
