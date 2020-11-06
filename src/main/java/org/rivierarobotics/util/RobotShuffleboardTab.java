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

import java.util.LinkedHashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;

public class RobotShuffleboardTab {
    private final ShuffleboardTab tab;
    private final Map<String, NetworkTableEntry> entries;
    private final Map<String, QueueEntry<?>> queue;

    public RobotShuffleboardTab(String name) {
        this.tab = Shuffleboard.getTab(name);
        this.entries = new LinkedHashMap<>();
        this.queue = new LinkedHashMap<>();

        for (ShuffleboardComponent<?> comp : tab.getComponents()) {
            if (comp instanceof SimpleWidget) {
                entries.put(comp.getTitle(), ((SimpleWidget) comp).getEntry());
            }
        }
    }

    public RobotShuffleboardTab addSendable(Sendable sendable) {
        return setEntry(SendableRegistry.getName(sendable), sendable);
    }

    public <T> RobotShuffleboardTab setEntry(String title, T value) {
        queue.put(title, new QueueEntry<>(title, value));
        return this;
    }

    public NetworkTableEntry getEntry(String title) {
        return entries.get(title);
    }

    public int getQueueLength() {
        return queue.size();
    }

    public void update(int minIdx, int maxIdx) {
        List<QueueEntry<?>> queueValues = new LinkedList<>(queue.values());
        for (int i = minIdx; i < maxIdx; i++) {
            QueueEntry<?> queueEntry = queueValues.get(i);
            String title = queueEntry.getTitle();
            if (!entries.containsKey(title)) {
                entries.put(title, tab.add(title, queueEntry.getValue()).getEntry());
            } else {
                entries.get(title).forceSetValue(queueEntry.getValue());
            }
            queue.remove(title);
        }
    }

    private static class QueueEntry<T> {
        private final String title;
        private T value;

        public QueueEntry(String title, T value) {
            this.title = title;
            this.value = value;
        }

        public String getTitle() {
            return title;
        }

        public T getValue() {
            return value;
        }

        public void setValue(T value) {
            this.value = value;
        }
    }
}
