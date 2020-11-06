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

import java.util.LinkedHashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.stream.Collectors;
import java.util.stream.IntStream;
import javax.inject.Inject;
import javax.inject.Singleton;

@Singleton
public class RobotShuffleboard {
    private static final int maxSendCtr = 50;
    private final Map<String, RobotShuffleboardTab> tabs;
    private final List<BufferEntry> buffer;
    private int robotLoopCtr;

    @Inject
    public RobotShuffleboard() {
        this.tabs = new LinkedHashMap<>();
        this.buffer = new LinkedList<>();
        this.robotLoopCtr = 0;
    }

    public RobotShuffleboardTab getTab(String tabName) {
        if (tabs.get(tabName) == null) {
            addTab(tabName);
        }
        return tabs.get(tabName);
    }

    public void addTab(String... tabNames) {
        for (String tab : tabNames) {
            if (!tabs.containsKey(tab)) {
                tabs.put(tab, new RobotShuffleboardTab(tab));
            }
        }
    }

    public void removeTab(String tabName) {
        tabs.remove(tabName);
    }

    public void update() {
        if (++robotLoopCtr % 5 == 0) {
            buffer.clear();
            int updateCtr = 0;
            for (Map.Entry<String, RobotShuffleboardTab> tabEntry : tabs.entrySet()) {
                RobotShuffleboardTab tab = tabEntry.getValue();
                int len = tab.getQueueLength();
                if (updateCtr + len < maxSendCtr) {
                    tab.update(0, len);
                } else if (updateCtr < maxSendCtr) {
                    tab.update(0, maxSendCtr - updateCtr);
                    buffer.add(new BufferEntry(tab, maxSendCtr - updateCtr, len));
                } else {
                    buffer.add(new BufferEntry(tab, 0, len));
                }
                updateCtr += len;
            }
            robotLoopCtr = 0;
        } else if (buffer.size() != 0) {
            List<BufferEntry> tempBuffer = new LinkedList<>();
            int updateCtr = 0;
            for (BufferEntry entry : buffer) {
                int min = entry.getMin();
                int max = entry.getMax();
                if (updateCtr + (max - min) < maxSendCtr) {
                    entry.getTab().update(min, max);
                } else if (updateCtr < maxSendCtr) {
                    int sendDiff = maxSendCtr - updateCtr;
                    entry.getTab().update(min, max - sendDiff);
                    entry.setMax(max - sendDiff);
                    tempBuffer.add(entry);
                } else {
                    tempBuffer.add(entry);
                }
                updateCtr += max - min;
            }
            buffer.clear();
            buffer.addAll(tempBuffer);
        }
    }

    private static class BufferEntry {
        private final RobotShuffleboardTab tab;
        private int min;
        private int max;

        public BufferEntry(RobotShuffleboardTab tab, int min, int max) {
            this.tab = tab;
            this.min = min;
            this.max = max;
        }

        public RobotShuffleboardTab getTab() {
            return tab;
        }

        public int getMin() {
            return min;
        }

        public int getMax() {
            return max;
        }

        public void setMin(int min) {
            this.min = min;
        }

        public void setMax(int max) {
            this.max = max;
        }
    }
}
