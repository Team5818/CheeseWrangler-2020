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

package org.rivierarobotics.util;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class ShuffleUtil {
    private ShuffleUtil() {
    }

    public static void setOutEntry(ShuffleboardTab tab, String key, double value) {
        addEntry(tab, key).setDouble(value);
    }

    public static void setOutEntry(ShuffleboardTab tab, String key, boolean value) {
        addEntry(tab, key).setBoolean(value);
    }

    private static NetworkTableEntry addEntry(ShuffleboardTab tab, String key) {
        return tab.add(key, 0).getEntry();
    }
}
