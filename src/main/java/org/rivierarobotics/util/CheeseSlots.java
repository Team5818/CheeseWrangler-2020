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

public class CheeseSlots {
    private static boolean[] slots;

    static {
        slots = new boolean[5];
    }

    public static void increment() {
        boolean temp = slots[slots.length - 1];
        for (int i = slots.length - 1; i >= 0; i--) {
            slots[i] = slots[i - 1];
        }
        slots[0] = temp;
    }

    public static void decrement() {
        boolean temp = slots[0];
        for (int i = 0; i < slots.length - 1; i++) {
            slots[i] = slots[i + 1];
        }
        slots[slots.length - 1] = temp;
    }

    public static void setSlotStatus(int index, boolean filled) {
        slots[index] = filled;
    }

    public static boolean getSlotStatus(int index) {
        return slots[index];
    }
}
