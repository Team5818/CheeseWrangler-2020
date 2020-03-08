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

import org.rivierarobotics.subsystems.CheeseWheel;

public enum CheeseSlot {
    // Facing like the robot, on the right of the cheese wheel, to the right of the number 1, is slot 0
    ZERO, ONE, TWO, THREE, FOUR;

    private final int index;
    private final double frontCollectPosition;
    private final double backCollectPosition;
    private final double frontFixPosition;
    private final double backFixPosition;
    private final double shootPosition;
    public boolean isFilled;

    public static final double INDEX_DIFF = 4096.0 / 5;
    private static final double frontOffset = 50;
    private static final double backOffset = (frontOffset + INDEX_DIFF * 2) + 30;
    private static final double frontFixOffset = 577;
    private static final double backFixOffset = 3090;
    private static final double shootOffset = 2920;
    private static final int NUMBER_OF_SLOTS = values().length;

    private static double boundPosition(double pos) {
        if (pos < 0) {
            return pos + 4096;
        } else if (pos > 4095) {
            return pos - 4096;
        }
        return pos;
    }

    CheeseSlot() {
        this.index = ordinal();
        double indexOffset = INDEX_DIFF * index;
        this.frontCollectPosition = boundPosition(indexOffset + frontOffset);
        this.backCollectPosition = boundPosition(indexOffset + backOffset);
        this.frontFixPosition = boundPosition(indexOffset + frontFixOffset);
        this.backFixPosition = boundPosition(indexOffset + backFixOffset);
        this.shootPosition = boundPosition(indexOffset + shootOffset);
    }

    public CheeseSlot next(int amount) {
        var next = (index + amount) % NUMBER_OF_SLOTS;
        while (next < 0) {
            next += NUMBER_OF_SLOTS;
        }
        return values()[next];
    }

    public double getModePosition(CheeseWheel.Mode mode) {
        switch (mode) {
            case COLLECT_FRONT:
                return this.frontCollectPosition;
            case COLLECT_BACK:
                return this.backCollectPosition;
            case FIX_FRONT:
                return this.frontFixPosition;
            case FIX_BACK:
                return this.backFixPosition;
            case SHOOTING:
                return this.shootPosition;
            default:
                throw new IllegalStateException("Unexpected mode: " + mode);
        }
    }

    public int getIndex() {
        return index;
    }
}
