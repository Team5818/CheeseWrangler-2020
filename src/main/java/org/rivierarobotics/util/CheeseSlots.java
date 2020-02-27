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

import org.rivierarobotics.subsystems.CheeseWheel;

public enum CheeseSlots {
    ZERO(0),
    ONE(1),
    TWO(2),
    THREE(3),
    FOUR(4);

    private final double frontCollectPosition;
    private final double backCollectPosition;
    private final double shootPosition;
    private boolean isFilled;

    private static final double indexDiff = 360.0 / 5;
    //TODO determine offsets
    private static final double frontOffset = 0;
    private static final double backOffset = 0;
    private static final double shootOffset = 0;

    CheeseSlots(int index) {
        this.frontCollectPosition = (indexDiff * index) + frontOffset;
        this.backCollectPosition = (indexDiff * index) + backOffset;
        this.shootPosition = (indexDiff * index) + shootOffset;
    }

    public boolean isFilled() {
        return isFilled;
    }

    public void setFilled(boolean filled) {
        isFilled = filled;
    }

    public static int getClosestIndex(CheeseWheel.Mode mode, double currentPos, boolean lookForFilled) {
        CheeseSlots[] allSlots = CheeseSlots.values();
        int minIndex = 0;
        double minDiff = 0;

        for (int i = 0; i < allSlots.length; i++) {
            if (!lookForFilled && !allSlots[i].isFilled) {
                continue;
            }

            double intendPos;
            switch (mode) {
                case COLLECT_FRONT:
                    intendPos = allSlots[i].frontCollectPosition;
                    break;
                case COLLECT_BACK:
                    intendPos = allSlots[i].backCollectPosition;
                    break;
                case SHOOTING:
                    intendPos = allSlots[i].shootPosition;
                    break;
                default:
                    throw new IllegalStateException("Unexpected Cheese Wheel mode: " + mode);
            }

            double diff = Math.abs(intendPos - currentPos);
            if (diff < minDiff) {
                minIndex = i;
                minDiff = diff;
            }
        }
        return minIndex;
    }
}
