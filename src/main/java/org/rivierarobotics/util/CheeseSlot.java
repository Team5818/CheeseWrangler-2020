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

public enum CheeseSlot {
    ZERO(0),
    ONE(1),
    TWO(2),
    THREE(3),
    FOUR(4);

    public final int index;
    public final double frontCollectPosition;
    public final double backCollectPosition;
    public final double shootPosition;
    public boolean isFilled;

    public static final double INDEX_DIFF = 4096.0 / 5;
    //TODO determine offsets
    private static final double frontOffset = 0;
    private static final double backOffset = 0;
    private static final double shootOffset = 0;

    CheeseSlot(int index) {
        this.index = index;
        this.frontCollectPosition = (INDEX_DIFF * index) + frontOffset;
        this.backCollectPosition = (INDEX_DIFF * index) + backOffset;
        this.shootPosition = (INDEX_DIFF * index) + shootOffset;
    }

    public double getCurrentModedPosition() {
        return getModedPosition(CheeseWheel.mode);
    }

    public double getModedPosition(CheeseWheel.Mode mode) {
        double pos;
        switch (mode) {
            case COLLECT_FRONT:
                pos = this.frontCollectPosition;
                break;
            case COLLECT_BACK:
                pos = this.backCollectPosition;
                break;
            case SHOOTING:
                pos = this.shootPosition;
                break;
            default:
                throw new IllegalStateException("Unexpected mode: " + mode);
        }
        return pos;
    }

    public int getIndex() {
        return index;
    }
}
