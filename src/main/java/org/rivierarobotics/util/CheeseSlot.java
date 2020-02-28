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
    ZERO, ONE, TWO, THREE, FOUR;

    public final int index;
    public final double frontCollectPosition;
    public final double backCollectPosition;
    public final double shootPosition;
    public boolean isFilled;

    public static final double INDEX_DIFF = 4096.0 / 5;
    //TODO determine offsets
    private static final double frontOffset = 0;
    private static final double backOffset = 0;
    private static final double shootOffset = -200;

    CheeseSlot() {
        this.index = ordinal();
        this.frontCollectPosition = (INDEX_DIFF * index) + frontOffset;
        this.backCollectPosition = (INDEX_DIFF * index) + backOffset;
        this.shootPosition = (INDEX_DIFF * index) + shootOffset;
    }

    public double getModePosition(CheeseWheel.Mode mode) {
        switch (mode) {
            case COLLECT_FRONT:
                return this.frontCollectPosition;
            case COLLECT_BACK:
                return this.backCollectPosition;
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
