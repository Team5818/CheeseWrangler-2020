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

import edu.wpi.first.wpilibj.DigitalInput;

public enum CheeseSlot {
    // Facing like the robot, on the right of the cheese wheel, to the right of the number 1, is slot 0
    // Labeled with labelmaker (small) labels
    ZERO, ONE, TWO, THREE, FOUR;

    private final DigitalInput sensor;

    CheeseSlot() {
        this.sensor = new DigitalInput(this.ordinal() == 0 ? 5 : 10 - this.ordinal());
    }

    public boolean hasBall() {
        return !sensor.get();
    }

    public boolean noBall() {
        return !hasBall();
    }

    public static CheeseSlot slotOfNum(int num) {
        return CheeseSlot.values()[num];
    }

    public enum State {
        BALL, NO_BALL, EITHER
    }
}
