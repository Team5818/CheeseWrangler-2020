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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public enum CheeseSlot {
    // Facing like the robot, on the right of the cheese wheel, to the right of the number 1, is slot 0
    ZERO, ONE, TWO, THREE, FOUR;

    private static final int SENSOR_DIO_PIN_OFFSET = 5;
    private final DigitalInput sensor;

    CheeseSlot() {
        if(this.ordinal() == 0){
            this.sensor = new DigitalInput(5);
        } else {
            this.sensor = new DigitalInput(SENSOR_DIO_PIN_OFFSET + (5 - this.ordinal()));
        }
    }

    public boolean hasBall() {
        return !sensor.get();
    }

    public static CheeseSlot slotOfNum(int num) {
        for (CheeseSlot slot : CheeseSlot.values()) {
            if (slot.ordinal() == num) {
                return slot;
            }
        }
        return CheeseSlot.values()[0];
    }
}
