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

package org.rivierarobotics.subsystems;

/**
 * Positions for secondary camera servo. Also stores flip value for
 * {@link org.rivierarobotics.util.CameraFlip} when the servo is moved.
 *
 * @see CameraServo
 * @see org.rivierarobotics.util.CameraFlip
 */
public enum CameraPosition {
    FRONT(0.9, false),
    BACK(0.0, true),
    CLIMB(0.5, false);

    private final double servoValue;
    private final boolean flipped;

    CameraPosition(double servoValue, boolean flipped) {
        this.servoValue = servoValue;
        this.flipped = flipped;
    }

    public double getServoValue() {
        return servoValue;
    }

    public boolean isFlipped() {
        return flipped;
    }
}
