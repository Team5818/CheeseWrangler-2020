/*
 * This file is part of PracticeBot-2020-example, licensed under the GNU General Public License (GPLv3).
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

/**
 * A storage for constants noting CAN or USB IDs and their associated function
 */
public interface RobotMap {
    // Left DriveTrainSide
    int LEFT_TALON_MASTER = 1;
    int LEFT_SPARK_SLAVE_ONE = 2;
    int LEFT_SPARK_SLAVE_TWO = 3;
    boolean LEFT_INVERT = true;

    // Right DriveTrainSide
    int RIGHT_TALON_MASTER = 4;
    int RIGHT_SPARK_SLAVE_ONE = 5;
    int RIGHT_SPARK_SLAVE_TWO = 6;
    boolean RIGHT_INVERT = false;

    // Turret and Hood
    int TURRET_TALON = 10;
    int HOOD_TALON = 11;
    int FLYWHEEL_TALON = 7;

    // Joysticks
    int DRIVER_LEFT_JS = 0;
    int DRIVER_RIGHT_JS = 1;
    int CODRIVER_LEFT_JS = 2;
    int CODRIVER_RIGHT_JS = 3;
}