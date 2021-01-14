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

import edu.wpi.first.wpilibj.DriverStation;

public class GameData {
    private GameData() {
    }

    public static String getRawData() {
        return DriverStation.getInstance().getGameSpecificMessage();
    }

    // Possible 2020 values (upper case):
    // R = red, B = blue, G = green, Y = yellow
    // C = corrupt (error)
    public static char getChar() {
        try {
            return getRawData().charAt(0);
        } catch (IndexOutOfBoundsException e) {
            return 'C';
        }
    }

    // Corrupt is bad data, null is nothing matching
    public enum ColorChar {
        RED, BLUE, GREEN, YELLOW, CORRUPT, NULL;

        private final char gameChar;

        ColorChar() {
            gameChar = name().toLowerCase().charAt(0);
        }

        public static ColorChar getColor() {
            char actualChar = getChar();
            for (ColorChar color : ColorChar.values()) {
                if (color.gameChar == actualChar) {
                    return color;
                }
            }
            return ColorChar.NULL;
        }
    }
}
