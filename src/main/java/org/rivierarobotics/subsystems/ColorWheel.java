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

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.rivierarobotics.appjack.Logging;
import org.rivierarobotics.appjack.MechLogger;

import javax.inject.Singleton;

@Singleton
public class ColorWheel extends SubsystemBase implements RRSubsystem {
    private final CheeseWheel cheeseWheel;
    private final ColorSensorV3 colorSensor;
    private final ColorMatch colorMatcher;
    private final MechLogger logger;

    public ColorWheel(I2C.Port sensorId, CheeseWheel cheeseWheel) {
        // Uses CheeseWheel motor for movement, no PID/feedback
        this.cheeseWheel = cheeseWheel;
        this.colorSensor = new ColorSensorV3(sensorId);
        this.colorMatcher = new ColorMatch();
        this.logger = Logging.getLogger(getClass());

        colorMatcher.addColorMatch(GameColor.RED.matchColor);
        colorMatcher.addColorMatch(GameColor.GREEN.matchColor);
        colorMatcher.addColorMatch(GameColor.BLUE.matchColor);
        colorMatcher.addColorMatch(GameColor.YELLOW.matchColor);
    }

    // Game color is matched against possible values
    public GameColor getGameColor() {
        ColorMatchResult matchedColor = colorMatcher.matchColor(getSensorColor());
        if (matchedColor == null) {
            return GameColor.NULL;
        }
        for (GameColor color : GameColor.values()) {
            if (matchedColor.color.equals(color.matchColor)) {
                return color;
            }
        }
        return GameColor.NULL;
    }

    // Sensor color is raw feedback from color sensor
    public Color getSensorColor() {
        return colorSensor.getColor();
    }

    public void setPositionTicks(double pos) {
        logger.setpointChange(pos);
        cheeseWheel.setPositionTicks(pos);
    }

    @Override
    public double getPositionTicks() {
        return cheeseWheel.getPositionTicks();
    }

    @Override
    public void setPower(double pwr) {
        logger.powerChange(pwr);
        cheeseWheel.setPositionTicks(pwr);
    }

    public static String getFMSString() {
        return DriverStation.getInstance().getGameSpecificMessage();
    }

    // Possible 2020 values (upper case):
    // R = red, B = blue, G = green, Y = yellow
    // C = corrupt (error)
    public static char getFMSChar() {
        try {
            return getFMSString().charAt(0);
        } catch (IndexOutOfBoundsException e) {
            return 'C';
        }
    }

    // Corrupt is bad data, null is nothing matching
    public enum GameColor {
        RED(0.561, 0.351, 0.114),
        GREEN(0.197, 0.561, 0.240),
        BLUE(0.143, 0.427, 0.429),
        YELLOW(0.361, 0.524, 0.113),
        NULL(0, 0, 0);

        private final char gameChar = name().toLowerCase().charAt(0);
        private final Color matchColor;

        GameColor(double r, double g, double b) {
            this.matchColor = new Color(r, g, b);
        }

        public static GameColor getFMSColor() {
            char actualChar = getFMSChar();
            for (GameColor color : GameColor.values()) {
                if (color.gameChar == actualChar) {
                    return color;
                }
            }
            return GameColor.NULL;
        }
    }
}
