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

/**
 * Subsystem for spinning the color wheel. Consists of a motor that spins a
 * rubber wheel and a REV color sensor. Motor controller is the same as the
 * CheeseWheel. No feedback from motor (no encoders), hence no PID.
 *
 * @see CheeseWheel
 * @see ColorWheel.GameColor
 */
@Singleton
public class ColorWheel extends SubsystemBase implements RRSubsystem {
    private final CheeseWheel cheeseWheel;
    private final ColorSensorV3 colorSensor;
    private final ColorMatch colorMatcher;
    private final MechLogger logger;

    public ColorWheel(I2C.Port sensorId, CheeseWheel cheeseWheel) {
        this.cheeseWheel = cheeseWheel;
        this.colorSensor = new ColorSensorV3(sensorId);
        this.colorMatcher = new ColorMatch();
        this.logger = Logging.getLogger(getClass());

        colorMatcher.addColorMatch(GameColor.RED.matchColor);
        colorMatcher.addColorMatch(GameColor.GREEN.matchColor);
        colorMatcher.addColorMatch(GameColor.BLUE.matchColor);
        colorMatcher.addColorMatch(GameColor.YELLOW.matchColor);
    }

    /**
     * Match sensor color against possible values and convert to
     * {@link GameColor} internal storage unit.
     *
     * @return the current color as a GameColor.
     */
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

    /**
     * Get raw color returned from sensor.
     *
     * @return the RGB color from the sensor.
     */
    public Color getSensorColor() {
        return colorSensor.getColor();
    }

    public void setPositionTicks(double positionTicks) {
        logger.setpointChange(positionTicks);
        cheeseWheel.setPositionTicks(positionTicks);
    }

    @Override
    public double getPositionTicks() {
        return cheeseWheel.getPositionTicks();
    }

    @Override
    public void setPower(double pwr) {
        logger.powerChange(pwr);
        cheeseWheel.setPower(pwr);
    }

    /**
     * Retrieve game-specific strings passed to the driver station from the FMS.
     *
     * @return the string from the FMS if present, else an empty string.
     */
    public static String getFMSString() {
        return DriverStation.getInstance().getGameSpecificMessage();
    }

    /**
     * Converts retrieved game-specific strings into usable characters for
     * 2020/2021 game as noted below or in rule book.
     *
     * <p>Possible 2020 values (upper case):
     * R = red, B = blue, G = green, Y = yellow
     * C = corrupt (error)</p>
     *
     * @return the FMS color character.
     */
    //
    public static char getFMSChar() {
        try {
            return getFMSString().charAt(0);
        } catch (IndexOutOfBoundsException e) {
            return 'C';
        }
    }

    /**
     * Get {@link GameColor} matching returned color character from FMS.
     *
     * @return the color corresponding to the FMS color character.
     */
    public static GameColor getFMSColor() {
        char actualChar = getFMSChar();
        for (GameColor color : GameColor.values()) {
            if (color.gameChar == actualChar) {
                return color;
            }
        }
        return GameColor.NULL;
    }

    /**
     * Defines possible characters that may be returned from the FMS as
     * game-specific strings. Entries may also be used as matches for the
     * color sensor matcher.
     *
     * <p>Corrupt = bad data, null = nothing matching</p>
     */
    public enum GameColor {
        RED(0.561, 0.351, 0.114),
        GREEN(0.197, 0.561, 0.240),
        BLUE(0.143, 0.427, 0.429),
        YELLOW(0.361, 0.524, 0.113),
        CORRUPT(0, 0, 0),
        NULL(0, 0, 0);

        private final char gameChar = name().toLowerCase().charAt(0);
        private final Color matchColor;

        GameColor(double r, double g, double b) {
            this.matchColor = new Color(r, g, b);
        }
    }
}
