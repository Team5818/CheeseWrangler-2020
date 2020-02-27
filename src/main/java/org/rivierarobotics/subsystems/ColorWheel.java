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

package org.rivierarobotics.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import org.rivierarobotics.util.ColorWheelColor;
import com.revrobotics.ColorSensorV3;

public class ColorWheel extends BasePIDSubsystem {
    private final WPI_TalonSRX colorWheelTalon;
    private ColorWheelColor colorInit;
    private ColorSensorV3 colorSensorV3;
    public final Piston piston = Piston.COLORWHEEL;
    public final double colorWheelRadius;
    private final double colorleniency = 10.0;
    //colorleniency is a setting which modifies how lenient the sensor is to detect a color

    public ColorWheel(int id, double colorWheelRadius, ColorSensorV3 colorSensorV3) {
        super(new PIDConfig(0,0,0,1));
        colorWheelTalon = new WPI_TalonSRX(id);
        colorWheelTalon.configFactoryDefault();
        colorWheelTalon.setNeutralMode(NeutralMode.Brake);
        this.colorWheelRadius = colorWheelRadius;
        this.colorSensorV3 = colorSensorV3;
        colorInit = getColor();
    }
    public ColorWheelColor getColor() {
        if ((colorSensorV3.getRed() <= 255 + colorleniency|| colorSensorV3.getRed() >= 255 - colorleniency)
                && (colorSensorV3.getGreen() <= 0 + colorleniency || colorSensorV3.getGreen() >= 0 - colorleniency)
                && (colorSensorV3.getBlue() <= 0 + colorleniency || colorSensorV3.getBlue() >= 0 - colorleniency)) {
            return ColorWheelColor.RED;
        } else if ((colorSensorV3.getRed() <= 255 + colorleniency|| colorSensorV3.getRed() >= 255 - colorleniency)
                && (colorSensorV3.getGreen() <= 255 + colorleniency || colorSensorV3.getGreen() >= 255 - colorleniency)
                && (colorSensorV3.getBlue() <= 0 + colorleniency || colorSensorV3.getBlue() >= 0 - colorleniency)) {
            return ColorWheelColor.YELLOW;
        } else if ((colorSensorV3.getRed() <= 0 + colorleniency|| colorSensorV3.getRed() >= 0 - colorleniency)
                && (colorSensorV3.getGreen() <= 255 + colorleniency || colorSensorV3.getGreen() >= 255 - colorleniency)
                && (colorSensorV3.getBlue() <= 0 + colorleniency || colorSensorV3.getBlue() >= 0 - colorleniency)) {
            return  ColorWheelColor.GREEN;
        } else if ((colorSensorV3.getRed() <= 255 + colorleniency|| colorSensorV3.getRed() >= 255 - colorleniency)
                && (colorSensorV3.getGreen() <= 255 + colorleniency || colorSensorV3.getGreen() >= 255 - colorleniency)
                && (colorSensorV3.getBlue() <= 255 + colorleniency || colorSensorV3.getBlue() >= 255 - colorleniency)) {
            return  ColorWheelColor.BLUE;
        }
        return null;
    }

    @Override
    public double getPositionTicks() {
        return colorWheelTalon.getSensorCollection().getQuadratureVelocity();
    }

    @Override
    protected void setPower(double pwr) {
        colorWheelTalon.set(pwr);
    }
}