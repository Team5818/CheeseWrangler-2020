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

package org.rivierarobotics.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import net.octyl.aptcreator.GenerateCreator;
import net.octyl.aptcreator.Provided;
import org.rivierarobotics.subsystems.ColorWheel;
import org.rivierarobotics.subsystems.PistonController;
import org.rivierarobotics.util.ColorWheelColor;

@GenerateCreator
public class ColorWheelSetColor extends InstantCommand {
    private ColorWheel colorWheel;
    private ColorWheelColor color;
    //The color you are setting it to
    private PistonController pistonController;

    public ColorWheelSetColor(@Provided ColorWheel colorWheel, ColorWheelColor color, @Provided PistonController pistonController) {
        this.pistonController = pistonController;
        this.colorWheel = colorWheel;
        this.color = color;
    }

    @Override
    public void execute() {
        pistonController.operatePiston(colorWheel.piston, true);
        if (colorWheel.getColor() != color) {
           colorWheel.setManualPower(0.2);
        }

    }
    //TODO be able to rotate wheel until it reaches certain color

}
