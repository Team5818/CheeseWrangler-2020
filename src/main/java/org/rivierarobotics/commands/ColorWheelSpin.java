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

@GenerateCreator
public class ColorWheelSpin extends InstantCommand {
    private ColorWheel colorWheel;
    private PistonController pistonController;
    private int rotations;
    private double circumference = 1.0;
    //Circumference of actual color Wheel, not that of the spinner

    public ColorWheelSpin(@Provided ColorWheel colorWheel, @Provided PistonController pistonController, int rotations) {
        this.colorWheel = colorWheel;
        this.rotations = rotations;
        this.pistonController = pistonController;
    }

    @Override
    public void execute() {
        pistonController.operatePiston(colorWheel.piston, true);
        colorWheel.setPosition((rotations * circumference) / colorWheel.colorWheelRadius);
    }


}
