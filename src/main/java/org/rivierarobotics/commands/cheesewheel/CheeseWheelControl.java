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

package org.rivierarobotics.commands.cheesewheel;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.rivierarobotics.inject.Input;
import org.rivierarobotics.subsystems.CheeseWheel;
import org.rivierarobotics.util.MathUtil;

import javax.inject.Inject;

public class CheeseWheelControl extends CommandBase {
    private final CheeseWheel cheeseWheel;
    private final Joystick coDriverLeftJs;

    @Inject
    public CheeseWheelControl(@Input(Input.Selector.CODRIVER_LEFT) Joystick coDriverLeftJs,
                              CheeseWheel cheeseWheel) {
        this.cheeseWheel = cheeseWheel;
        this.coDriverLeftJs = coDriverLeftJs;
        addRequirements(cheeseWheel);
    }

    @Override
    public void execute() {
        cheeseWheel.setPower(MathUtil.fitDeadband(coDriverLeftJs.getY()));
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
