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

package org.rivierarobotics.commands.hood;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.rivierarobotics.inject.Input;
import org.rivierarobotics.subsystems.Hood;
import org.rivierarobotics.util.MathUtil;

import javax.inject.Inject;

public class HoodControl extends CommandBase {
    private final Joystick driverButtons;
    private final Hood hood;
    private final Joystick coDriverRightJs;

    @Inject
    public HoodControl(@Input(Input.Selector.CODRIVER_RIGHT) Joystick coDriverRightJs,
                       @Input(Input.Selector.DRIVER_BUTTONS) Joystick driverButtons,
                       Hood hood) {
        this.hood = hood;
        this.driverButtons = driverButtons;
        this.coDriverRightJs = coDriverRightJs;
        addRequirements(hood);
    }

    @Override
    public void execute() {
        if (!driverButtons.getRawButtonPressed(6)) {
            hood.setPower(MathUtil.fitDeadband(-coDriverRightJs.getY()));
        }
    }
}
