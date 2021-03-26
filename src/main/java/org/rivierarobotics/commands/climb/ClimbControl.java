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

package org.rivierarobotics.commands.climb;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.rivierarobotics.inject.Input;
import org.rivierarobotics.subsystems.Climb;
import org.rivierarobotics.util.MathUtil;

import javax.inject.Inject;

public class ClimbControl extends CommandBase {
    private final Joystick coDriverRightJs;
    private final Joystick driverButtons;
    private final Climb climb;

    @Inject
    public ClimbControl(@Input(Input.Selector.CODRIVER_RIGHT) Joystick coDriverRightJs,
                        @Input(Input.Selector.DRIVER_BUTTONS) Joystick driverButtons,
                        Climb climb) {
        this.coDriverRightJs = coDriverRightJs;
        this.driverButtons = driverButtons;
        this.climb = climb;
        addRequirements(climb);
    }

    @Override
    public void execute() {
        if (driverButtons.getRawButtonPressed(6)) {
//            climb.setPower(MathUtil.fitDeadband(-coDriverRightJs.getY()));
        }
    }
}
