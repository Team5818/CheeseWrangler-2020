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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.rivierarobotics.inject.Input;
import org.rivierarobotics.subsystems.Hook;
import org.rivierarobotics.util.MathUtil;

import javax.inject.Inject;

public class HookControl extends CommandBase {
    private final Joystick coDriverLeftJs;
    private final Joystick driverButtons;
    private final Hook hook;

    @Inject
    public HookControl(@Input(Input.Selector.CODRIVER_LEFT) Joystick coDriverLeftJs,
                       @Input(Input.Selector.DRIVER_BUTTONS) Joystick driverButtons,
                       Hook hook) {
        this.coDriverLeftJs = coDriverLeftJs;
        this.driverButtons = driverButtons;
        this.hook = hook;
    }

    @Override
    public void execute() {
        if (driverButtons.getRawButton(2)) {
            hook.setPower(MathUtil.fitDeadband(coDriverLeftJs.getX()));
        }
    }
}
