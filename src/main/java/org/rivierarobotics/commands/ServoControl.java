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

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.rivierarobotics.inject.Input;
import org.rivierarobotics.util.MathUtil;

import javax.inject.Inject;

//TODO remove after testing servos
public class ServoControl extends CommandBase {
    private final Servo servo;
    private final Joystick js;

    @Inject
    public ServoControl(@Input(Input.Selector.CODRIVER_RIGHT) Joystick js) {
        this.servo = new Servo(0);
        this.js = js;
    }

    @Override
    public void execute() {
        servo.set(MathUtil.fitDeadband(js.getX()));
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
