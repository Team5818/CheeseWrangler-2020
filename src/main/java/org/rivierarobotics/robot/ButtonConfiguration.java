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

package org.rivierarobotics.robot;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import org.rivierarobotics.commands.SetTurretPosition;
import org.rivierarobotics.commands.TurretControl;
import org.rivierarobotics.subsystems.Turret;

public class ButtonConfiguration {
    private ButtonConfiguration() { }

    public static void init() {
        Turret turret = Robot.runningRobot.turret;
        var button = new JoystickButton(Robot.runningRobot.coDriverLeftJs,1);
        button.whenPressed(new SetTurretPosition(turret,90));
        var button2 = new JoystickButton(Robot.runningRobot.coDriverLeftJs,2);
        button2.whenPressed(new SetTurretPosition(turret,180));
        var ctrlbtn = new JoystickButton(Robot.runningRobot.coDriverRightJs, 1);
        ctrlbtn.whenPressed(new TurretControl(turret, Robot.runningRobot.hood));
    }
}