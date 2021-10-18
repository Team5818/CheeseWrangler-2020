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

package org.rivierarobotics.commands.shooting;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import net.octyl.aptcreator.GenerateCreator;
import net.octyl.aptcreator.Provided;
import org.rivierarobotics.commands.cheesewheel.CheeseWheelCommands;
import org.rivierarobotics.subsystems.CheeseWheel;

@GenerateCreator
public class ShootUntilEmpty extends CommandBase {
    private Command command;
    private final CheeseWheel wheel;
    private final CheeseWheelCommands commands;

    public ShootUntilEmpty(@Provided CheeseWheel cheese, @Provided CheeseWheelCommands comm) {
        this.wheel = cheese;
        this.commands = comm;
    }

    @Override
    public void execute() {
        if (wheel.hasBall() && (command == null || !CommandScheduler.getInstance().isScheduled(command))) {
            Command shootCommand = commands.continuousShoot();
            CommandScheduler.getInstance().schedule(shootCommand);
            this.command = shootCommand;
        }
    }

    @Override
    public boolean isFinished() {
        return !wheel.hasBall();
    }
}
