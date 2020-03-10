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

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import net.octyl.aptcreator.GenerateCreator;
import net.octyl.aptcreator.Provided;
import org.rivierarobotics.subsystems.CheeseWheel;
import org.rivierarobotics.util.ShooterUtil;
import org.rivierarobotics.util.VisionTarget;

import javax.inject.Inject;

@GenerateCreator
public class CheeseWheelCommandCanceler extends InstantCommand {

    private final CheeseWheelCommands commands;
    private final int direction;
    private final CheeseWheel.AngleOffset mode;

    public CheeseWheelCommandCanceler(@Provided CheeseWheelCommands commands, int direction, CheeseWheel.AngleOffset mode) {
        this.commands = commands;
        this.direction = direction;
        this.mode = mode;
    }

    @Override
    public void initialize() {
        commands.moveToNextIndex(direction, mode).schedule();
    }
}