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

import edu.wpi.first.wpilibj2.command.CommandBase;
import net.octyl.aptcreator.GenerateCreator;
import net.octyl.aptcreator.Provided;

@GenerateCreator
public class VisionAim extends CommandBase {

    private final int target;
    private final VisionCommands vision;

    public VisionAim (int target, @Provided VisionCommands vision) {
        //Target = 1 for Bottom, Target = 2 for Top, Target = 3 for inner
        this.target = target;
        this.vision = vision;
    }

    @Override
    public void execute() {
        //TODO: we need to get our heights to match the actual heights of the goal. extraDistance should be correct.
        if (target == 1) {
            vision.autoAimHood(0, .2);
            vision.autoAimTurret(0, .2);
        }
        else {
            if (target == 2) {
            vision.autoAimHood(0, 0.69);
            vision.autoAimTurret(0, .69);
            }
            else {
            vision.autoAimHood(0.74295,  0.69);
            vision.autoAimTurret(0.74295,  0.69);
            }
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
