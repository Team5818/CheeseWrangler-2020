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

package org.rivierarobotics.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import net.octyl.aptcreator.GenerateCreator;
import net.octyl.aptcreator.Provided;
import org.rivierarobotics.util.ShooterUtil;
import org.rivierarobotics.util.VisionTarget;

@GenerateCreator
public class VisionAim extends ParallelCommandGroup {

    public VisionAim(VisionTarget target, @Provided VisionCommands vision) {
        //TODO: we need to get our heights to match the actual heights of the goal. extraDistance should be correct.
        if (target == VisionTarget.BOTTOM) {
            addCommands(vision.autoAimHood(0, 0.2), vision.autoAimTurret(0, 1));
        } else {
            if (target == VisionTarget.TOP) {
                addCommands(vision.autoAimHood(0, ShooterUtil.getTopHeight()), vision.autoAimTurret(0, ShooterUtil.getTopHeight()));
            } else {
                addCommands(vision.autoAimHood(ShooterUtil.getDistanceFromOuterToInnerTarget(), ShooterUtil.getTopHeight()), vision.autoAimTurret(ShooterUtil.getDistanceFromOuterToInnerTarget(), ShooterUtil.getTopHeight()));
            }
        }

    }

}
