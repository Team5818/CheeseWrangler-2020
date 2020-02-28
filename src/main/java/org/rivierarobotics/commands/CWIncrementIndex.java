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

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import net.octyl.aptcreator.GenerateCreator;
import net.octyl.aptcreator.Provided;
import org.rivierarobotics.subsystems.CheeseWheel;
import org.rivierarobotics.util.CheeseSlot;

@GenerateCreator
public class CWIncrementIndex extends BasePIDSetPosition<CheeseWheel> {
    public CWIncrementIndex(@Provided CheeseWheel cheeseWheel) {
        super(cheeseWheel, 20, 0.0);
    }

    @Override
    protected void setSetPosition(double position) {
        positionTicks = subsystem.getClosestIndexAngle(subsystem.lastMode, false) + CheeseSlot.INDEX_DIFF;
    }

    @Override
    protected double getPositionTicks() {
        SmartDashboard.putNumber("possset", positionTicks);
        SmartDashboard.putNumber("yticks", subsystem.getPositionTicks());
        SmartDashboard.putNumber("yindex", subsystem.getClosestSlot(subsystem.lastMode, false).getIndex());
        return super.getPositionTicks();
    }

    @Override
    public void end(boolean interrupted) {
        //TODO remove temporary hard set over
        subsystem.getClosestSlot(subsystem.lastMode, false).isFilled = true;
    }
}