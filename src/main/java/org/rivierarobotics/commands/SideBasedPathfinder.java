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
import jaci.pathfinder.followers.EncoderFollower;
import net.octyl.aptcreator.Provided;
import org.rivierarobotics.subsystems.DriveTrain;

public class SideBasedPathfinder extends CommandBase {
    public EncoderFollower left;
    protected EncoderFollower right;
    private double wheelDiameter = 4.0;

    public SideBasedPathfinder(@Provided EncoderFollower left, @Provided EncoderFollower right, @Provided DriveTrain dt) {
        this.left = left;
        this.right = right;
        //TODO put everything in this command, including making the encoderfollowers and waypoints.
        // Ideally drivetrain should just have set/get methods and everything else can be done higher up.
        // Also using @Provided for EncoderFollowers isn't going to work.
        //left.configureEncoder();
    }
}

