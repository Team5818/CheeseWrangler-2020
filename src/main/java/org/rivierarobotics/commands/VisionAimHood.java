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
import org.rivierarobotics.subsystems.DriveTrain;
import org.rivierarobotics.subsystems.Flywheel;
import org.rivierarobotics.subsystems.Hood;
import org.rivierarobotics.util.VisionUtil;

@GenerateCreator
public class VisionAimHood extends CommandBase {
    private final Hood hood;
    private final DriveTrain driveTrain;
    private final Flywheel flywheel;
    private final VisionUtil vision;

    public VisionAimHood(@Provided Hood hd, @Provided DriveTrain dt, @Provided Flywheel fly, @Provided VisionUtil vision) {
        this.hood = hd;
        this.driveTrain = dt;
        this.flywheel = fly;
        this.vision = vision;
        addRequirements(hood, flywheel);
    }

    @Override
    public void execute() {
        double ty = vision.getLLValue("ty");
        double vy = 3.679;  //Vy constant
        double t = 0.375;   //time constant
        double h = 0.69;    //height of goal
        double m = 0.14;    //mass of ball
        double dist = h / Math.tan(Math.toRadians(ty)); //change 0's to VXrobot and VYrobot once available
        double vxz = Math.sqrt(Math.pow((dist / t), 2) + Math.pow(0, 2));
        double hoodAngle = Math.atan2(vy - ((0.336 * vxz + 0.2) / m) * t, vxz);
        double flywheelVelocity = vxz / Math.cos(Math.toRadians(hoodAngle) / 0.0005); //passes through value in ticks/100ms

        hood.setPosition(hoodAngle);
        flywheel.setPositionTicks(flywheelVelocity);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}