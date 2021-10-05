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

package org.rivierarobotics.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import net.octyl.aptcreator.GenerateCreator;
import net.octyl.aptcreator.Provided;
import org.rivierarobotics.subsystems.DriveTrain;

@GenerateCreator
public class DriveArc extends CommandBase {
    private final DriveTrain driveTrain;
    private final double arcRadius;
    private final double time;
    private double startPos;

    public DriveArc(@Provided DriveTrain driveTrain, double arcRadius, double time) {
        this.driveTrain = driveTrain;
        this.arcRadius = arcRadius;
        this.time = time;
        addRequirements(driveTrain);
    }

    @Override
    public void initialize() {
        startPos = driveTrain.getLeft().getPosition();
    }

    @Override
    public void execute() {
        double leftVel = (2 * Math.PI * (arcRadius + (0.5 * DriveTrain.getTrackwidth()))) / time;
        double rightVel = (2 * Math.PI * (arcRadius - (0.5 * DriveTrain.getTrackwidth()))) / time;
        driveTrain.setVelocity(leftVel, rightVel);
    }

    public DriveArc(@Provided DriveTrain driveTrain, double arcRadius) {
        this(driveTrain, arcRadius, 5);
    }

    @Override
    public void end(boolean interrupted) {
        driveTrain.setPower(0, 0);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(driveTrain.getLeft().getPosition() - startPos)
                > Math.abs(2 * Math.PI * (arcRadius + (0.5 * DriveTrain.getTrackwidth())));
    }
}
