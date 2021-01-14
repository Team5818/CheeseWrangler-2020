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
import org.rivierarobotics.util.MathUtil;
import org.rivierarobotics.util.NavXGyro;

@GenerateCreator
public class SpinInPlace extends CommandBase {
    private static final double MAX_SPEED = 0.2;
    private final DriveTrain driveTrain;
    private final NavXGyro gyro;
    private final double turnDegrees;
    private final boolean isAbsolute;
    private double initialDegrees;
    private int signum = 1;

    public SpinInPlace(@Provided DriveTrain driveTrain, @Provided NavXGyro gyro, double turnDegrees, boolean isAbsolute) {
        this.driveTrain = driveTrain;
        this.gyro = gyro;
        this.turnDegrees = turnDegrees;
        this.isAbsolute = isAbsolute;
        addRequirements(driveTrain);
    }

    @Override
    public void initialize() {
        this.initialDegrees = gyro.getYaw();
        if ((isAbsolute && turnDegrees < initialDegrees) || (turnDegrees < 0)) {
            this.signum = -1;
        }
    }

    @Override
    public void execute() {
        driveTrain.setPower(signum * MAX_SPEED, signum * -MAX_SPEED);
    }

    @Override
    public boolean isFinished() {
        double yaw = gyro.getYaw();
        return isAbsolute ? (signum == 1 && yaw > turnDegrees) || (signum == -1 && yaw < turnDegrees) :
                MathUtil.wrapToCircle(yaw) - MathUtil.wrapToCircle(initialDegrees) >= MathUtil.wrapToCircle(turnDegrees);
    }
}
