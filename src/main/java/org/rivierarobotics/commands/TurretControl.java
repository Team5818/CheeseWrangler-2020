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

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.rivierarobotics.robot.Robot;
import org.rivierarobotics.subsystems.Flywheel;
import org.rivierarobotics.subsystems.Hood;
import org.rivierarobotics.subsystems.Turret;
import org.rivierarobotics.util.MathUtil;

public class TurretControl extends CommandBase {
    private final Turret turret;
    private final Hood hood;
    private final Joystick leftCoDriverJs;

    public TurretControl(Turret turret, Hood hood) {
        this.hood = hood;
        this.turret = turret;
        this.leftCoDriverJs = Robot.runningRobot.coDriverLeftJs;
        addRequirements(turret, hood);
    }

    @Override
    public void execute() {
        turret.setPower(MathUtil.fitDeadband(leftCoDriverJs.getX()));
        hood.setPower(MathUtil.fitDeadband(leftCoDriverJs.getY()));
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
