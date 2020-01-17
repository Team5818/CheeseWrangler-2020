/*
 * This file is part of PracticeBot-2020-example, licensed under the GNU General Public License (GPLv3).
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

package org.rivierarobotics.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.rivierarobotics.subsystems.DriveTrain;
import org.rivierarobotics.subsystems.Turret;
import org.rivierarobotics.util.RobotMap;

public class Robot extends TimedRobot {
    public static Robot runningRobot;
    public final DriveTrain driveTrain;
    public final Turret turret;
    public final Joystick driverLeftJs, driverRightJs, coDriverRightJs, coDriverLeftJs;
    public boolean isArcade = true;

    public Robot() {
        runningRobot = this;
        this.driverLeftJs = new Joystick(RobotMap.DRIVER_LEFT_JS);
        this.driverRightJs = new Joystick(RobotMap.DRIVER_RIGHT_JS);
        this.coDriverLeftJs = new Joystick(RobotMap.CODRIVER_LEFT_JS);
        this.coDriverRightJs = new Joystick(RobotMap.CODRIVER_RIGHT_JS);
        this.driveTrain = new DriveTrain();
        this.turret = new Turret();
    }

    @Override
    public void robotInit() {
        ButtonConfiguration.init();
    }

    @Override
    public void teleopInit() {
    }

    @Override
    public void teleopPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void robotPeriodic() {
        if (!isDisabled()) {
            turret.tickPID();
        }
    }

}