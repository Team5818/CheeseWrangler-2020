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

package org.rivierarobotics.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.rivierarobotics.subsystems.*;
import org.rivierarobotics.util.RobotMap;

public class Robot extends TimedRobot {
    public static Robot runningRobot;
    public final DriveTrain driveTrain;
    public final Turret turret;
    public final Hood hood;
    public final Flywheel flywheel;
    public final PistonController pistonController;
    public final Joystick driverLeftJs, driverRightJs, coDriverRightJs, coDriverLeftJs;
    public boolean isArcade = true;

    public Robot() {
        runningRobot = this;

        this.driverLeftJs = new Joystick(RobotMap.Joysticks.DRIVER_LEFT_JS);
        this.driverRightJs = new Joystick(RobotMap.Joysticks.DRIVER_RIGHT_JS);
        this.coDriverLeftJs = new Joystick(RobotMap.Joysticks.CODRIVER_LEFT_JS);
        this.coDriverRightJs = new Joystick(RobotMap.Joysticks.CODRIVER_RIGHT_JS);

        this.driveTrain = new DriveTrain();
        this.hood = new Hood();
        this.flywheel = new Flywheel();
        this.turret = new Turret();
        this.pistonController = new PistonController();
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
        selectiveTickPid(turret);
        selectiveTickPid(hood);
        selectiveTickPid(flywheel);
    }

    private void selectiveTickPid(BasePIDSubsystem subsystem) {
        if (!isDisabled() && !subsystem.getPidController().atSetpoint()) {
            subsystem.tickPid();
        }
    }
}
