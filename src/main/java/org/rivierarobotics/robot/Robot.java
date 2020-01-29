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

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.rivierarobotics.commands.LimelightLedToggle;
import org.rivierarobotics.inject.DaggerGlobalComponent;
import org.rivierarobotics.inject.GlobalComponent;
import org.rivierarobotics.util.VisionUtil;

public class Robot extends TimedRobot {
    private GlobalComponent globalComponent;

    @Override
    public void robotInit() {
        globalComponent = DaggerGlobalComponent.create();
        globalComponent.robotInit();
    }

    @Override
    public void robotPeriodic() {
        displayShuffleboard();
    }

    @Override
    public void autonomousInit() {
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopInit() {
        globalComponent.getButtonConfiguration().initTeleop();
        addCommand(new LimelightLedToggle(true));
    }

    @Override
    public void teleopPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledInit() {
        addCommand(new LimelightLedToggle(false));
    }

    @Override
    public void disabledPeriodic() {
    }

    private void displayShuffleboard() {
        SmartDashboard.putNumber("tv", VisionUtil.getLLValue("tv"));
        SmartDashboard.putNumber("tx", VisionUtil.getLLValue("tx"));
        SmartDashboard.putNumber("ty", VisionUtil.getLLValue("ty"));
    }

    private void addCommand(Command... commands) {
        CommandScheduler.getInstance().schedule(commands);
    }
}
