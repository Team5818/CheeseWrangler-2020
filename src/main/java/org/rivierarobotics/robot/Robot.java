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
import org.rivierarobotics.util.RobotMap;

/**
 * The main class representing the Robot
 * Has no function code, but as an exception stores single instances of major mechanisms/subsystems
 * and control systems as instance variables, accessible statically though the runningRobot Robot object
 */
public class Robot extends TimedRobot {
    public static Robot runningRobot;
    public final DriveTrain driveTrain;
    public final Joystick leftJs, rightJs, buttons;
    public boolean isArcade = true;

    /**
     * Initializes DriveTrain (all subsystems) and joysticks
     * Instances actually created, done in constructor instead
     * of RobotInit because the constructor always runs first
     * Note that runningRobot is set to "this", aka the current instance
     */
    public Robot() {
        this.driveTrain = new DriveTrain();
        this.leftJs = new Joystick(RobotMap.LEFT_JS);
        this.rightJs = new Joystick(RobotMap.RIGHT_JS);
        this.buttons = new Joystick(RobotMap.BUTTONS);
        runningRobot = this;
    }

    /**
     * Runs once every time the robot initializes. It's a good idea to put button
     * initialization calls here or other things that need subsystems but should be
     * initialized before the robot starts.
     */
    @Override
    public void robotInit() {
        ButtonConfiguration.init();
    }

    // Usually there would be autonomousInit() and autonomousPeriodic() methods,
    // but for this use case we don't need to use them because we have no auto YET.

    /**
     * Runs once every time the robot goes into teleop mode and is enabled.
     */
    @Override
    public void teleopInit() {

    }

    /**
     * Runs very often on a loop while the robot is enabled and in teleop mode.
     * Very important for the CommandScheduler, which runs commands through this method
     */
    @Override
    public void teleopPeriodic() {
        CommandScheduler.getInstance().run();
    }

    // There is the option for disabledInit() and disabledPeriodic(), but
    // again it wasn't useful for the purpose of this project.
}
