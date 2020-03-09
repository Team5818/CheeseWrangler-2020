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

package org.rivierarobotics.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.rivierarobotics.inject.CommandComponent;
import org.rivierarobotics.inject.DaggerGlobalComponent;
import org.rivierarobotics.inject.GlobalComponent;
import org.rivierarobotics.subsystems.CheeseWheel;
import org.rivierarobotics.util.LimelightLEDState;
import org.rivierarobotics.util.RobotShuffleboard;

import java.util.Objects;

public class Robot extends TimedRobot {
    private static RobotShuffleboard shuffleboard;
    private GlobalComponent globalComponent;
    private CommandComponent commandComponent;
    private Command autonomousCommand;
    private SendableChooser<Command> chooser;
    private Command flyingWheelman;

    @Override
    public void robotInit() {
        globalComponent = DaggerGlobalComponent.create();
        globalComponent.robotInit();
        commandComponent = globalComponent.getCommandComponentBuilder().build();

        chooser = new SendableChooser<>();
        chooser.addOption("AutoAiming 5 x 5", commandComponent.auto().forwardAuto(true));
        chooser.addOption("NoAiming 5 x 5", commandComponent.auto().forwardAuto(false));
        chooser.addOption("Shoot'n'drive", commandComponent.auto().shootAndDrive());
        chooser.addOption("Just Drive!", commandComponent.drive().driveDistance(-1, 0.25));

        shuffleboard = new RobotShuffleboard();
        flyingWheelman = commandComponent.flywheel().setVelocity(15_900);
        globalComponent.getNavXGyro().resetGyro();
    }

    @Override
    public void robotPeriodic() {
        displayShuffleboard();
        if (isEnabled()) {
            if (!flyingWheelman.isScheduled()) {
                flyingWheelman.schedule();
            }
            globalComponent.getVisionUtil().setLedState(LimelightLEDState.FORCE_ON);
            globalComponent.getPositionTracker().trackPosition();
        }
    }

    @Override
    public void autonomousInit() {
        globalComponent.getNavXGyro().resetGyro();
        globalComponent.getDriveTrain().resetEncoder();
        autonomousCommand = Objects.requireNonNullElseGet(
            chooser.getSelected(),
            () -> commandComponent.auto().shootAndDrive()
        );
        autonomousCommand.schedule();
    }

    @Override
    public void autonomousPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void teleopInit() {
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }

        globalComponent.getButtonConfiguration().initTeleop();
    }

    @Override
    public void teleopPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledInit() {
        globalComponent.getVisionUtil().setLedState(LimelightLEDState.FORCE_OFF);
    }

    @Override
    public void disabledPeriodic() {
        CommandScheduler.getInstance().run();
    }

    private void displayShuffleboard() {
        var turret = globalComponent.getTurret();
        var hood = globalComponent.getHood();
        var gyro = globalComponent.getNavXGyro();
        var visionUtil = globalComponent.getVisionUtil();
        var flywheel = globalComponent.getFlywheel();
        var dt = globalComponent.getDriveTrain();
        var cw = globalComponent.getCheeseWheel();

        shuffleboard.getTab("Turret/Hood")
            .setEntry("Hood Position Ticks", hood.getPositionTicks())
            .setEntry("Hood Absolute Angle", hood.getAbsoluteAngle())
            .setEntry("Turret Absolute Angle", turret.getAbsoluteAngle())
            .setEntry("Turret Position Ticks", turret.getPositionTicks());

        shuffleboard.getTab("Vision")
            .setEntry("tx", visionUtil.getLLValue("tx"))
            .setEntry("ty", visionUtil.getLLValue("ty"))
            .setEntry("Adj. ty", visionUtil.getActualTY())
            .setEntry("Flywheel Velocity", flywheel.getPositionTicks())
            .setEntry("Gyro Angle", gyro.getYaw());

        shuffleboard.getTab("Drive Train")
            .setEntry("Left Enc", dt.getLeft().getPosition())
            .setEntry("Right Enc", dt.getRight().getPosition())
            .setEntry("Left Vel", dt.getLeft().getVelocity())
            .setEntry("Right Vel", dt.getRight().getVelocity());

        shuffleboard.getTab("Cheese Wheel")
            .setEntry("Position Ticks", cw.getPositionTicks())
            .setEntry("Front Sensor", cw.getFrontSensorValue())
            .setEntry("Back Sensor", cw.getBackSensorValue());

        SmartDashboard.putData(chooser);
    }

    public static RobotShuffleboard getShuffleboard() {
        return shuffleboard;
    }
}
