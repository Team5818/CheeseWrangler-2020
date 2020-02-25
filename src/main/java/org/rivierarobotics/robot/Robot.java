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
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.rivierarobotics.autonomous.PathweaverExecutor;
import org.rivierarobotics.autonomous.Pose2dPath;
import org.rivierarobotics.inject.DaggerGlobalComponent;
import org.rivierarobotics.inject.GlobalComponent;
import org.rivierarobotics.subsystems.CheeseWheel;
import org.rivierarobotics.subsystems.DriveTrainSide;
import org.rivierarobotics.subsystems.Flywheel;
import org.rivierarobotics.subsystems.Hood;
import org.rivierarobotics.subsystems.LimelightServo;
import org.rivierarobotics.subsystems.Turret;
import org.rivierarobotics.util.LimelightLedState;
import org.rivierarobotics.util.NavXGyro;
import org.rivierarobotics.util.VisionUtil;

public class Robot extends TimedRobot {
    private GlobalComponent globalComponent;
    private Command autonomousCommand;
    private SendableChooser<Command> chooser;

    @Override
    public void robotInit() {
        globalComponent = DaggerGlobalComponent.create();
        globalComponent.robotInit();
        chooser = new SendableChooser<>();
        
        chooser.addOption("Flex", new PathweaverExecutor(globalComponent.getDriveTrain(), Pose2dPath.FLEX));
        chooser.addOption("CheeseRun", new PathweaverExecutor(globalComponent.getDriveTrain(), Pose2dPath.CHEESERUN));
    }

    @Override
    public void robotPeriodic() {
        displayShuffleboard();
        if (isEnabled()) {
            globalComponent.getPositionTracker().trackPosition();
        }
    }

    @Override
    public void autonomousInit() {
        autonomousCommand = chooser.getSelected();
        if (autonomousCommand != null) {
            autonomousCommand.schedule();
        }
    }

    @Override
    public void autonomousPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void teleopInit() {
        var commandComponent = globalComponent.getCommandComponentBuilder().build();
        CommandScheduler.getInstance().schedule(commandComponent.hood().alignQuadrature());
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }

        globalComponent.getDriveTrain().resetEncoder();
        globalComponent.getButtonConfiguration().initTeleop();
        globalComponent.getVisionUtil().setLedState(LimelightLedState.FORCE_ON);
        globalComponent.getNavXGyro().resetGyro();
        globalComponent.getCheeseWheel().setPositionTicks(globalComponent.getCheeseWheel().getIndexPosition(0));
    }

    @Override
    public void teleopPeriodic() {
        globalComponent.getPositionTracker().trackPosition();
        CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledInit() {
        globalComponent.getVisionUtil().setLedState(LimelightLedState.FORCE_OFF);
    }

    @Override
    public void disabledPeriodic() {
    }

    private void displayShuffleboard() {
        VisionUtil vision = globalComponent.getVisionUtil();
        NavXGyro navX = globalComponent.getNavXGyro();
        Turret tt = globalComponent.getTurret();
        CheeseWheel in = globalComponent.getCheeseWheel();
        Hood h = globalComponent.getHood();
        Flywheel fly = globalComponent.getFlywheel();
        LimelightServo servo = globalComponent.getLimelightServo();

        SmartDashboard.putNumber("tv", vision.getLLValue("tv"));
        SmartDashboard.putNumber("tx", vision.getLLValue("tx"));
        SmartDashboard.putNumber("ty", vision.getLLValue("ty"));
        SmartDashboard.putNumber("hoodPosition", h.getPositionTicks());
        SmartDashboard.putNumber("yaw", navX.getYaw());
        SmartDashboard.putNumber("AbsTurret", tt.getAbsoluteAngle());
        SmartDashboard.putNumber("TurretPosition", tt.getPositionTicks());
        SmartDashboard.putBoolean("Limit", h.isAtEnd());
        SmartDashboard.putNumber("HoodAngle", h.getAbsolutePosition());
        SmartDashboard.putBoolean("InState", in.getIntakeSensorState());
        SmartDashboard.putNumber("Flywheel Velocity", fly.getPositionTicks());
        SmartDashboard.putNumber("TurretPosTicks", tt.getPositionTicks());
        SmartDashboard.putNumber("TurretVelocity", tt.getVelocity());
        SmartDashboard.putNumber("TurretAbsAngle", tt.getAbsoluteAngle());
        SmartDashboard.putNumber("LLAngle", servo.getAngle());
        SmartDashboard.putData(chooser);
    }
}
