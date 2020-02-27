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
import org.rivierarobotics.autonomous.Pose2dPath;
import org.rivierarobotics.inject.CommandComponent;
import org.rivierarobotics.inject.DaggerGlobalComponent;
import org.rivierarobotics.inject.GlobalComponent;
import org.rivierarobotics.subsystems.CheeseWheel;
import org.rivierarobotics.subsystems.Hood;
import org.rivierarobotics.subsystems.LimelightServo;
import org.rivierarobotics.subsystems.Turret;
import org.rivierarobotics.util.LimelightLedState;
import org.rivierarobotics.util.NavXGyro;
import org.rivierarobotics.util.VisionUtil;

public class Robot extends TimedRobot {
    private GlobalComponent globalComponent;
    private CommandComponent commandComponent;
    private Command autonomousCommand;
    private SendableChooser<Command> chooser;

    @Override
    public void robotInit() {
        globalComponent = DaggerGlobalComponent.create();
        commandComponent = globalComponent.getCommandComponentBuilder().build();
        globalComponent.robotInit();
        chooser = new SendableChooser<>();

        //TODO not sure if this is valid syntax
        chooser.addOption("Flex", commandComponent.auto().pathweaver(Pose2dPath.FLEX));
        chooser.addOption("Cheeserun", commandComponent.auto().pathweaver(Pose2dPath.CHEESERUN));
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
            //TODO uncomment when auto is testing/implemented
            //autonomousCommand.schedule();
        }
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

        //CommandScheduler.getInstance().schedule(commandComponent.cheeseWheel().setPosition(0));
        globalComponent.getDriveTrain().resetEncoder();
        globalComponent.getButtonConfiguration().initTeleop();
        globalComponent.getVisionUtil().setLedState(LimelightLedState.FORCE_ON);
        globalComponent.getNavXGyro().resetGyro();
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
        LimelightServo servo = globalComponent.getLimelightServo();
        CheeseWheel cheeseWheel = globalComponent.getCheeseWheel();

        SmartDashboard.putNumber("index", cheeseWheel.getIndex());
        SmartDashboard.putNumber("tv", vision.getLLValue("tv"));
        SmartDashboard.putNumber("tx", vision.getLLValue("tx"));
        SmartDashboard.putNumber("ty", vision.getLLValue("ty"));
        SmartDashboard.putNumber("yaw", navX.getYaw());
        SmartDashboard.putNumber("wheelAngle", cheeseWheel.getPosition());
        SmartDashboard.putNumber("AbsTurret", tt.getAbsoluteAngle());
        SmartDashboard.putNumber("TurretPosition", tt.getPositionTicks());
        SmartDashboard.putBoolean("Limit", h.isAtEnd());
        SmartDashboard.putBoolean("InState", in.getSensors().getIntakeSensorStatus());
        SmartDashboard.putNumber("LLAngle", servo.getAngle());
        SmartDashboard.putNumber("CheeseWheel Pos", cheeseWheel.getPositionTicks());
        SmartDashboard.putData(chooser);
    }
}
