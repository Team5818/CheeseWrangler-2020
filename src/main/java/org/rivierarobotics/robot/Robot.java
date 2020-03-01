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

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.rivierarobotics.autonomous.Pose2dPath;
import org.rivierarobotics.inject.CommandComponent;
import org.rivierarobotics.inject.DaggerGlobalComponent;
import org.rivierarobotics.inject.GlobalComponent;
import org.rivierarobotics.util.LimelightLedState;

import java.util.Objects;

public class Robot extends TimedRobot {
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
//        UsbCamera driverCamera = CameraServer.getInstance().startAutomaticCapture();
//        driverCamera.setVideoMode(VideoMode.PixelFormat.kMJPEG, 144, 108, 60);
        chooser = new SendableChooser<>();

        //chooser.addOption("Flex", commandComponent.auto().pathweaver(Pose2dPath.FLEX));
        //chooser.addOption("Cheeserun", commandComponent.auto().pathweaver(Pose2dPath.CHEESERUN));
        chooser.addOption("4 x 4", commandComponent.auto().forwardAuto());
        flyingWheelman = commandComponent.flywheel().setVelocity(15_900);
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
        autonomousCommand = Objects.requireNonNullElseGet(
            chooser.getSelected(),
            commandComponent.auto()::forwardAuto
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

        globalComponent.getDriveTrain().resetEncoder();
        globalComponent.getButtonConfiguration().initTeleop();
        globalComponent.getVisionUtil().setLedState(LimelightLedState.FORCE_ON);
        globalComponent.getNavXGyro().resetGyro();
        globalComponent.getNavXGyro().setAngleAdjustment(180);
        globalComponent.getLimelightServo().setAngle(70);
    }

    @Override
    public void teleopPeriodic() {
        if (!flyingWheelman.isScheduled()) {
            flyingWheelman.schedule();
        }
        CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledInit() {
        globalComponent.getVisionUtil().setLedState(LimelightLedState.FORCE_OFF);
    }

    @Override
    public void disabledPeriodic() {
        CommandScheduler.getInstance().run();
    }

    private final ShuffleboardTab visionConfTab = Shuffleboard.getTab("Vision Conf");
    private final ShuffleboardTab driveTrainTab = Shuffleboard.getTab("Drive Train");
    private final NetworkTableEntry turretTargetPos = visionConfTab.add("TurrTargetPos", 0)
        .getEntry();
    private final NetworkTableEntry turretPositionTicks = visionConfTab.add("TurretPosTicks", 0)
        .getEntry();
    private final NetworkTableEntry turretAbsoluteAngle = visionConfTab.add("TurretAbsAngle", 0)
        .getEntry();
    private final NetworkTableEntry hoodPositionTicks = visionConfTab.add("HoodPosTicks", 0)
        .getEntry();
    private final NetworkTableEntry hoodAbsoluteAngle = visionConfTab.add("HoodAbsAngle", 0)
        .getEntry();
    private final NetworkTableEntry gyroYaw = visionConfTab.add("Gyro Yaw", 0)
        .getEntry();
    private final NetworkTableEntry ty = visionConfTab.add("ty", 0)
        .getEntry();
    private final NetworkTableEntry tx = visionConfTab.add("tx", 0)
        .getEntry();
    private final NetworkTableEntry adjustedTy = visionConfTab.add("Adj. ty", 0)
        .getEntry();
    private final NetworkTableEntry servoAngle = visionConfTab.add("Servo Angle", 0)
        .getEntry();
    private final NetworkTableEntry flyVel = visionConfTab.add("Fly Vel", 0)
        .getEntry();

    private final NetworkTableEntry leftEnc = driveTrainTab.add("Left Enc", 0)
        .getEntry();
    private final NetworkTableEntry rightEnc = driveTrainTab.add("Right Enc", 0)
        .getEntry();

    private final NetworkTableEntry leftVel = driveTrainTab.add("Left Vel", 0)
        .getEntry();
    private final NetworkTableEntry rightVel = driveTrainTab.add("Right Vel", 0)
        .getEntry();

    private void displayShuffleboard() {
        var turret = globalComponent.getTurret();
        turretTargetPos.setNumber(turret.getAbsolutePosition());
        turretPositionTicks.setNumber(turret.getPositionTicks());
        turretAbsoluteAngle.setNumber(turret.getAbsoluteAngle());
        SmartDashboard.putNumber("TurrAbsAng", turret.getAbsoluteAngle());

        var hood = globalComponent.getHood();
        hoodPositionTicks.setNumber(hood.getPositionTicks());
        hoodAbsoluteAngle.setNumber(hood.getAbsolutePosition());

        var gyro = globalComponent.getNavXGyro();
        gyroYaw.setNumber(gyro.getYaw());

        var visionUtil = globalComponent.getVisionUtil();
        ty.setNumber(visionUtil.getLLValue("ty"));
        tx.setNumber(visionUtil.getLLValue("tx"));
        adjustedTy.setNumber(visionUtil.getActualTY());

        var llServo = globalComponent.getLimelightServo();
        servoAngle.setNumber(llServo.getAngle());

        var flywheel = globalComponent.getFlywheel();
        flyVel.setNumber(flywheel.getPositionTicks());

        var dt = globalComponent.getDriveTrain();
        leftEnc.setNumber(dt.getLeft().getPosition());
        rightEnc.setNumber(dt.getRight().getPosition());
        leftVel.setNumber(dt.getLeft().getVelocity());
        rightVel.setNumber(dt.getLeft().getVelocity());

        SmartDashboard.putData(chooser);
    }
}
