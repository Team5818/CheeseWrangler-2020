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

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.rivierarobotics.inject.CommandComponent;
import org.rivierarobotics.inject.DaggerGlobalComponent;
import org.rivierarobotics.inject.GlobalComponent;
import org.rivierarobotics.subsystems.RobotShuffleboard;
import org.rivierarobotics.util.LimelightLedState;

import java.util.Objects;

public class Robot extends TimedRobot {
    private GlobalComponent globalComponent;
    private CommandComponent commandComponent;
    private Command autonomousCommand;
    private SendableChooser<Command> chooser;
    private Command flyingWheelman;
    private RobotShuffleboard shuffleboard;

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

        flyingWheelman = commandComponent.flywheel().setVelocity(15_900);
        initShuffleboard();
    }

    @Override
    public void robotPeriodic() {
        displayShuffleboard();
        if (isEnabled()) {
            if (!flyingWheelman.isScheduled()) {
                //flyingWheelman.schedule();
            }
            globalComponent.getVisionUtil().setLedState(LimelightLedState.FORCE_ON);
            globalComponent.getPositionTracker().trackPosition();
        }
    }

    @Override
    public void autonomousInit() {
        globalComponent.getNavXGyro().resetGyro();
        globalComponent.getNavXGyro().setAngleAdjustment(180);
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
        globalComponent.getLimelightServo().setAngle(45);
    }

    @Override
    public void teleopPeriodic() {
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

    private void initShuffleboard() {
    }

    private final ShuffleboardTab visionConfTab = Shuffleboard.getTab("Vision Conf");
    private final ShuffleboardTab driveTrainTab = Shuffleboard.getTab("Drive Train");
    private final ShuffleboardTab cheeseTab = Shuffleboard.getTab("Cheese Wheel");
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

    private final NetworkTableEntry frontSensor = driveTrainTab.add("Front Sensor", 0)
        .getEntry();
    private final NetworkTableEntry backSensor = driveTrainTab.add("Back Sensor", 0)
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

        var cw = globalComponent.getCheeseWheel();
        frontSensor.setNumber(cw.getFrontSensorValue());
        backSensor.setNumber(cw.getBackSensorValue());

        SmartDashboard.putData(chooser);
    }
}
