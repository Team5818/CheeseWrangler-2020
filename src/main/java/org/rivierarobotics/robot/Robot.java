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

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.rivierarobotics.inject.CommandComponent;
import org.rivierarobotics.inject.DaggerGlobalComponent;
import org.rivierarobotics.inject.GlobalComponent;
import org.rivierarobotics.subsystems.CheeseWheel;
import org.rivierarobotics.util.BallTracker;
import org.rivierarobotics.util.LimelightLEDState;
import org.rivierarobotics.util.RobotShuffleboard;

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

//      flyingWheelman = commandComponent.flywheel().setVelocity(15_900);
        shuffleboard = new RobotShuffleboard("Vision Conf", "Drive Train", "Cheese Wheel");
    }

    @Override
    public void robotPeriodic() {
        displayShuffleboard();
        if (isEnabled()) {
//            if (!flyingWheelman.isScheduled()) {
//                flyingWheelman.schedule();
//            }
            globalComponent.getVisionUtil().setLedState(LimelightLEDState.FORCE_ON);
            globalComponent.getPositionTracker().trackPosition();
            globalComponent.getBallTracker().checkIfEmpty();
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
//        globalComponent.getLimelightServo().setAngle(45);
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
        final var turret = globalComponent.getTurret();
        final var hood = globalComponent.getHood();
        final var gyro = globalComponent.getNavXGyro();
        final var visionUtil = globalComponent.getVisionUtil();
        final var llServo = globalComponent.getLimelightServo();
        final var flywheel = globalComponent.getFlywheel();
        final var dt = globalComponent.getDriveTrain();
        var cw = globalComponent.getCheeseWheel();
        var bt = globalComponent.getBallTracker();
        boolean[] bta = globalComponent.getBallTracker().getBallArray();

        SmartDashboard.putNumber("CWAngle", cw.getAdjustedAngle(0));
        SmartDashboard.putNumber("cwtick", cw.getPositionTicks());
        SmartDashboard.putNumber("ShooterIndex", cw.getIndex(CheeseWheel.AngleOffset.SHOOTING));
        SmartDashboard.putNumber("CollectFrontIndex", cw.getIndex(CheeseWheel.AngleOffset.COLLECT_FRONT));
        SmartDashboard.putNumber("CollectBackIndex", cw.getIndex(CheeseWheel.AngleOffset.COLLECT_BACK));
        SmartDashboard.putBoolean("SensorFront", cw.isFrontBallPresent());
        SmartDashboard.putBoolean("SensorBack", cw.isBackBallPresent());
        SmartDashboard.putBoolean("CW AT POSITION?!??!", cw.getPidController().atSetpoint());
        SmartDashboard.putBoolean("index0", bta[0]);
        SmartDashboard.putBoolean("index1", bta[1]);
        SmartDashboard.putBoolean("index2", bta[2]);
        SmartDashboard.putBoolean("index3", bta[3]);
        SmartDashboard.putBoolean("index4", bta[4]);

        shuffleboard.getTab("Vision Conf")
            .setEntry("TurrTargetPos", turret.getAbsolutePosition())
            .setEntry("TurrAbsAngle", turret.getPositionTicks())
            .setEntry("TurrAbsTicks", turret.getAbsoluteAngle())
            .setEntry("HoodPosTicks", hood.getPositionTicks())
            .setEntry("HoodPosAngle", hood.getAbsolutePosition())
            .setEntry("Gyro Angle", gyro.getYaw())
            .setEntry("tx", visionUtil.getLLValue("tx"))
            .setEntry("ty", visionUtil.getLLValue("ty"))
            .setEntry("Adj. ty", visionUtil.getActualTY())
            .setEntry("Servo Angle", llServo.getAngle())
            .setEntry("Fly Vel", flywheel.getPositionTicks());
        shuffleboard.getTab("Drive Train")
            .setEntry("Left Enc", dt.getLeft().getPosition())
            .setEntry("Right Enc", dt.getRight().getPosition())
            .setEntry("Left Vel", dt.getLeft().getVelocity())
            .setEntry("Right Vel", dt.getRight().getVelocity());
        shuffleboard.getTab("Cheese Wheel")
            .setEntry("Front Sensor", cw.getFrontSensorValue());

        SmartDashboard.putData(chooser);
    }
}
