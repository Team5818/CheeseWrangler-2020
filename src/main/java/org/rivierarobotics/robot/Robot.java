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

import edu.wpi.cscore.VideoException;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.rivierarobotics.inject.CommandComponent;
import org.rivierarobotics.inject.DaggerGlobalComponent;
import org.rivierarobotics.inject.GlobalComponent;
import org.rivierarobotics.subsystems.CheeseWheel;
import org.rivierarobotics.subsystems.ColorWheel;
import org.rivierarobotics.subsystems.MotorTemp;
import org.rivierarobotics.util.CameraFlip;
import org.rivierarobotics.util.CheeseSlot;
import org.rivierarobotics.util.LimelightLEDState;
import org.rivierarobotics.util.RSTileOptions;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Objects;

/**
 * Main robot class for 2020/2021 robot CheeseWrangler.
 */
public class Robot extends TimedRobot {
    private GlobalComponent globalComponent;
    private CommandComponent commandComponent;
    private Command autonomousCommand;
    private SendableChooser<Command> chooser;
    private CameraFlip cameraThread;

    @Override
    public void robotInit() {
        // Dagger initialization
        globalComponent = DaggerGlobalComponent.create();
        globalComponent.robotInit();
        commandComponent = globalComponent.getCommandComponentBuilder().build();

        // Create autonomous options
        chooser = new SendableChooser<>();
        chooser.addOption("SixBallTrench", commandComponent.auto().sixBallTrench());
        chooser.addOption("ShootThreeBalls", commandComponent.auto().shootThreeBalls());
        chooser.addOption("EnemyTrench5Ball", commandComponent.auto().enemyTrench5Ball());
        chooser.addOption("OffsetSixBallTrench", commandComponent.auto().offsetSixBallTrench());
        chooser.addOption("CenterShoot", commandComponent.auto().centerShoot());

        // Secondary camera initialize
        CameraServer.getInstance().startAutomaticCapture();
        if (cameraThread == null) {
            cameraThread = new CameraFlip();
            cameraThread.start();
        }

        // Misc logging config
        try {
            globalComponent.getShuffleboard().getTab("Driver")
                    .setSendable(chooser, new RSTileOptions(2, 1, 0, 3))
                    .setCamera("Flipped", new RSTileOptions(3, 3, 0, 0))
                    .setCamera("limelight", new RSTileOptions(3, 3, 3, 0));
        } catch (VideoException ignored) {
            // Padding for checkstyle
        }
        globalComponent.getNavXGyro().resetGyro();
        DriverStation.getInstance().silenceJoystickConnectionWarning(true);
        globalComponent.getVisionUtil().setLEDState(LimelightLEDState.FORCE_ON);
    }

    @Override
    public void robotPeriodic() {
        //displayShuffleboard();
        var shuffleboard = globalComponent.getShuffleboard();
        var physics = globalComponent.getPhysicsUtil();
        var flywheel = globalComponent.getFlywheel();
        var ll = globalComponent.getVisionUtil();
        shuffleboard.getTab("Driver")
            .setEntry("AutoAim Speed", physics.getTargetVelocity(), new RSTileOptions(1, 1, 3, 3))
            .setEntry("AutoAim Mode", physics.getAimMode().name(), new RSTileOptions(1, 1, 4, 3))
            .setEntry("Flywheel Target Speed", flywheel.getTargetVel(), new RSTileOptions(1, 1, 5, 3))
            .setEntry("AA Enabled", physics.isAutoAimEnabled(), new RSTileOptions(1, 1, 6, 3))
            .setEntry("LL", ll.getLLValue("tx"));
        if (isEnabled()) {
            globalComponent.getVisionUtil().setLEDState(LimelightLEDState.FORCE_ON);
            globalComponent.getPositionTracker().trackPosition();
        }
    }

    @Override
    public void autonomousInit() {
        // Reset everything at autonomous begin
        globalComponent.getNavXGyro().resetGyro();
        globalComponent.getDriveTrain().resetOdometry();
        globalComponent.getPositionTracker().reset();
        // Choose autonomous path (or default)
        autonomousCommand = Objects.requireNonNullElseGet(
            chooser.getSelected(),
            () -> commandComponent.auto().shootThreeBalls()
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
            CommandScheduler.getInstance().cancel(autonomousCommand);
        }
        // Force flywheel to stop
        //commandComponent.flywheel().setPower(0).schedule();
        //commandComponent.vision().calcAim(VisionTarget.TOP);
        globalComponent.getButtonConfiguration().initTeleop();
    }

    @Override
    public void teleopPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledInit() {
        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().clearButtons();
    }

    @Override
    public void disabledPeriodic() {
        //globalComponent.getVisionUtil().setLEDState(LimelightLEDState.FORCE_OFF);
    }

    private void displayShuffleboard() {
        // Periodic logging to Shuffleboard
        var turret = globalComponent.getTurret();
        var hood = globalComponent.getHood();
        var gyro = globalComponent.getNavXGyro();
        var visionUtil = globalComponent.getVisionUtil();

        var flywheel = globalComponent.getFlywheel();
        var dt = globalComponent.getDriveTrain();
        var cw = globalComponent.getCheeseWheel();
        var physics = globalComponent.getPhysicsUtil();
        var climb = globalComponent.getClimb();
        var cow = globalComponent.getColorWheel();
        var shuffleboard = globalComponent.getShuffleboard();

        shuffleboard.getTab("TurretHood")
                .setEntry("Hood Pos Ticks", hood.getPositionTicks())
                .setEntry("Hood Abs Angle", hood.getAngle())
                .setEntry("Turret Abs Angle", turret.getAngle(true))
                .setEntry("Turret Rel Angle", turret.getAngle(false))
                .setEntry("Turret Pos Ticks", turret.getPositionTicks())
                .setEntry("turretVel", turret.getVelocity());

        shuffleboard.getTab("Vision")
                .setEntry("tx", visionUtil.getLLValue("tx"))
                .setEntry("ty", visionUtil.getLLValue("ty"))
                .setEntry("Adj. ty", visionUtil.getActualTY(hood.getAngle()))
                .setEntry("Flywheel Velocity", flywheel.getPositionTicks())
                .setEntry("Gyro Angle", gyro.getYaw())
                .setEntry("Adj tx", turret.getTurretCalculations(0, hood.getAngle())[1])
                .setEntry("Target Velocity", physics.getTargetVelocity());

        shuffleboard.getTab("Auto Aim")
                .setEntry("AutoAim Enabled", physics.isAutoAimEnabled());

        shuffleboard.getTab("Auto Aim").getTable("Random")
                .addTabData(shuffleboard.getTab("Cheese Wheel"));

        shuffleboard.getTab("Drive Train")
                .setEntry("Left Enc", dt.getLeft().getPosition())
                .setEntry("Right Enc", dt.getRight().getPosition())
                .setEntry("Left Vel", dt.getLeft().getVelocity())
                .setEntry("Right Vel", dt.getRight().getVelocity())
                .setEntry("XVel", dt.getXVelocity())
                .setEntry("YVel", dt.getYVelocity());

        shuffleboard.getTab("Cheese Wheel")
            .setEntry("Position Ticks", cw.getPositionTicks())
            .setEntry("Ball 0", CheeseSlot.ZERO.hasBall())
            .setEntry("Ball 1", CheeseSlot.ONE.hasBall())
            .setEntry("Ball 2", CheeseSlot.TWO.hasBall())
            .setEntry("Ball 3", CheeseSlot.THREE.hasBall())
            .setEntry("Ball 4", CheeseSlot.FOUR.hasBall())
            .setEntry("Front Index", cw.getIndex(CheeseWheel.AngleOffset.COLLECT_FRONT))
            .setEntry("Back Index", cw.getIndex(CheeseWheel.AngleOffset.COLLECT_BACK))
            .setEntry("OnIndex", cw.onSlot(CheeseWheel.AngleOffset.COLLECT_FRONT, 40))
            .setEntry("Shooter Index", cw.getIndex(CheeseWheel.AngleOffset.SHOOTER_FRONT))
            .setEntry("Shooter Ball", cw.getClosestSlot(CheeseWheel.AngleOffset.SHOOTER_BACK, CheeseWheel.Direction.BACKWARDS, CheeseSlot.State.BALL).ordinal());

        shuffleboard.getTab("Driver")
            .setEntry("AutoAim Speed", physics.getTargetVelocity(), new RSTileOptions(1, 1, 3, 3))
            .setEntry("AutoAim Mode", physics.getAimMode().name(), new RSTileOptions(1, 1, 4, 3))
            .setEntry("Flywheel Target Speed", flywheel.getTargetVel(), new RSTileOptions(1, 1, 5, 3))
            .setEntry("AA Enabled", physics.isAutoAimEnabled(), new RSTileOptions(1, 1, 6, 3));


        var sensorColor = cow.getSensorColor();
        shuffleboard.getTab("Climb")
            .setEntry("Limit Closed", climb.isAtBottom())
            .setEntry("Rel Pos", climb.getPositionTicks())
            .setEntry("Color R", sensorColor.red)
            .setEntry("Color G", sensorColor.green)
            .setEntry("Color B", sensorColor.blue)
            .setEntry("Match Color", cow.getGameColor().name())
            .setEntry("Target Color", ColorWheel.getFMSColor().name());

        List<MotorTemp> temps = new ArrayList<>();
        temps.add(turret.getTemp());
        temps.add(flywheel.getTemp());
        temps.addAll(Arrays.asList(dt.getLeft().getTemps()));
        temps.addAll(Arrays.asList(dt.getRight().getTemps()));
        var tempTab = shuffleboard.getTab("Motor Temps");
        for (MotorTemp temp : temps) {
            tempTab.setEntry(temp.getName(), "id: " + temp.getId() + " temp: " + temp.getValue());
        }
    }
}
