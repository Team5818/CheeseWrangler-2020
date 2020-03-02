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

package org.rivierarobotics.inject;

import dagger.Component;
import org.rivierarobotics.inject.CommandComponent.CCModule;
import org.rivierarobotics.robot.ButtonConfiguration;
import org.rivierarobotics.robot.ControlsModule;
import org.rivierarobotics.subsystems.CWSensors;
import org.rivierarobotics.subsystems.CameraServo;
import org.rivierarobotics.subsystems.CheeseWheel;
import org.rivierarobotics.subsystems.Climb;
import org.rivierarobotics.subsystems.DriveTrain;
import org.rivierarobotics.subsystems.Ejector;
import org.rivierarobotics.subsystems.Flywheel;
import org.rivierarobotics.subsystems.Hood;
import org.rivierarobotics.subsystems.Intake;
import org.rivierarobotics.subsystems.LimelightServo;
import org.rivierarobotics.subsystems.PistonController;
import org.rivierarobotics.subsystems.SubsystemModule;
import org.rivierarobotics.subsystems.Turret;
import org.rivierarobotics.util.NavXGyro;
import org.rivierarobotics.util.PositionTracker;
import org.rivierarobotics.util.VisionUtil;

import javax.inject.Singleton;

@Component(modules = {SubsystemModule.class, ControlsModule.class, CCModule.class})
@Singleton
public abstract class GlobalComponent {
    public void robotInit() {
        getDriveTrain();
        getTurret();
        getHood();
        getFlywheel();
        getIntake();
        getEjector();
        getCheeseWheel();
        getClimb();
        getPistonController();
        getNavXGyro();
        getButtonConfiguration();
        getVisionUtil();
        getLimelightServo();
        getCameraServo();
        getPositionTracker();
        getCWSensors();
    }

    public abstract DriveTrain getDriveTrain();

    public abstract Turret getTurret();

    public abstract Hood getHood();

    public abstract Flywheel getFlywheel();

    public abstract Intake getIntake();

    public abstract Ejector getEjector();

    public abstract CheeseWheel getCheeseWheel();

    public abstract Climb getClimb();

    public abstract PistonController getPistonController();

    public abstract NavXGyro getNavXGyro();

    public abstract ButtonConfiguration getButtonConfiguration();

    public abstract VisionUtil getVisionUtil();

    public abstract LimelightServo getLimelightServo();

    public abstract CameraServo getCameraServo();

    public abstract CWSensors getCWSensors();

    public abstract PositionTracker getPositionTracker();

    public abstract CommandComponent.Builder getCommandComponentBuilder();

}
