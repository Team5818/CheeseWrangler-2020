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

package org.rivierarobotics.inject;

import dagger.Component;
import org.rivierarobotics.inject.CommandComponent.CCModule;
import org.rivierarobotics.robot.ButtonConfiguration;
import org.rivierarobotics.robot.ControlsModule;
import org.rivierarobotics.subsystems.*;
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
        getCheeseWheel();
        getPistonController();
        getPigeonGyro();
        getButtonConfiguration();
        getVisionUtil();
    }

    public abstract DriveTrain getDriveTrain();

    public abstract Turret getTurret();

    public abstract Hood getHood();

    public abstract Flywheel getFlywheel();

    public abstract CheeseWheel getCheeseWheel();

    public abstract PistonController getPistonController();

    public abstract PigeonGyro getPigeonGyro();

    public abstract ButtonConfiguration getButtonConfiguration();

    public abstract VisionUtil getVisionUtil();
}
