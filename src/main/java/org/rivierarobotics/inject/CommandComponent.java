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

import dagger.Module;
import dagger.Subcomponent;
import org.rivierarobotics.autonomous.AutonomousCommands;
import org.rivierarobotics.commands.CameraServoCommands;
import org.rivierarobotics.commands.CheeseWheelCommands;
import org.rivierarobotics.commands.ClimbCommands;
import org.rivierarobotics.commands.CollectionCommands;
import org.rivierarobotics.commands.DriveCommands;
import org.rivierarobotics.commands.EjectorCommands;
import org.rivierarobotics.commands.FlywheelCommands;
import org.rivierarobotics.commands.HoodCommands;
import org.rivierarobotics.commands.LimelightServoCommands;
import org.rivierarobotics.commands.PistonCommands;
import org.rivierarobotics.commands.TurretCommands;
import org.rivierarobotics.commands.VisionCommands;

@Subcomponent
public abstract class CommandComponent {
    public abstract DriveCommands drive();

    public abstract PistonCommands piston();

    public abstract HoodCommands hood();

    public abstract TurretCommands turret();

    public abstract FlywheelCommands flywheel();

    public abstract CollectionCommands collect();

    public abstract EjectorCommands ejector();

    public abstract VisionCommands vision();

    public abstract CheeseWheelCommands cheeseWheel();

    public abstract AutonomousCommands auto();

    public abstract LimelightServoCommands limelightServo();

    public abstract CameraServoCommands cameraServo();

    public abstract ClimbCommands climb();

    @Module(subcomponents = CommandComponent.class)
    public interface CCModule {
    }

    @Subcomponent.Builder
    public interface Builder {
        CommandComponent build();
    }
}
