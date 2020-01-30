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

package org.rivierarobotics.subsystems;

import dagger.Module;
import dagger.Provides;
import org.rivierarobotics.commands.TurretControl;
import org.rivierarobotics.inject.Sided;
import org.rivierarobotics.util.VisionUtil;

import javax.inject.Provider;
import javax.inject.Singleton;

@Module
public class SubsystemModule {
    private static final int TURRET_TALON = 7;
    private static final int HOOD_TALON = 11;
    private static final int FLYWHEEL_TALON = 13;
    private static final int CHEESE_WHEEL_TALON = 10;
    private static final DriveTrainSide.MotorIds DRIVETRAIN_LEFT_MOTOR_IDS = new DriveTrainSide.MotorIds(1, 2, 3),
            DRIVETRAIN_RIGHT_MOTOR_IDS = new DriveTrainSide.MotorIds(4, 5, 6);

    private SubsystemModule() {
    }

    @Provides
    @Singleton
    @Sided(Sided.Side.LEFT)
    public static DriveTrainSide provideDriveSideLeft() {
        return new DriveTrainSide(DRIVETRAIN_LEFT_MOTOR_IDS, true);
    }

    @Provides
    @Singleton
    @Sided(Sided.Side.RIGHT)
    public static DriveTrainSide provideDriveSideRight() {
        return new DriveTrainSide(DRIVETRAIN_RIGHT_MOTOR_IDS, false);
    }

    @Provides
    @Singleton
    public static Turret provideTurret(Provider<TurretControl> command) {
        return new Turret(TURRET_TALON, command);
    }

    @Provides
    @Singleton
    public static Hood provideHood() {
        return new Hood(HOOD_TALON);
    }

    @Provides
    @Singleton
    public static Flywheel provideFlywheel() {
        return new Flywheel(FLYWHEEL_TALON);
    }

    @Provides
    @Singleton
    public static CheeseWheel provideCheeseWheel() {
        return new CheeseWheel(CHEESE_WHEEL_TALON);
    }

    @Provides
    @Singleton
    public static VisionUtil provideVisionUtil() {
        return new VisionUtil();
    }
}
