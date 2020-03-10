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

package org.rivierarobotics.subsystems;

import dagger.Module;
import dagger.Provides;
import net.octyl.aptcreator.Provided;
import org.rivierarobotics.commands.cheesewheel.CheeseWheelControl;
import org.rivierarobotics.commands.hood.HoodControl;
import org.rivierarobotics.commands.turret.TurretControl;
import org.rivierarobotics.inject.Sided;
import org.rivierarobotics.util.NavXGyro;
import org.rivierarobotics.util.Side;
import org.rivierarobotics.util.VisionUtil;

import javax.inject.Provider;
import javax.inject.Singleton;

@Module
public class SubsystemModule {
    private static final int TURRET_TALON = 7;
    private static final int HOOD_TALON = 5;
    private static final int FLYWHEEL_FALCON = 4;
    private static final int CHEESE_WHEEL_TALON = 6;
    private static final int EJECTOR_VICTOR_LEFT = 8;
    private static final int EJECTOR_VICTOR_RIGHT = 11;
    private static final int INTAKE_VICTOR_FRONT = 9;
    private static final int INTAKE_VICTOR_BACK = 10;
    private static final int CLIMB_FALCON = 21;

    private static final int CAMERA_SERVO = 0;
    private static final int CW_FRONT_SENSOR = 1;
    private static final int CW_BACK_SENSOR = 0;

    private static final DTMotorIds DRIVETRAIN_LEFT_MOTOR_IDS =
        new DTMotorIds(1, 0, 0, 1);
    private static final DTMotorIds DRIVETRAIN_RIGHT_MOTOR_IDS =
        new DTMotorIds(13, 14, 2, 4);

    private SubsystemModule() {
    }

    @Provides
    @Singleton
    @Sided(Side.LEFT)
    public static DriveTrainSide provideDriveSideLeft() {
        return new DriveTrainSide(DRIVETRAIN_LEFT_MOTOR_IDS, true);
    }

    @Provides
    @Singleton
    @Sided(Side.RIGHT)
    public static DriveTrainSide provideDriveSideRight() {
        return new DriveTrainSide(DRIVETRAIN_RIGHT_MOTOR_IDS, false);
    }

    @Provides
    @Singleton
    public static Turret provideTurret(Provider<TurretControl> command, @Provided NavXGyro gyro, @Provided VisionUtil vision) {
        return new Turret(TURRET_TALON, command, gyro, vision);
    }

    @Provides
    @Singleton
    public static Hood provideHood(Provider<HoodControl> command) {
        return new Hood(HOOD_TALON, command);
    }

    @Provides
    @Singleton
    @Sided(Side.LEFT)
    public static EjectorSide provideEjectorLeft() {
        return new EjectorSide(EJECTOR_VICTOR_LEFT, false);
    }

    @Provides
    @Singleton
    @Sided(Side.RIGHT)
    public static EjectorSide provideEjectorRight() {
        return new EjectorSide(EJECTOR_VICTOR_RIGHT, false);
    }

    @Provides
    @Singleton
    public static Flywheel provideFlywheel() {
        return new Flywheel(FLYWHEEL_FALCON);
    }


    @Provides
    @Singleton
    @Sided(Side.FRONT)
    public static IntakeSide provideIntakeSideFront() {
        return new IntakeSide(INTAKE_VICTOR_FRONT, false);
    }

    @Provides
    @Singleton
    @Sided(Side.BACK)
    public static IntakeSide provideIntakeSideBack() {
        return new IntakeSide(INTAKE_VICTOR_BACK, true);
    }

    @Provides
    @Singleton
    public static CheeseWheel provideCheeseWheel(Provider<CheeseWheelControl> command) {
        return new CheeseWheel(CHEESE_WHEEL_TALON, CW_FRONT_SENSOR, CW_BACK_SENSOR, command);
    }

    @Provides
    @Singleton
    public static CameraServo provideCameraServo() {
        return new CameraServo(CAMERA_SERVO);
    }

    @Provides
    @Singleton
    public static Climb provideClimb() {
        return new Climb(CLIMB_FALCON);
    }
}
