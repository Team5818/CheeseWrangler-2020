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
import net.octyl.aptcreator.Provided;
import org.rivierarobotics.commands.CheeseWheelControl;
import org.rivierarobotics.commands.EjectorControl;
import org.rivierarobotics.commands.HoodControl;
import org.rivierarobotics.commands.TurretControl;
import org.rivierarobotics.inject.Sided;
import org.rivierarobotics.util.NavXGyro;
import org.rivierarobotics.util.VisionUtil;

import javax.inject.Provider;
import javax.inject.Singleton;

@Module
public class SubsystemModule {
    private static final int TURRET_TALON = 7;
    private static final int HOOD_TALON = 5;
    private static final int FLYWHEEL_TALON = 4;
    private static final int CHEESE_WHEEL_TALON = 6;
    private static final int EJECTOR_TALON = 8;
    private static final int INTAKE_FRONT_TALON = 9;
    private static final int INTAKE_BACK_TALON = 10;
    private static final int CLIMB_TALON = 21;

    private static final int HOOD_LIMIT_SWITCH = 6;
    private static final int INDEX_SENSOR_INTAKE = 9;
    private static final int INDEX_SENSOR_OUTPUT = 10;
    private static final int LIMELIGHT_SERVO = 1;

    private static final DriveTrainSide.MotorIds DRIVETRAIN_LEFT_MOTOR_IDS =
        new DriveTrainSide.MotorIds(
            1, 0, 3, 2,
            0, 1);
    private static final DriveTrainSide.MotorIds DRIVETRAIN_RIGHT_MOTOR_IDS =
        new DriveTrainSide.MotorIds(
            13, 12, 15, 14,
            2, 3);

    private SubsystemModule() {
    }

    @Provides
    @Singleton
    @Sided(Sided.Side.LEFT)
    public static DriveTrainSide provideDriveSideLeft() {
        return new DriveTrainSide(DRIVETRAIN_LEFT_MOTOR_IDS, false);
    }

    @Provides
    @Singleton
    @Sided(Sided.Side.RIGHT)
    public static DriveTrainSide provideDriveSideRight() {
        return new DriveTrainSide(DRIVETRAIN_RIGHT_MOTOR_IDS, true);
    }

    @Provides
    @Singleton
    public static Turret provideTurret(Provider<TurretControl> command, @Provided NavXGyro gyro, @Provided VisionUtil vision) {
        return new Turret(TURRET_TALON, command, gyro, vision);
    }

    @Provides
    @Singleton
    public static Hood provideHood(Provider<HoodControl> command) {
        return new Hood(HOOD_TALON, HOOD_LIMIT_SWITCH, command);
    }

    @Provides
    @Singleton
    public static Ejector provideEjector(Provider<EjectorControl> command) {
        return new Ejector(EJECTOR_TALON, command);
    }

    @Provides
    @Singleton
    public static Flywheel provideFlywheel() {
        return new Flywheel(FLYWHEEL_TALON);
    }


    @Provides
    @Singleton
    @Sided(Sided.Side.FRONT)
    public static IntakeSide provideIntakeSideFront() {
        return new IntakeSide(INTAKE_FRONT_TALON);
    }

    @Provides
    @Singleton
    @Sided(Sided.Side.BACK)
    public static IntakeSide provideIntakeSideBack() {
        return new IntakeSide(INTAKE_BACK_TALON);
    }

    @Provides
    @Singleton
    public static CheeseWheel provideCheeseWheel(Provider<CheeseWheelControl> command) {
        return new CheeseWheel(CHEESE_WHEEL_TALON, INDEX_SENSOR_INTAKE, INDEX_SENSOR_OUTPUT, command);
    }

    @Provides
    @Singleton
    public static PistonController providePistonController() {
        return new PistonController();
    }

    @Provides
    @Singleton
    public static LimelightServo provideLimelightServo() {
        return new LimelightServo(LIMELIGHT_SERVO);
    }

    @Provides
    @Singleton
    public static Climb provideClimb() {
        return new Climb(CLIMB_TALON);
    }
}
