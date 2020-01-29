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

import dagger.Module;
import dagger.Provides;
import edu.wpi.first.wpilibj.Joystick;
import org.rivierarobotics.inject.Input;

import javax.inject.Singleton;

@Module
public class ControlsModule {
    private static final int DRIVER_LEFT_JS = 0;
    private static final int DRIVER_RIGHT_JS = 1;
    private static final int CODRIVER_LEFT_JS = 2;
    private static final int CODRIVER_RIGHT_JS = 3;

    private static final int DRIVER_BUTTONS = 4;
    private static final int CODRIVER_BUTTONS = 5;

    private ControlsModule() {
    }

    @Provides
    @Singleton
    @Input(Input.Position.DRIVER_LEFT)
    public static Joystick provideDriverJoystickLeft() {
        return new Joystick(DRIVER_LEFT_JS);
    }

    @Provides
    @Singleton
    @Input(Input.Position.DRIVER_RIGHT)
    public static Joystick provideDriverJoystickRight() {
        return new Joystick(DRIVER_RIGHT_JS);
    }

    @Provides
    @Singleton
    @Input(Input.Position.CODRIVER_LEFT)
    public static Joystick provideCoDriverJoystickLeft() {
        return new Joystick(CODRIVER_LEFT_JS);
    }

    @Provides
    @Singleton
    @Input(Input.Position.CODRIVER_RIGHT)
    public static Joystick provideCoDriverJoystickRight() {
        return new Joystick(CODRIVER_RIGHT_JS);
    }

    @Provides
    @Singleton
    @Input(Input.Position.DRIVER_BUTTONS)
    public static Joystick provideDriverButtons() {
        return new Joystick(DRIVER_BUTTONS);
    }

    @Provides
    @Singleton
    @Input(Input.Position.CODRIVER_BUTTONS)
    public static Joystick provideCoDriverButtons() {
        return new Joystick(CODRIVER_BUTTONS);
    }
}