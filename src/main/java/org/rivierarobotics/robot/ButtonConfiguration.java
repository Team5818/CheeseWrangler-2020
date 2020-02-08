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

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import org.rivierarobotics.inject.CommandComponent;
import org.rivierarobotics.inject.Input;
import org.rivierarobotics.subsystems.DriveTrain;
import org.rivierarobotics.subsystems.LLServoPosition;

import javax.inject.Inject;

public class ButtonConfiguration {
    private final Joystick driverLeft, driverRight, coDriverLeft, coDriverRight, driverButtons, coDriverButtons;
    private final CommandComponent cmds;

    @Inject
    public ButtonConfiguration(@Input(Input.Selector.DRIVER_LEFT) Joystick driverLeft,
                               @Input(Input.Selector.DRIVER_RIGHT) Joystick driverRight,
                               @Input(Input.Selector.CODRIVER_LEFT) Joystick coDriverLeft,
                               @Input(Input.Selector.CODRIVER_RIGHT) Joystick coDriverRight,
                               @Input(Input.Selector.DRIVER_BUTTONS) Joystick driverButtons,
                               @Input(Input.Selector.CODRIVER_BUTTONS) Joystick coDriverButtons,
                               CommandComponent.Builder component) {
        this.driverLeft = driverLeft;
        this.driverRight = driverRight;
        this.coDriverLeft = coDriverLeft;
        this.coDriverRight = coDriverRight;
        this.driverButtons = driverButtons;
        this.coDriverButtons = coDriverButtons;
        this.cmds = component.build();
    }

    public void initTeleop() {
        new JoystickButton(coDriverRight, 1)
                .whenPressed(cmds.vision().autoAimTurret());
        new JoystickButton(coDriverRight, 2);
//                .whenPressed(cmds.auto().forwardBackRoutine());
        new JoystickButton(coDriverLeft, 1)
                .whenPressed(cmds.turret().setPosition(0));
        new JoystickButton(coDriverLeft, 2)
                .whenPressed(cmds.turret().setPosition(45));
        new JoystickButton(coDriverButtons, 12)
                .whenPressed(cmds.hood().alignQuadrature());

        // Competition Robot Button Map
        /*new JoystickButton(driverLeft, 1)
                .whenPressed(cmds.drive().changeGear(DriveTrain.Gear.LOW));
        new JoystickButton(driverLeft, 2)
                .whenPressed(cmds.drive().changeGear(DriveTrain.Gear.HIGH));
        new JoystickButton(driverRight, 1)
                .whenPressed(cmds.intake().setPower(1.0));
        new JoystickButton(driverRight, 2)
                .whenPressed(cmds.intake().setPower(-1.0));
        new JoystickButton(driverButtons, 6)
                .whenPressed(cmds.cheeseWheel().invertMode());

        new JoystickButton(coDriverButtons, 12)
                .whenPressed(cmds.climb().setPosition());
        new JoystickButton(coDriverButtons, 10)
                .whenPressed();
        new JoystickButton(coDriverButtons, 8)
                .whenPressed();
        new JoystickButton(coDriverButtons, 11)
                .whenPressed();
        new JoystickButton(coDriverButtons, 9)
                .whenPressed();
        new JoystickButton(coDriverLeft, 1)
                .whenPressed();
        new JoystickButton(coDriverLeft, 2)
                .whenPressed();
        new JoystickButton(coDriverRight, 1)
                .whenPressed(cmds.cheeseWheel().shootNext());
        new JoystickButton(coDriverRight, 2)
                .whenPressed(cmds.cheeseWheel().shootAll());
        new JoystickButton(coDriverButtons, 6)
                .whenPressed(cmds.cameraServo().setPosition(LLServoPosition.FRONT_COLLECT));
        new JoystickButton(coDriverButtons, 5)
                .whenPressed(cmds.cameraServo().setPosition(LLServoPosition.CLIMB));
        new JoystickButton(coDriverButtons, 4)
                .whenPressed(cmds.cameraServo().setPosition(LLServoPosition.FRONT_COLLECT));
        new JoystickButton(coDriverButtons, 3)
                .whenPressed();
        new JoystickButton(coDriverButtons, 2)
                .whenPressed(cmds.cheeseWheel().incrementIndex());
        new JoystickButton(coDriverButtons, 1)
                .whenPressed(cmds.cheeseWheel().decrementIndex());*/
    }
}