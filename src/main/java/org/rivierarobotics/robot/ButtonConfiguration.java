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
import org.rivierarobotics.util.Side;
import org.rivierarobotics.util.VisionTarget;

import javax.inject.Inject;

public class ButtonConfiguration {
    private final Joystick driverLeft;
    private final Joystick driverRight;
    private final Joystick coDriverLeft;
    private final Joystick coDriverRight;
    private final Joystick driverButtons;
    private final Joystick coDriverButtons;
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
        new JoystickButton(coDriverLeft, 1)
            .whenHeld(cmds.intake().setPower(Side.FRONT));
        new JoystickButton(coDriverLeft, 2)
            .whenHeld(cmds.intake().setPower(Side.BACK));
        new JoystickButton(coDriverRight, 1)
            .whenPressed(cmds.cheeseWheel().shootNWedges(VisionTarget.INNER, 1));
        new JoystickButton(coDriverRight, 2)
            .whenPressed(cmds.cheeseWheel().shootNWedges(VisionTarget.INNER, 5));
        new JoystickButton(coDriverButtons, 1)
            .whenPressed(cmds.cheeseWheel().moveToNextIndex(-1));
        new JoystickButton(coDriverButtons, 2)
            .whenPressed(cmds.cheeseWheel().moveToNextIndex(1));
        new JoystickButton(coDriverButtons, 3)
            .whenPressed(cmds.hood().setAngle(34.5));
        new JoystickButton(coDriverButtons, 4)
            .whenPressed(cmds.hood().setAngle(60));
        new JoystickButton(coDriverButtons, 11)
            .whileHeld(cmds.vision().visionAim(VisionTarget.INNER));
        new JoystickButton(coDriverButtons, 12)
            .whileHeld(cmds.vision().visionAim(VisionTarget.TOP));
    }
}
