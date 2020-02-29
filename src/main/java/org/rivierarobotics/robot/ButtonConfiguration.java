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
import org.rivierarobotics.subsystems.CheeseWheel;
import org.rivierarobotics.subsystems.Climb;
import org.rivierarobotics.subsystems.DriveTrain;
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
        /*new JoystickButton(coDriverRight, 1)
            .toggleWhenPressed(cmds.cheeseWheel().shootNWedges(VisionTarget.INNER, 1));
        new JoystickButton(coDriverRight, 2)
            .toggleWhenPressed(cmds.cheeseWheel().shootNWedges(VisionTarget.INNER, 5));
        new JoystickButton(coDriverButtons, 12)
            .whenPressed(cmds.cheeseWheel().moveToNextIndex(1));
        new JoystickButton(coDriverButtons, 11)
            .whenPressed(cmds.cheeseWheel().moveToNextIndex(-1));
*/

        // Competition Robot Button Map
        new JoystickButton(driverLeft, 1)
            .whenPressed(cmds.drive().changeGear(DriveTrain.Gear.LOW));
        new JoystickButton(driverLeft, 2)
            .whenPressed(cmds.drive().changeGear(DriveTrain.Gear.HIGH));
        /*new JoystickButton(driverRight, 1)
                .whenPressed(cmds.intake().setPower(1.0));
        new JoystickButton(driverRight, 2)
                .whenPressed(cmds.intake().setPower(-1.0));
        new JoystickButton(driverButtons, 6)
                .whenPressed(cmds.cheeseWheel().invertMode());

        new JoystickButton(coDriverButtons, 12)
                .whenPressed(cmds.climb().setPosition(Climb.Height.FORTY_FIVE));
        new JoystickButton(coDriverButtons, 10)
                .whenPressed(cmds.climb().setPosition(Climb.Height.SIXTY));
        new JoystickButton(coDriverButtons, 8)
                .whenPressed(cmds.climb().setPosition(Climb.Height.SEVENTY_TWO));
        new JoystickButton(coDriverButtons, 11)
                .whenPressed();
        new JoystickButton(coDriverButtons, 9)
                .whenPressed(cmds.climb().lock());
        new JoystickButton(coDriverLeft, 1)
                .whenPressed();
        new JoystickButton(coDriverLeft, 2)
                .whenPressed();*/
        new JoystickButton(coDriverRight, 1)
                .whenPressed(cmds.cheeseWheel().shootNWedges(VisionTarget.INNER, 1));
        new JoystickButton(coDriverRight, 2)
                .whenPressed(cmds.cheeseWheel().shootNWedges(VisionTarget.INNER, 5));
        /*new JoystickButton(coDriverButtons, 6)
                .whenPressed(cmds.cameraServo().setPosition(LLServoPosition.FRONT_COLLECT));
        new JoystickButton(coDriverButtons, 5)
                .whenPressed(cmds.cameraServo().setPosition(LLServoPosition.CLIMB));
        new JoystickButton(coDriverButtons, 4)
                .whenPressed(cmds.cameraServo().setPosition(LLServoPosition.FRONT_COLLECT));
        new JoystickButton(coDriverButtons, 3)
                .whenPressed();*/
        new JoystickButton(coDriverButtons, 2)
                .whenPressed(cmds.cheeseWheel().moveToNextIndex(1));
        new JoystickButton(coDriverButtons, 1)
                .whenPressed(cmds.cheeseWheel().moveToNextIndex(-1));
    }
}
