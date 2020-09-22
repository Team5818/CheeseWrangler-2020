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

package org.rivierarobotics.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import org.rivierarobotics.inject.CommandComponent;
import org.rivierarobotics.inject.Input;
import org.rivierarobotics.subsystems.CheeseWheel;
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
                .whenHeld(cmds.collect().continuous(CheeseWheel.AngleOffset.COLLECT_FRONT));
        new JoystickButton(coDriverLeft, 2)
                .whenHeld(cmds.collect().continuous(CheeseWheel.AngleOffset.COLLECT_BACK));

        new JoystickButton(coDriverRight, 1)
                .whenPressed(cmds.cheeseWheel().shootNWedges(1));
        new JoystickButton(coDriverRight, 2)
                .whenPressed(cmds.cheeseWheel().shootNWedges(5));

        new JoystickButton(coDriverButtons, 2)
                .whenPressed(cmds.cheeseWheel().cycleSlot(CheeseWheel.Direction.BACKWARDS, CheeseWheel.AngleOffset.COLLECT_FRONT, true), true);
        new JoystickButton(coDriverButtons, 3)
                .whenPressed(cmds.cheeseWheel().cycleSlot(CheeseWheel.Direction.BACKWARDS, CheeseWheel.AngleOffset.COLLECT_FRONT, false), true);
        new JoystickButton(coDriverButtons, 5)
                .whenPressed(cmds.cheeseWheel().cycleSlot(CheeseWheel.Direction.FORWARDS, CheeseWheel.AngleOffset.COLLECT_FRONT, true), true);
        new JoystickButton(coDriverButtons, 6)
                .whenPressed(cmds.cheeseWheel().cycleSlot(CheeseWheel.Direction.FORWARDS, CheeseWheel.AngleOffset.COLLECT_FRONT, false), true);
        new JoystickButton(coDriverButtons, 7)
                .whileHeld(cmds.vision().visionAim(VisionTarget.INNER));
        new JoystickButton(coDriverButtons, 8)
                .whileHeld(cmds.vision().visionAim(VisionTarget.TOP));
        // new JoystickButton(coDriverButtons, 9)
        //         .whileHeld(cmds.vision().encoderAim(VisionTarget.INNER));
        // new JoystickButton(coDriverButtons, 10)
        //         .whileHeld(cmds.vision().encoderAim(VisionTarget.TOP));
        new JoystickButton(coDriverButtons, 11)
                .whenPressed(cmds.hood().toggleTrenchMode());
        // new JoystickButton(coDriverButtons, 12)
        //         .whenPressed(cmds.vision().toggleAutoAim());

        new JoystickButton(driverButtons, 6)
                .whenPressed(cmds.vision().correctPosition());
    }
}
