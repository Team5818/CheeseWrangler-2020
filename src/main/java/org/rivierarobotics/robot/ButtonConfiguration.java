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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import org.rivierarobotics.autonomous.Pose2dPath;
import org.rivierarobotics.inject.CommandComponent;
import org.rivierarobotics.inject.Input;
import org.rivierarobotics.subsystems.CameraPosition;
import org.rivierarobotics.subsystems.CheeseWheel;
import org.rivierarobotics.util.CameraFlip;
import org.rivierarobotics.util.CheeseSlot;
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
        // Collecting
        new JoystickButton(coDriverLeft, 1)
                .whenHeld(cmds.collect().continuous(CheeseWheel.AngleOffset.COLLECT_FRONT)
                        .alongWith(cmds.camera().flipImage(CameraPosition.FRONT),
                        cmds.camera().setPosition(CameraPosition.FRONT)));
        new JoystickButton(coDriverLeft, 2)
                .whenHeld(cmds.collect().continuous(CheeseWheel.AngleOffset.COLLECT_BACK)
                        .alongWith(cmds.camera().flipImage(CameraPosition.BACK),
                        cmds.camera().setPosition(CameraPosition.BACK)));

        // Shooting
        new JoystickButton(coDriverRight, 1)
                .whenPressed(cmds.cheeseWheel().shootNWedges(1));
        new JoystickButton(coDriverRight, 2)
                .whileHeld(cmds.cheeseWheel().continuousShoot());

        // CheeseWheel manual cycles
        new JoystickButton(coDriverButtons, 2)
                .whenPressed(cmds.cheeseWheel().cycleSlot(CheeseWheel.Direction.BACKWARDS, CheeseWheel.AngleOffset.SHOOTER_FRONT, CheeseSlot.State.BALL));
        new JoystickButton(coDriverButtons, 3)
                .whenPressed(cmds.cheeseWheel().cycleSlot(CheeseWheel.Direction.BACKWARDS, CheeseWheel.AngleOffset.SHOOTER_FRONT, CheeseSlot.State.BALL));
        new JoystickButton(coDriverButtons, 5)
                .whenPressed(cmds.cheeseWheel().cycleSlot(CheeseWheel.Direction.FORWARDS, CheeseWheel.AngleOffset.COLLECT_BACK, CheeseSlot.State.EITHER));
        new JoystickButton(coDriverButtons, 6)
                .whenPressed(cmds.cheeseWheel().cycleSlot(CheeseWheel.Direction.FORWARDS, CheeseWheel.AngleOffset.COLLECT_FRONT, CheeseSlot.State.EITHER));

        // Autoaim
        new JoystickButton(coDriverButtons, 7)
                .whileHeld(cmds.vision().visionAim(VisionTarget.INNER));
        new JoystickButton(coDriverButtons, 8)
                .whileHeld(cmds.vision().visionAim(VisionTarget.TOP));
        new JoystickButton(coDriverButtons, 9)
                .toggleWhenPressed(cmds.vision().encoderAim(VisionTarget.INNER));
        new JoystickButton(coDriverButtons, 10)
                .toggleWhenPressed(cmds.vision().encoderAim(VisionTarget.TOP));
        new JoystickButton(coDriverButtons, 11)
                .whenPressed(cmds.vision().correctPosition());
        new JoystickButton(coDriverButtons, 12)
                .whenPressed(cmds.vision().toggleAutoAim());

        // Toggles
        new JoystickButton(coDriverButtons, 4)
                .toggleWhenPressed(cmds.flywheel().setVelocity());
        new JoystickButton(coDriverButtons, 1)
                .toggleWhenPressed(cmds.ejector().setPower(1.0));

        // Camera servo
        new JoystickButton(driverButtons, 11)
                .whenPressed(() -> CameraFlip.DO_FLIP = !CameraFlip.DO_FLIP);
        new JoystickButton(driverButtons, 12)
                .whenPressed(cmds.camera().setPosition(CameraPosition.FRONT)
                        .alongWith(cmds.camera().flipImage(CameraPosition.FRONT)));
        new JoystickButton(driverButtons, 10)
                .whenPressed(cmds.camera().setPosition(CameraPosition.BACK)
                        .alongWith(cmds.camera().flipImage(CameraPosition.BACK)));
        new JoystickButton(driverButtons, 8)
                .whenPressed(cmds.camera().setPosition(CameraPosition.CLIMB)
                        .alongWith(cmds.camera().flipImage(CameraPosition.CLIMB)));

        // Testing/dev
        new JoystickButton(driverButtons, 6)
                .whenPressed(cmds.flywheel().changeAutoAimSpeed(1));
        new JoystickButton(driverButtons, 5)
                .whenPressed(cmds.flywheel().changeAutoAimSpeed(-1));
        new JoystickButton(driverButtons, 9)
                .whenPressed(cmds.auto().pathweaver(Pose2dPath.STRAIGHT));
        new JoystickButton(driverButtons, 7)
                .whenPressed(cmds.auto().pathweaver(Pose2dPath.STRAIGHT, false, true));
        new JoystickButton(driverButtons, 3)
                .whileHeld(cmds.cheeseWheel().continuousShoot());
        new JoystickButton(driverButtons, 2)
                .whenPressed(cmds.cheeseWheel().all5Shoot());
    }
}
