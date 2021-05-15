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

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import javax.inject.Inject;

public class CameraServo extends SubsystemBase {
    private final Servo servo;

    @Inject
    public CameraServo(int id) {
        this.servo = new Servo(id);
    }

    public void setValue(double value) {
        servo.set(value);
    }

    /**
     * Set the angle of the camera servo in degrees. Has a range of 0 (front)
     * to 180 (back). Modify by changing mounting bracket.
     *
     * @param degrees the set position of the servo in degrees.
     */
    public void setAngle(double degrees) {
        double range = CameraPosition.BACK.getServoValue() - CameraPosition.FRONT.getServoValue();
        servo.set(CameraPosition.FRONT.getServoValue() + ((degrees / 180) * range));
    }
}
