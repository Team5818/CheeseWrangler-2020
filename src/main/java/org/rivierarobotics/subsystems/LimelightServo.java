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
import edu.wpi.first.wpiutil.math.MathUtil;

import javax.inject.Inject;

public class LimelightServo extends SubsystemBase {
    private static final double MAX_SERVO_ANGLE = 0.3;
    private static final double MIN_SERVO_ANGLE = 0;
    private static final int MAX_ANGLE = 80;
    private static final double ANGLE_PER_TICK = MAX_ANGLE / (MAX_SERVO_ANGLE - MIN_SERVO_ANGLE);
    private final Servo servo;

    @Inject
    public LimelightServo(int id) {
        this.servo = new Servo(id);
    }

    public double getAngle() {
        return (servo.getPosition() - MIN_SERVO_ANGLE) * ANGLE_PER_TICK;
    }

    public void setAngle(double angle) {
        angle = MathUtil.clamp(angle, 0, 80);
        this.servo.set((angle / ANGLE_PER_TICK) + MIN_SERVO_ANGLE);
    }

    public static double getMaxAngle() {
        return MAX_ANGLE;
    }
}
