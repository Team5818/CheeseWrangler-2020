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

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.rivierarobotics.inject.Sided;
import org.rivierarobotics.util.Side;

import javax.inject.Inject;
import javax.inject.Singleton;

@Singleton
public class Intake extends SubsystemBase {
    private final IntakeSide front;
    private final IntakeSide back;

    @Inject
    public Intake(@Sided(Side.FRONT) IntakeSide front,
                  @Sided(Side.BACK) IntakeSide back) {
        this.front = front;
        this.back = back;
    }

    public void setPower(double frontPwr, double backPwr) {
        front.setPower(frontPwr);
        back.setPower(backPwr);
    }
}
