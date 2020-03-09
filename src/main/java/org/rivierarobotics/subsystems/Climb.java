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

public class Climb extends BasePIDSubsystem implements RRSubsystem {
    // private final WPI_TalonSRX climbTalon;

    public Climb(int id) {
        super(new PIDConfig(0, 0, 0, 0));
        //        this.climbTalon = new WPI_TalonSRX(id);
        //        climbTalon.configFactoryDefault();
        //        climbTalon.setSensorPhase(true);
        //        climbTalon.setNeutralMode(NeutralMode.Brake);
    }

    @Override
    public double getPositionTicks() {
        //        return climbTalon.getSensorCollection().getPulseWidthPosition();
        throw new UnsupportedOperationException();
    }

    @Override
    public void setPower(double pwr) {
        //        climbTalon.set(pwr);
    }

}
