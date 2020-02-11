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

package org.rivierarobotics.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class Flywheel extends BasePIDSubsystem {
    private final WPI_TalonSRX flywheelTalon;
    private double position;

    public Flywheel(int id) {
        //TODO ryan help
        super(0.02, 0.0, 0.0, 1.0);
        flywheelTalon = new WPI_TalonSRX(id);
        flywheelTalon.configFactoryDefault();
        flywheelTalon.setInverted(true);
        flywheelTalon.setNeutralMode(NeutralMode.Coast);
    }

    public boolean readyToShoot() {
        if (Math.abs(getPositionTicks() - position) <= 5) {
            return true;
        }
        else {
            return false;
        }
    }

    @Override
    public void setPositionTicks(double pos) {
        this.position = pos;
        super.setPositionTicks(pos);
    }

    @Override
    public double getPositionTicks() {
        return flywheelTalon.getSensorCollection().getQuadratureVelocity();
    }

    @Override
    public void setPower(double pwr) {
        flywheelTalon.set(pwr);
    }
}
