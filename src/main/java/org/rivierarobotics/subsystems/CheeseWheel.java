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

public class CheeseWheel extends BasePIDSubsystem {
    private final WPI_TalonSRX wheelTalon;
    //TODO change baseTicks
    private int baseTicks = 0;
    private final double diff = 4096.0 / 5;
    private int currentIndex = 0;

    public CheeseWheel(int id) {
        super(0.0, 0.0, 0.0, 1.0);
        this.wheelTalon = new WPI_TalonSRX(id);
        wheelTalon.configFactoryDefault();
        wheelTalon.setNeutralMode(NeutralMode.Brake);
    }

    @Override
    public double getPositionTicks() {
        return wheelTalon.getSensorCollection().getPulseWidthPosition();
    }

    @Override
    protected void setPower(double pwr) {
        wheelTalon.set(pwr);
    }

    public void setToIndex(int index, boolean front) {
        super.setPositionTicks(front ? baseTicks + (index * diff) : baseTicks - (index * diff));
    }

    public void advanceCurrentIndex() {
        currentIndex += 1;
        currentIndex %= 5;
        setToIndex(currentIndex, true);
    }
}
