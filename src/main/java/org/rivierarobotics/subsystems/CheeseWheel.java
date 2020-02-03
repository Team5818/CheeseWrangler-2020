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
    private static final int baseTicks = 0;
    private final WPI_TalonSRX wheelTalon;

    public CheeseWheel(int id) {
        super(0.0, 0.0, 0.0, 1.0);
        this.wheelTalon = new WPI_TalonSRX(id);
        wheelTalon.configFactoryDefault();
        wheelTalon.setNeutralMode(NeutralMode.Brake);
    }

    public void setIndex(CheeseWheelSlot index, boolean side) {
        super.setPositionTicks(index.frTicks);
    }

    @Override
    public double getPositionTicks() {
        return wheelTalon.getSensorCollection().getPulseWidthPosition();
    }

    @Override
    protected void setPower(double pwr) {
        wheelTalon.set(pwr);
    }

    public enum CheeseWheelSlot {
        ONE(1), TWO(2), THREE(3), FOUR(4), FIVE(5);

        public final int frTicks, brTicks;

        CheeseWheelSlot(int slotDiff) {
            int dTicks = (int) ((4096.0 / 5) * slotDiff);
            this.frTicks = baseTicks + dTicks;
            this.brTicks = baseTicks - dTicks;
        }
    }
}
