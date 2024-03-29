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

import com.ctre.phoenix.motorcontrol.can.BaseTalon;

/**
 * Manages multiple different PID configurations on a
 * single CTRE motor controller.
 *
 * <p>Most often used for separate position and velocity PID
 * constants though it has the capability to be used for
 * any number of configurations. Uses the {@link PIDConfig}
 * system to store PID modes/constant sets.</p>
 *
 * @see PIDConfig
 * @see MultiPID.Type
 */
public class MultiPID {
    private final BaseTalon motor;
    private final PIDConfig[] configs;
    private int currentIdx = 0;

    public MultiPID(BaseTalon motor, PIDConfig... configs) {
        this.motor = motor;
        this.configs = configs;
        applyAllConfigs();
        motor.selectProfileSlot(0, 0);
    }

    public PIDConfig getConfig(MultiPID.Type type) {
        return getConfig(type.ordinal());
    }

    public PIDConfig getConfig(int idx) {
        return configs[idx];
    }

    public void selectConfig(MultiPID.Type type) {
        selectConfig(type.ordinal());
    }

    /**
     * Selects the configuration stored at a certain index.
     *
     * @param idx the index to select at.
     */
    public void selectConfig(int idx) {
        if (currentIdx != idx) {
            motor.selectProfileSlot(idx, 0);
            currentIdx = idx;
        }
    }

    /**
     * Apply all configurations to the selected motor.
     */
    public void applyAllConfigs() {
        for (int i = 0; i < configs.length; i++) {
            configs[i].applyTo(motor, i);
        }
    }

    /**
     * Represents physics movement types (position, velocity, and acceleration)
     * for use in {@link MultiPID} managers.
     */
    public enum Type {
        POSITION, VELOCITY, ACCELERATION
    }
}
