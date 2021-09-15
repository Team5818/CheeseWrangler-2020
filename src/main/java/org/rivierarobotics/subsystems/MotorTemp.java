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

/**
 * Holds motor temperatures for any single motor on any subsystem.
 * Contains fields for motor id, temperature value (usually in
 * degrees celsius) and the name of the motor. Typically used for
 * appending to Shuffleboard as an entry.
 */
public class MotorTemp {
    private int id;
    private double value;
    private String name;

    public MotorTemp(int id, double value, String name) {
        this.id = id;
        this.value = value;
        this.name = name;
    }

    public MotorTemp(int id, String name) {
        this(id, -1, name);
    }

    public int getId() {
        return id;
    }

    public double getValue() {
        return value;
    }

    public String getName() {
        return name;
    }

    public MotorTemp setId(int id) {
        this.id = id;
        return this;
    }

    public MotorTemp setValue(double value) {
        this.value = value;
        return this;
    }

    public MotorTemp setName(String name) {
        this.name = name;
        return this;
    }
}
