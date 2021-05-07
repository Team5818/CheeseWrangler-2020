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

package org.rivierarobotics.autonomous;

/**
 * Generic class for <code>SplinePath</code> outputs. Contains a position,
 * velocity, and acceleration vector. Split into X and Y components and
 * stored as doubles. Designed for high-efficiency use.
 *
 * @see SplinePath
 */
public class SPOutput {
    private final double posX;
    private final double posY;
    private final double velX;
    private final double velY;
    private final double accelX;
    private final double accelY;

    public SPOutput(double posX, double posY, double velX, double velY, double accelX, double accelY) {
        this.posX = posX;
        this.posY = posY;
        this.velX = velX;
        this.velY = velY;
        this.accelX = accelX;
        this.accelY = accelY;
    }

    public double getPosX() {
        return posX;
    }

    public double getPosY() {
        return posY;
    }

    public double getVelX() {
        return velX;
    }

    public double getVelY() {
        return velY;
    }

    public double getAccelX() {
        return accelX;
    }

    public double getAccelY() {
        return accelY;
    }

    @Override
    public String toString() {
        return "SPOutput{posX=" + posX + ", posY=" + posY
                + ", velX=" + velX + ", velY=" + velY
                + ", accelX=" + accelX + ", accelY=" + accelY + '}';
    }
}
