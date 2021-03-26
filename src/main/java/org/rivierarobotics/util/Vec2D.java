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

package org.rivierarobotics.util;

public class Vec2D {
    protected final double x;
    protected final double y;

    public Vec2D(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public double getMagnitude() {
        return Math.sqrt(x * x + y * y);
    }

    public double getDirection() {
        return Math.atan2(y, x);
    }

    public double getAngle() {
        return Math.toDegrees(getDirection());
    }

    public double dot(Vec2D vec) {
        return x * vec.x + y * vec.y;
    }

    public double dist(Vec2D p1) {
        final double dx = p1.getX() - this.getX();
        final double dy = p1.getY() - this.getY();
        return Math.sqrt(dx * dx + dy * dy);
    }

    public Vec2D negate() {
        return new Vec2D(this.x * -1, this.y * -1);
    }

    public Vec2D addVec(Vec2D toAdd) {
        return new Vec2D(this.x + toAdd.getX(), this.y + toAdd.getY());
    }

    public Vec2D addNum(double toAdd) {
        return addNum(toAdd, toAdd);
    }

    public Vec2D addNum(double x, double y) {
        return new Vec2D(this.x + x, this.y + y);
    }

    public Vec2D multNum(double toMult) {
        return new Vec2D(this.x * toMult, this.y * toMult);
    }

    public Vec2D multVec(Vec2D toMult) {
        return new Vec2D(this.x * toMult.getX(), this.y * toMult.getX());
    }

    public static Vec2D createBlank() {
        return new Vec2D(0, 0);
    }

    @Override
    public String toString() {
        return "Vec2D{"
                + "x=" + x
                + ", y=" + y
                + '}';
    }
}
