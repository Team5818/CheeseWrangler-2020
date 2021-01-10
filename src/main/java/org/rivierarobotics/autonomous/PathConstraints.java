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

//TODO implement
public class PathConstraints {
    private Double maxAccel = null;
    private Double maxVel = null;
    private Boolean absPath = null;
    private Boolean absHeading = null;

    public PathConstraints setMaxAccel(double maxAccel) {
        this.maxAccel = maxAccel;
        return this;
    }

    public PathConstraints setMaxVel(double maxVel) {
        this.maxVel = maxVel;
        return this;
    }

    public PathConstraints setAbsPath(boolean absPath) {
        this.absPath = absPath;
        return this;
    }

    public PathConstraints setAbsHeading(boolean absHeading) {
        this.absHeading = absHeading;
        return this;
    }

    public Double getMaxAccel() {
        return maxAccel;
    }

    public Double getMaxVel() {
        return maxVel;
    }

    public Boolean getAbsPath() {
        return absPath;
    }

    public Boolean getAbsHeading() {
        return absHeading;
    }
}
