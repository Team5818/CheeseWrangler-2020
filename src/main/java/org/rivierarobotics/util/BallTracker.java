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

import org.rivierarobotics.subsystems.CheeseWheel;
import org.rivierarobotics.subsystems.Ejector;

import javax.inject.Inject;
import javax.inject.Singleton;

@Singleton
public class BallTracker {
    private final CheeseWheel cheeseWheel;
    private final Ejector ejector;
    private boolean[] hasBall = {true, true, true, true, true};
    public boolean frontOnIndex;
    public boolean backOnIndex;

    @Inject
    public BallTracker(CheeseWheel cheeseWheel, Ejector ejector) {
        this.cheeseWheel = cheeseWheel;
        this.ejector = ejector;
    }

    public void checkIfEmpty() {
        if (!cheeseWheel.getPidController().atSetpoint()) {
            return;
        }

        frontOnIndex = getOnIndex(CheeseWheel.AngleOffset.COLLECT_FRONT);
        backOnIndex = getOnIndex(CheeseWheel.AngleOffset.COLLECT_BACK);
        boolean shooterOnIndex = getOnIndex(CheeseWheel.AngleOffset.SHOOTING);

        if (frontOnIndex && !cheeseWheel.isFrontBallPresent()) {
            hasBall[cheeseWheel.getIndex(CheeseWheel.AngleOffset.COLLECT_FRONT)] = false;
        }
        if (backOnIndex && !cheeseWheel.isBackBallPresent()) {
            hasBall[cheeseWheel.getIndex(CheeseWheel.AngleOffset.COLLECT_BACK)] = false;
        }
        if (shooterOnIndex && ejector.isActive()) {
            hasBall[cheeseWheel.getIndex(CheeseWheel.AngleOffset.SHOOTING)] = false;
        }
    }

    public boolean getOnIndex(CheeseWheel.AngleOffset offset) {
        double adjIndex = cheeseWheel.getAdjustedAngle(cheeseWheel.getAngleOffset(offset)) % 72;
        return adjIndex < 6 || adjIndex > 66;
    }

    public boolean[] getBallArray() {
        return hasBall;
    }

    public void addBall(int index) {
        hasBall[index] = true;
    }

    public int getClosestIndex(CheeseWheel.AngleOffset mode, Direction direction, boolean ball) {
        int startingIndex = cheeseWheel.getIndex(mode);
        int indexDiff = 5;
        int minDex = 0;
        int searchDirection;

        switch (direction) {
            case ANY:
                for (int i = 0; i < 5; i++) {
                    if (hasBall[i] == ball && startingIndex - i < indexDiff) {
                        minDex = i;
                    }
                }
                return minDex;
            case BACKWARD:
                searchDirection = -1;
                break;
            default:
                searchDirection = 1;
                break;
        }

        int currentIndex = startingIndex;
        for (int i = 0; i < 5; i++) {
            if (currentIndex < 0) {
                currentIndex += 5;
            } else if (currentIndex > 5) {
                currentIndex -= 5;
            }

            if (hasBall[currentIndex] == ball) {
                return currentIndex;
            }

            currentIndex += searchDirection;
        }
        return 0;
    }

    public boolean isEmpty() {
        for (int i = 0; i < 5; i++) {
            if (hasBall[i]) {
                return false;
            }
        }
        return true;
    }

    public enum Direction {
        FORWARD, BACKWARD, ANY
    }
}