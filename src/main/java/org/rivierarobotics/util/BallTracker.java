package org.rivierarobotics.util;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

    @Inject
    public BallTracker(CheeseWheel cheeseWheel, Ejector ejector) {
        this.cheeseWheel = cheeseWheel;
        this.ejector = ejector;
    }

    public void checkIfEmpty() {

        if(!cheeseWheel.getPidController().atSetpoint()){
            return;
        }

            frontOnIndex = (cheeseWheel.getAdjustedAngle(cheeseWheel.getAngleOffset(CheeseWheel.AngleOffset.COLLECT_FRONT))) % 72 < 6
            || (cheeseWheel.getAdjustedAngle(cheeseWheel.getAngleOffset(CheeseWheel.AngleOffset.COLLECT_FRONT))) % 72 > 66;
        boolean backOnIndex = cheeseWheel.getAdjustedAngle(cheeseWheel.getAngleOffset(CheeseWheel.AngleOffset.COLLECT_BACK)) % 72 < 6
            || cheeseWheel.getAdjustedAngle(cheeseWheel.getAngleOffset(CheeseWheel.AngleOffset.COLLECT_BACK)) % 72 > 66;
        boolean shooterOnIndex = cheeseWheel.getAdjustedAngle(cheeseWheel.getAngleOffset(CheeseWheel.AngleOffset.SHOOTING)) % 72 < 6
            || cheeseWheel.getAdjustedAngle(cheeseWheel.getAngleOffset(CheeseWheel.AngleOffset.SHOOTING)) % 72 > 66;

        if (frontOnIndex) {
            if (!cheeseWheel.isFrontBallPresent()) {
                hasBall[cheeseWheel.getIndex(CheeseWheel.AngleOffset.COLLECT_FRONT)] = false;
            }
        }
        if (backOnIndex) {
            if (!cheeseWheel.isBackBallPresent()) {
                hasBall[cheeseWheel.getIndex(CheeseWheel.AngleOffset.COLLECT_BACK)] = false;
            }
        }
        if (shooterOnIndex) {
            if (ejector.isActive()) {
                hasBall[cheeseWheel.getIndex(CheeseWheel.AngleOffset.SHOOTING)] = false;
            }
        }
    }

    public boolean[] getBallArray() {
        return hasBall;
    }

    public void addBall(int index) {
        hasBall[index] = true;
    }

    public int getClosestIndex(CheeseWheel.AngleOffset mode, Direction direction, Boolean ball) {
        int startingIndex = cheeseWheel.getIndex(mode);
        int indexDiff = 5;
        int minDex = 0;
        int searchDirection;

        if(direction.equals(Direction.ANY)) {
            for(int i = 0; i < 5; i++) {
                if(hasBall[i] = ball && startingIndex - i < indexDiff) {
                        minDex = i;
                }
            }
            return minDex;
        }

        if(direction.equals(Direction.BACKWARD)) {
            searchDirection = -1;
        } else {
            searchDirection = 1;
        }

        int currentIndex = startingIndex;
        for(int i = 0; i < 5; i++) {
            if(currentIndex < 0) {
                currentIndex += 5;
            } else if(currentIndex > 5) {
                currentIndex -= 5;
            }

            if(hasBall[currentIndex] = ball) {
                return currentIndex;
            }

            currentIndex += searchDirection;
        }
        return 0;
    }

    public boolean isEmpty() {
        for(int i = 0; i < 5; i++) {
            if(hasBall[i]) {
                return false;
            }
        }
        return true;
    }

    public enum Direction {
        FORWARD, BACKWARD, ANY
    }

}