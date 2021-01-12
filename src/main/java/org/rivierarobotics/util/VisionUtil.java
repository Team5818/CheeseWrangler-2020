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

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;
import javax.inject.Inject;
import javax.inject.Singleton;

@Singleton
public class VisionUtil {
    private static final Scalar LOWER_COLOR_BOUNDS = new Scalar(27, 100, 6);
    private static final Scalar UPPER_COLOR_BOUNDS = new Scalar(64, 255, 255);
    private final NetworkTable limelight;

    @Inject
    public VisionUtil() {
        this.limelight = NetworkTableInstance.getDefault().getTable("limelight");
    }

    public double getLLValue(String key) {
        return getLLValue(key, -1);
    }

    public double getLLValue(String key, double def) {
        return limelight.getEntry(key).getDouble(def);
    }

    public double getActualTY(double hoodAbsPos) {
        return getLLValue("ty") + hoodAbsPos -  3;
    }

    public void setLEDState(LimelightLEDState state) {
        limelight.getEntry("ledMode").setNumber(state.ordinal());
    }
    
    public boolean hasLEDState(LimelightLEDState state) {
        return (int) limelight.getEntry("ledMode").getNumber(-1) == state.ordinal();
    }

    public void invertLedState() {
        NetworkTableEntry led = limelight.getEntry("ledMode");
        int cs = (int) led.getNumber(1.0);
        led.setNumber(cs == 1 ? 3 : 1);
    }

    public List<Point> findBallLocations(Mat img) {
        List<Point> out = new ArrayList<>();
        Mat matA = img.clone();
        Mat matB = new Mat();
        Imgproc.GaussianBlur(matA, matB, new Size(11, 11), 0);
        Imgproc.cvtColor(matB, matA, Imgproc.COLOR_BGR2HSV);
        Core.inRange(matA, LOWER_COLOR_BOUNDS, UPPER_COLOR_BOUNDS, matB);
        //TODO ensure this doesn't break b/c kernel=null
        Imgproc.erode(matB, matA, null, new Point(-1, -1), 2);
        Imgproc.dilate(matA, matB, null, new Point(-1, -1), 2);
        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(matB, contours, null, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        for (MatOfPoint contour : contours) {
            Point center = new Point();
            float[] radius = new float[1];
            Imgproc.minEnclosingCircle(new MatOfPoint2f(contour.toArray()), center, radius);
            out.add(center);
        }
        return out;
    }
}
