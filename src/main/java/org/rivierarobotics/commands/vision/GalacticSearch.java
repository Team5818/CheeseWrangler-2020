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

package org.rivierarobotics.commands.vision;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import net.octyl.aptcreator.GenerateCreator;
import net.octyl.aptcreator.Provided;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.rivierarobotics.autonomous.AutonomousCommands;
import org.rivierarobotics.autonomous.ChallengePath;
import org.rivierarobotics.autonomous.PathTracerExecutor;
import org.rivierarobotics.util.MathUtil;

import java.util.ArrayList;
import java.util.List;

@GenerateCreator
public class GalacticSearch extends CommandBase {
    private static final Scalar LOWER_COLOR_BOUNDS = new Scalar(32, 20, 120);
    private static final Scalar UPPER_COLOR_BOUNDS = new Scalar(50, 140, 255);
    private static final double[] GS_CONSTANTS = new double[] {
        0.75,   // Area for balls on left side, path A
        0.5,    // Area for balls on left side, path B
        3,      // Balls on left side, path A
        2,      // Balls on left side, path B
        5,      // Erode iterations
        8,      // Dilate iterations
        0,      // Minimum contour area
        1,      // Maximum contour tolerance from 1:1 aspect ratio
        0.5,    // Minimum proportion of bounding box filled by contour
        10,     // Minimum enclosing circle radius
        50      // Maximum enclosing circle radius
    };
    private final AutonomousCommands autonomousCommands;
    private final boolean isPathA;
    private PathTracerExecutor cmd;

    public GalacticSearch(@Provided AutonomousCommands autonomousCommands, boolean isPathA) {
        this.autonomousCommands = autonomousCommands;
        this.isPathA = isPathA;
    }

    @Override
    public void initialize() {
        Mat frame = new Mat();
        CameraServer.getInstance().getVideo("Flipped").grabFrame(frame);
        frame = frame.submat(new Rect(0, frame.height() / 2, frame.width(), frame.height()));
        List<Point> ballLocs = findBallLocations(frame);
        double leftArea =  GS_CONSTANTS[isPathA ? 0 : 1];
        int countLeft = 0;
        for (Point loc : ballLocs) {
            if (loc.x < frame.width() * leftArea) {
                countLeft++;
            }
        }
        int leftBalls = (int) GS_CONSTANTS[isPathA ? 2 : 3];
        String pathName = "GS_" + (isPathA ? "A" : "B") + "_" + (countLeft == leftBalls ? "RED" : "BLUE");
        cmd = autonomousCommands.challengePath(ChallengePath.valueOf(pathName));
        cmd.schedule();
    }

    @Override
    public boolean isFinished() {
        return cmd != null && cmd.isScheduled();
    }

    public List<Point> findBallLocations(Mat img) {
        List<Point> out = new ArrayList<>();
        Mat matA = img.clone();
        Mat matB = new Mat();

        Imgproc.GaussianBlur(matA, matB, new Size(11, 11), 0);
        Imgproc.cvtColor(matB, matA, Imgproc.COLOR_BGR2HSV);
        Core.inRange(matA, LOWER_COLOR_BOUNDS, UPPER_COLOR_BOUNDS, matB);
        Imgproc.erode(matB, matA, new Mat(), new Point(-1, -1), (int) GS_CONSTANTS[4]);
        Imgproc.dilate(matA, matB, new Mat(), new Point(-1, -1), (int) GS_CONSTANTS[5]);
        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(matB, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        for (MatOfPoint contour : contours) {
            double area = Imgproc.contourArea(contour);
            Rect bounds = Imgproc.boundingRect(contour);
            double aspectRatio = (double) bounds.width / bounds.height;
            double filledProp = area / (bounds.width * bounds.height);
            if (MathUtil.isWithinTolerance(aspectRatio, 1, GS_CONSTANTS[7])
                    && area > GS_CONSTANTS[6] && filledProp > GS_CONSTANTS[8]) {
                Point center = new Point();
                float[] radius = new float[1];
                Imgproc.minEnclosingCircle(new MatOfPoint2f(contour.toArray()), center, radius);
                if (radius[0] < GS_CONSTANTS[9] && radius[0] < GS_CONSTANTS[10]) {
                    out.add(center);
                }
            }
        }
        return out;
    }
}
