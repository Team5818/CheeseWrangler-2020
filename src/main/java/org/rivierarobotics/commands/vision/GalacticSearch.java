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
import org.rivierarobotics.autonomous.PathTracerExecutor;
import org.rivierarobotics.autonomous.advanced.ChallengePath;
import org.rivierarobotics.util.MathUtil;
import org.rivierarobotics.util.Pair;

import java.util.ArrayList;
import java.util.List;

@GenerateCreator
public class GalacticSearch extends CommandBase {
    private static final Pair<Scalar> HSV_MASK_BOUNDS =
            new Pair<>(new Scalar(32, 20, 120), new Scalar(50, 140, 255));
    private static final Pair<Double> BALL_AREA_LEFT = new Pair<>(0.75, 0.5);
    private static final Pair<Integer> BALL_NUM_LEFT = new Pair<>(3, 2);
    private static final Pair<Integer> ENCL_CIRCLE_RADIUS = new Pair<>(10, 50);
    private static final double MIN_CONTOUR_AREA = 0;
    private static final double MAX_ASPECT_TOLERANCE = 1;
    private static final double MIN_BOUNDBOX_FILLED = 0.5;
    private static final int ERODE_ITERATIONS = 5;
    private static final int DILATE_ITERATIONS = 8;

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
        frame = frame.submat(new Rect(0, frame.height() / 2, frame.width(), frame.height() / 2));
        List<Point> ballLocs = findBallLocations(frame);
        double leftArea = isPathA ? BALL_AREA_LEFT.getA() : BALL_AREA_LEFT.getB();
        int countLeft = 0;
        for (Point loc : ballLocs) {
            if (loc.x < frame.width() * leftArea) {
                countLeft++;
            }
        }
        int leftBalls = isPathA ? BALL_NUM_LEFT.getA() : BALL_NUM_LEFT.getB();
        String pathName = "GS_" + (isPathA ? "A" : "B") + "_" + (countLeft == leftBalls ? "RED" : "BLUE");
        cmd = autonomousCommands.challengePath(ChallengePath.valueOf(pathName));
        cmd.schedule();
    }

    @Override
    public boolean isFinished() {
        return cmd != null && cmd.isScheduled();
    }

    private static List<Point> findBallLocations(Mat img) {
        List<Point> out = new ArrayList<>();
        Mat matA = img.clone();
        Mat matB = new Mat();

        Imgproc.GaussianBlur(matA, matB, new Size(11, 11), 0);
        Imgproc.cvtColor(matB, matA, Imgproc.COLOR_BGR2HSV);
        Core.inRange(matA, HSV_MASK_BOUNDS.getA(), HSV_MASK_BOUNDS.getB(), matB);
        Imgproc.erode(matB, matA, new Mat(), new Point(-1, -1), ERODE_ITERATIONS);
        Imgproc.dilate(matA, matB, new Mat(), new Point(-1, -1), DILATE_ITERATIONS);
        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(matB, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        for (MatOfPoint contour : contours) {
            double area = Imgproc.contourArea(contour);
            Rect bounds = Imgproc.boundingRect(contour);
            double aspectRatio = (double) bounds.width / bounds.height;
            double filledProp = area / (bounds.width * bounds.height);
            if (MathUtil.isWithinTolerance(aspectRatio, 1, MAX_ASPECT_TOLERANCE)
                    && area > MIN_CONTOUR_AREA && filledProp > MIN_BOUNDBOX_FILLED) {
                Point center = new Point();
                float[] radius = new float[1];
                Imgproc.minEnclosingCircle(new MatOfPoint2f(contour.toArray()), center, radius);
                if (radius[0] > ENCL_CIRCLE_RADIUS.getA() && radius[0] < ENCL_CIRCLE_RADIUS.getB()) {
                    out.add(center);
                }
            }
        }
        return out;
    }
}
