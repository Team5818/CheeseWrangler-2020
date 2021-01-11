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
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.rivierarobotics.autonomous.AutonomousCommands;
import org.rivierarobotics.autonomous.PathTracerExecutor;

import java.util.ArrayList;
import java.util.List;
import javax.inject.Inject;

@GenerateCreator
public class GalacticSearch extends CommandBase {
    private static final Scalar LOWER_COLOR_BOUNDS = new Scalar(27, 100, 6);
    private static final Scalar UPPER_COLOR_BOUNDS = new Scalar(64, 255, 255);
    private static final double LEFT_BALL_AREA = 0.5;
    private final AutonomousCommands autonomousCommands;
    private PathTracerExecutor cmd;

    public GalacticSearch(@Provided AutonomousCommands autonomousCommands) {
        this.autonomousCommands = autonomousCommands;
    }

    @Override
    public void initialize() {
        Mat frame = new Mat();
        CameraServer.getInstance().getVideo("Flipped").grabFrame(frame);
        List<Point> ballLocs = findBallLocations(frame);
        int countLeft = 0;
        for (Point loc : ballLocs) {
            if (loc.x < frame.width() * LEFT_BALL_AREA) {
                countLeft++;
            }
        }
        //TODO make paths and uncomment
        // cmd = autonomousCommands.pathtracer(countLeft > ballLocs.size() - countLeft ?
        //         Pose2dPath.LEFT_FAH_CHALLENGE : Pose2dPath.RIGHT_FAH_CHALLENGE);
        // cmd.schedule();
    }

    private List<Point> findBallLocations(Mat img) {
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

    @Override
    public boolean isFinished() {
        return cmd != null && cmd.isScheduled();
    }
}
