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
import org.rivierarobotics.autonomous.ChallengePath;
import org.rivierarobotics.autonomous.PathTracerExecutor;
import org.rivierarobotics.util.Pair;
import org.rivierarobotics.util.VisionUtil;

import java.util.ArrayList;
import java.util.List;

@GenerateCreator
public class GalacticSearch extends CommandBase {
    private static final Pair<Double> LEFT_BALL_AREAS = new Pair<>(0.75, 0.5);
    private static final Pair<Integer> NUM_LEFT_BALLS = new Pair<>(3, 2);
    private final AutonomousCommands autonomousCommands;
    private final VisionUtil visionUtil;
    private final boolean isPathA;
    private PathTracerExecutor cmd;

    public GalacticSearch(@Provided AutonomousCommands autonomousCommands,
                          @Provided VisionUtil visionUtil, boolean isPathA) {
        this.autonomousCommands = autonomousCommands;
        this.visionUtil = visionUtil;
        this.isPathA = isPathA;
    }

    @Override
    public void initialize() {
        Mat frame = new Mat();
        CameraServer.getInstance().getVideo("Flipped").grabFrame(frame);
        List<Point> ballLocs = visionUtil.findBallLocations(frame);
        double leftArea = isPathA ? LEFT_BALL_AREAS.getA() : LEFT_BALL_AREAS.getB();
        int countLeft = 0;
        for (Point loc : ballLocs) {
            if (loc.x < frame.width() * leftArea) {
                countLeft++;
            }
        }
        int leftBalls = isPathA ? NUM_LEFT_BALLS.getA() : NUM_LEFT_BALLS.getB();
        String pathName = "GS_" + (isPathA ? "A" : "B") + "_" + (countLeft == leftBalls ? "RED" : "BLUE");
        cmd = autonomousCommands.challengePath(ChallengePath.valueOf(pathName));
        cmd.schedule();
    }

    @Override
    public boolean isFinished() {
        return cmd != null && cmd.isScheduled();
    }
}
