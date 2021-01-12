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
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import net.octyl.aptcreator.GenerateCreator;
import org.ejml.data.MatrixType;
import org.ejml.simple.SimpleMatrix;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.rivierarobotics.autonomous.AutonomousCommands;
import org.rivierarobotics.autonomous.PathTracerExecutor;
import org.rivierarobotics.autonomous.SplinePath;
import org.rivierarobotics.autonomous.SplinePoint;
import org.rivierarobotics.commands.collect.CollectionCommands;
import org.rivierarobotics.subsystems.CheeseWheel;
import org.rivierarobotics.util.NavXGyro;
import org.rivierarobotics.util.Pair;
import org.rivierarobotics.util.VisionUtil;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;
import javax.inject.Inject;

@GenerateCreator
public class FindAllBalls extends CommandBase {
    private static final Pair<Double> FOCAL_LENGTH_PX = new Pair<>(0.0, 0.0);
    private final AutonomousCommands autonomousCommands;
    private final CollectionCommands collectionCommands;
    private final VisionUtil visionUtil;
    private ParallelDeadlineGroup cmd;

    @Inject
    public FindAllBalls(AutonomousCommands autonomousCommands,
                        CollectionCommands collectionCommands,
                        VisionUtil visionUtil) {
        this.autonomousCommands = autonomousCommands;
        this.collectionCommands = collectionCommands;
        this.visionUtil = visionUtil;
    }

    @Override
    public void initialize() {
        SimpleMatrix intrinsic = new SimpleMatrix(3, 3, MatrixType.DDRM);
        Mat frame = new Mat();
        CameraServer.getInstance().getVideo("Flipped").grabFrame(frame);
        intrinsic.set(0, 0, FOCAL_LENGTH_PX.getA());
        intrinsic.set(1, 1, FOCAL_LENGTH_PX.getB());
        intrinsic.set(0, 2, frame.width() / 2.0);
        intrinsic.set(1, 2, frame.height() / 2.0);
        intrinsic = intrinsic.invert();

        List<SplinePoint> points = new LinkedList<>();
        List<Point> cvBalls = visionUtil.findBallLocations(frame);
        for (Point ballPt : cvBalls) {
            SimpleMatrix ballMatrix = new SimpleMatrix(3, 1, MatrixType.DDRM);
            ballMatrix.setColumn(0, 0, ballPt.x, ballPt.y, 1);
            var loc3d = intrinsic.mult(ballMatrix);
            points.add(new SplinePoint(loc3d.get(0, 0), loc3d.get(1, 0), 0));
        }

        cmd = autonomousCommands.pathtracer(new SplinePath(points))
                .deadlineWith(collectionCommands.continuous(CheeseWheel.AngleOffset.COLLECT_FRONT));
        cmd.schedule();
    }

    @Override
    public boolean isFinished() {
        return cmd != null && cmd.isScheduled();
    }
}
