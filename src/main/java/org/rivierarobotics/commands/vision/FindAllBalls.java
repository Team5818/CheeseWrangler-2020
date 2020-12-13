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
import org.ejml.data.MatrixType;
import org.ejml.simple.SimpleMatrix;
import org.opencv.core.Mat;
import org.rivierarobotics.autonomous.AutonomousCommands;
import org.rivierarobotics.autonomous.PathTracerExecutor;
import org.rivierarobotics.autonomous.SplinePath;
import org.rivierarobotics.autonomous.SplinePoint;
import org.rivierarobotics.util.NavXGyro;
import org.rivierarobotics.util.Pair;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;
import javax.inject.Inject;

@GenerateCreator
public class FindAllBalls extends CommandBase {
    private static final Pair<Double> FOCAL_LENGTH_PX = new Pair<>(0.0, 0.0);
    private final AutonomousCommands autonomousCommands;
    private final NavXGyro gyro;
    private SimpleMatrix intrinsic;
    private Mat frame;
    private boolean finished;
    private PathTracerExecutor cmd;

    @Inject
    public FindAllBalls(AutonomousCommands autonomousCommands, NavXGyro gyro) {
        this.autonomousCommands = autonomousCommands;
        this.gyro = gyro;
    }

    @Override
    public void initialize() {
        finished = false;
        intrinsic = new SimpleMatrix(3, 3, MatrixType.DDRM);
        CameraServer.getInstance().getVideo("Flipped").grabFrame(frame);
        intrinsic.set(0, 0, FOCAL_LENGTH_PX.getA());
        intrinsic.set(1, 1, FOCAL_LENGTH_PX.getB());
        intrinsic.set(0, 2, frame.width() / 2.0);
        intrinsic.set(1, 2, frame.height() / 2.0);
        intrinsic = intrinsic.invert();
    }

    @Override
    public void execute() {
        List<SplinePoint> points = new LinkedList<>();
        List<SimpleMatrix> balls = findBalls();
        SimpleMatrix last = null;
        for (int i = 0; i < balls.size(); i++) {
            var loc3d = intrinsic.mult(balls.get(i));
            last = loc3d;
            //TODO final gyro angle dynamic
            points.add(new SplinePoint(loc3d.get(0, 0), loc3d.get(1, 0), 0));
        }
        cmd = autonomousCommands.pathtracer(new SplinePath(points), false, false);
        cmd.schedule();
    }

    private List<SimpleMatrix> findBalls() {
        //TODO ball finding, transmitted as [x y 1]
        //new SimpleMatrix(3, 1, MatrixType.DDRM)
        return new ArrayList<>();
    }

    @Override
    public boolean isFinished() {
        return cmd != null && cmd.isScheduled();
    }
}
