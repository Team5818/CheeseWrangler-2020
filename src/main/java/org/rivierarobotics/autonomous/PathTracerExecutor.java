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

import edu.wpi.cscore.CvSource;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardComponent;
import edu.wpi.first.wpilibj2.command.CommandBase;
import net.octyl.aptcreator.GenerateCreator;
import net.octyl.aptcreator.Provided;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.rivierarobotics.commands.drive.DriveCommands;
import org.rivierarobotics.subsystems.DriveTrain;
import org.rivierarobotics.util.NavXGyro;
import org.rivierarobotics.util.RobotShuffleboard;
import org.rivierarobotics.util.RobotShuffleboardTab;

import java.awt.Color;
import java.util.Arrays;

@GenerateCreator
public class PathTracerExecutor extends CommandBase {
    private static final int WIDTH = 640;
    private static final int HEIGHT = 480;
    private static final double BORDER_WIDTH = 10;
    private static final double BUFFER_SECONDS = 0.5;
    private static final String PATH_TRACER = "PathTracer";
    private final DriveTrain driveTrain;
    private final DriveCommands driveCommands;
    private final NavXGyro gyro;
    private final RobotShuffleboardTab tab;
    private final SplinePath path;
    private Mat mat;
    private CvSource graphPublish;
    private double t;
    private double scale;
    private int thickness;

    public PathTracerExecutor(@Provided DriveTrain driveTrain, @Provided DriveCommands driveCommands,
                              @Provided NavXGyro gyro, @Provided RobotShuffleboard shuffleboard, SplinePath path) {
        this.driveTrain = driveTrain;
        this.driveCommands = driveCommands;
        this.gyro = gyro;
        this.tab = shuffleboard.getTab(PATH_TRACER);
        this.path = path;
        addRequirements(driveTrain);
    }

    public static Scalar fromAWT(Color color) {
        return new Scalar(color.getBlue(), color.getGreen(), color.getRed());
    }

    private void initGraph() {
        double mWidth = path.getExtrema().getPosX() + (2 * BORDER_WIDTH);
        double mHeight = path.getExtrema().getPosY() + (2 * BORDER_WIDTH);
        scale = Math.max(WIDTH / mWidth, HEIGHT / mHeight);
        thickness = (int) (scale / 4);

        graphPublish = CameraServer.getInstance().putVideo(PATH_TRACER, WIDTH, HEIGHT);
        boolean makeWidget = true;
        for (ShuffleboardComponent<?> comp : tab.getAPITab().getComponents()) {
            if (comp.getTitle().equals(PATH_TRACER)) {
                makeWidget = false;
                break;
            }
        }
        if (makeWidget) {
            tab.setVideoSource(graphPublish);
        }

        mat = new Mat(WIDTH, HEIGHT, CvType.CV_8UC3, new Scalar(255, 255, 255));
        for (SplinePath.Output out : path.getPrecomputedSpline()) {
            drawGraphPoint(out, fromAWT(Color.ORANGE), fromAWT(Color.CYAN));
        }
        graphPublish.putFrame(mat);
    }

    private void drawGraphPoint(SplinePath.Output out, Scalar posColor, Scalar velColor) {
        Point posBounds = new Point(((int) (out.getPosX() * scale)) + (WIDTH / 2.0), ((int) (out.getPosY() * scale)) + (HEIGHT / 2.0));
        Imgproc.circle(mat, posBounds, thickness, posColor, thickness);
        Point velBounds = new Point(((int) (out.getVelX() * scale)) + (WIDTH / 2.0), ((int) (out.getVelY() * scale)) + (HEIGHT / 2.0));
        Imgproc.circle(mat, velBounds, thickness, velColor, thickness);
    }

    @Override
    public void initialize() {
        t = 0;
        initGraph();
        System.out.println(Arrays.toString(path.getSections().keySet().toArray(new Double[0])));
        tab.setEntry("totalTime", path.getTotalTime());
        tab.setEntry("scale", scale);
    }

    public double[] getLRVel(double velX, double velY) {
        return path.getLRVel(velX, velY, Math.toRadians(gyro.getYaw()));
    }

    @Override
    public void execute() {
        if (t < path.getTotalTime()) {
            SplinePath.Output calc = path.calculate(t);
            drawGraphPoint(calc, fromAWT(Color.RED), fromAWT(Color.BLUE));
            graphPublish.putFrame(mat);
            tab.setEntry("calc", calc.toString());
            tab.setEntry("currTime", t);
            double[] wheelSpeeds = getLRVel(calc.getVelX(), calc.getVelY());
            driveTrain.setVelocity(wheelSpeeds[0], wheelSpeeds[1]);
        }
        t += 0.0025; //time scale
    }

    @Override
    public void end(boolean interrupted) {
        SplinePoint endPt = path.getPathPoints().get(path.getPathPoints().size() - 1);
        driveCommands.rotateTo(endPt.isPrecomputedTan() ? Math.acos(endPt.getTanVX()) : endPt.getHeading()).schedule();
    }

    @Override
    public boolean isFinished() {
        return t >= path.getTotalTime() + BUFFER_SECONDS;
    }
}
