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
import org.rivierarobotics.util.Pair;
import org.rivierarobotics.util.RobotShuffleboard;
import org.rivierarobotics.util.RobotShuffleboardTab;

import java.awt.Color;
import java.util.Map;

@GenerateCreator
public class PathTracerExecutor extends CommandBase {
    private static final int WIDTH = 300;
    private static final int HEIGHT = 300;
    private static final double BORDER_WIDTH = 10;
    private static final double BUFFER_SECONDS = 0.5;
    private static final int ARROW_DENSITY = 180;
    private static final String PATH_TRACER = "PathTracer";
    private final DriveTrain driveTrain;
    private final DriveCommands driveCommands;
    private final NavXGyro gyro;
    private final RobotShuffleboardTab tab;
    private final SplinePath path;
    private final boolean absPos;
    private final boolean absHeading;
    private int loopRun = 0;
    private Mat mat;
    private CvSource graphPublish;
    private double t;
    private double scale;
    private int thickness;

    public PathTracerExecutor(@Provided DriveTrain driveTrain, @Provided DriveCommands driveCommands,
                              @Provided NavXGyro gyro, @Provided RobotShuffleboard shuffleboard,
                              SplinePath path, boolean absPos, boolean absHeading) {
        this.driveTrain = driveTrain;
        this.driveCommands = driveCommands;
        this.gyro = gyro;
        this.tab = shuffleboard.getTab(PATH_TRACER);
        this.path = path;
        this.absPos = absPos;
        this.absHeading = absHeading;
        addRequirements(driveTrain);
    }

    public static Scalar fromAWT(Color color) {
        return new Scalar(color.getBlue(), color.getGreen(), color.getRed());
    }

    private void initGraph() {
        //TODO fix graph scaling
        double mWidth = path.getExtrema().getA() + BORDER_WIDTH;
        double mHeight = path.getExtrema().getB() + BORDER_WIDTH;
        scale = Math.max(WIDTH / mWidth, HEIGHT / mHeight);
        thickness = (int) (scale *  (0.1));

        graphPublish = CameraServer.getInstance().putVideo(PATH_TRACER, (int) mWidth, (int) mHeight);
        boolean makeWidget = true;
        for (ShuffleboardComponent<?> comp : tab.getAPITab().getComponents()) {
            if (comp.getTitle().equals(PATH_TRACER)) {
                makeWidget = false;
                break;
            }
        }
        if (makeWidget) {
            tab.getAPITab().add(PATH_TRACER, graphPublish).withProperties(Map.of("Crosshair color", "Black"));
        }

        mat = new Mat(WIDTH, HEIGHT, CvType.CV_8UC3, new Scalar(255, 255, 255));
        for(int i = 0; i < path.getPrecomputedSpline().size(); i++) {
            drawGraphPoint(path.getPrecomputedSpline().get(i), fromAWT(Color.ORANGE), fromAWT(Color.CYAN), i);
            graphPublish.putFrame(mat);
        }
    }

    private void drawGraphPoint(SPOutput out, Scalar posColor, Scalar velColor, int index) {
        Point posBounds = new Point(((int) (out.getPosX() * scale)) + (WIDTH / 2.0), ((int) (out.getPosY() * scale)) + (HEIGHT / 2.0));
        if (index % ARROW_DENSITY == 0) {
            Point velBounds = new Point((int) (posBounds.x + out.getVelX() * 20), (int) (posBounds.y + out.getVelY() * 20));
            Imgproc.arrowedLine(mat, posBounds, velBounds, velColor, thickness);
        }
        Imgproc.circle(mat, posBounds, thickness, posColor, -1);
    }

    @Override
    public void initialize() {
        t = 0;
        initGraph();
        if (absPos) {
            path.addPathPoint(0, new SplinePoint(driveTrain.getPose()));
            path.recalculatePath();
        } else if (absHeading) {
            rotateToPointHeading(path.getPathPoints().get(0));
        }
        tab.setEntry("totalTime", path.getTotalTime());
        tab.setEntry("scale", scale);
    }

    public Pair<Double> getLRVel(double velX, double velY) {
        return path.getLRVel(velX, velY, Math.toRadians(gyro.getYaw()));
    }

    public void rotateToPointHeading(SplinePoint pt) {
        driveCommands.rotateTo(pt.isPrecomputedTan() ? Math.acos(pt.getTanVX()) : pt.getHeading()).schedule();
    }

    @Override
    public void execute() {
        if (t < path.getTotalTime()) {
            SPOutput calc = path.calculate(t);
            drawGraphPoint(calc, fromAWT(Color.RED), fromAWT(Color.BLUE), loopRun);
            loopRun++;
            graphPublish.putFrame(mat);
            tab.setEntry("calc", calc.toString());
            tab.setEntry("currTime", t);
            Pair<Double> wheelSpeeds = getLRVel(calc.getVelX(), calc.getVelY());
            tab.setEntry("vL", wheelSpeeds.getA());
            tab.setEntry("vR", wheelSpeeds.getB());
            driveTrain.setVelocity(wheelSpeeds.getA(), wheelSpeeds.getB());
        }
        t += 0.0025; //time scale
    }

    @Override
    public void end(boolean interrupted) {
        rotateToPointHeading(path.getPathPoints().get(path.getPathPoints().size() - 1));
    }

    @Override
    public boolean isFinished() {
        return t >= path.getTotalTime() + BUFFER_SECONDS;
    }
}
