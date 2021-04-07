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

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.first.cameraserver.CameraServer;
import org.opencv.core.Core;
import org.opencv.core.Mat;

public class CameraFlip extends Thread {
    public static boolean DO_FLIP = false;
    private final CvSink cvSink;
    private final CvSource outputStream;

    public CameraFlip() {
        this.cvSink = CameraServer.getInstance().getVideo("USB Camera 0");
        this.outputStream = CameraServer.getInstance().putVideo("Flipped", 320, 240);
    }

    @Override
    public void run() {
        Mat source = new Mat();
        Mat output = new Mat();

        while (!Thread.interrupted()) {
            if (cvSink.grabFrame(source) == 0) {
                continue;
            }
            if (DO_FLIP) {
                // https://first.wpi.edu/FRC/roborio/release/docs/java/org/opencv/core/Core.html#flip(org.opencv.core.Mat,org.opencv.core.Mat,int)
                // https://docs.wpilib.org/en/stable/docs/software/vision-processing/introduction/using-the-cameraserver-on-the-roborio.html
                // 0 = flip about x axis
                Core.flip(source, output, -1);
            } else {
                output = source.clone();
            }
            outputStream.putFrame(output);
        }
        super.run();
    }
}
