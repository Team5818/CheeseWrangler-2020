package org.rivierarobotics.util;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoMode;
import edu.wpi.first.cameraserver.CameraServer;
import org.opencv.core.Core;
import org.opencv.core.Mat;

public class CameraFlip extends Thread {
    public static boolean DO_FLIP = false;

    @Override
    public void run() {
        UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
//        camera.setVideoMode(VideoMode.PixelFormat.kMJPEG, 320, 240, 60);
        CvSink cvSink = CameraServer.getInstance().getVideo();
        CvSource outputStream = CameraServer.getInstance().putVideo("Secondary", 320, 240);

        Mat source = new Mat();
        Mat output = new Mat();

        while(!Thread.interrupted()) {
            if (cvSink.grabFrame(source) == 0) {
                continue;
            }
            if (DO_FLIP) {
                // https://first.wpi.edu/FRC/roborio/release/docs/java/org/opencv/core/Core.html#flip(org.opencv.core.Mat,org.opencv.core.Mat,int)
                // https://docs.wpilib.org/en/stable/docs/software/vision-processing/introduction/using-the-cameraserver-on-the-roborio.html
                // 0 = flip about x axis
                Core.flip(source, output, 0);
            }
            outputStream.putFrame(output);
        }
        super.run();
    }
}
