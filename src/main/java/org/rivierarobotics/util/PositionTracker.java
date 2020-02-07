package org.rivierarobotics.util;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.rivierarobotics.subsystems.DriveTrain;

import javax.inject.Inject;
import javax.inject.Singleton;

public class PositionTracker {

    private final Timer time;
    private final NavXGyro gyro;
    private final DriveTrain driveTrain;
    private final double wheelCircumference = 0.32; // meters
    double[] pos = new double[2];
    double t = 0;

    @Inject
    public PositionTracker(DriveTrain dt, NavXGyro gyro)
    {
        this.gyro = gyro;
        this.driveTrain = dt;
        time = new Timer();
    }

    public void TrackPosition()
    {
        t = time.get() - t;
        double distanceTravelled = driveTrain.getAvgVelocity() * t;
        pos[0] = pos[0] + Math.sin(Math.toRadians(gyro.getYaw())) * distanceTravelled;
        pos[1] = pos[1] + Math.cos(Math.toRadians(gyro.getYaw())) * distanceTravelled;
        SmartDashboard.putNumber("EncoderX", pos[0]);
        SmartDashboard.putNumber("EncoderY", pos[1]);
    }

    public double[] GetPosition()
    {
        return pos;
    }




}
