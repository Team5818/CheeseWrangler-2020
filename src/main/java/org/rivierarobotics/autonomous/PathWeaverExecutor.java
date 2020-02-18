package org.rivierarobotics.autonomous;

import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.rivierarobotics.util.MathUtil;

public class PathWeaverExecutor extends CommandBase {
    private final double ksVolts = 0.0;

    private final double kVoltsSecondsPerMeter = 0.0;
    private final double kaVoltsSecondsSquaredperMeter = 0.0;
    private final double kMaxSpeedMetersPerSecond = 0.0;
    private final double kMaxAccelerationMetersPerSecondSquared = 0.0;
    public static final double kTrackwidthMeters = MathUtil.feetToMeters(2.2083);
    public static final DifferentialDriveKinematics kDriveKinematics =
            new DifferentialDriveKinematics(kTrackwidthMeters);
    private static final double maxVoltage = 10.0 ;

    //TODO find and replace with real values

    DifferentialDriveVoltageConstraint differentialDriveVoltageConstraint = new DifferentialDriveVoltageConstraint(new SimpleMotorFeedforward(
            ksVolts, kVoltsSecondsPerMeter, kaVoltsSecondsSquaredperMeter),kDriveKinematics, 10.0);

    TrajectoryConfig config =
            new TrajectoryConfig(kMaxSpeedMetersPerSecond, kMaxAccelerationMetersPerSecondSquared)
                    .setKinematics(kDriveKinematics)
                    .addConstraint(differentialDriveVoltageConstraint);



}
