package org.rivierarobotics.autonomous;

import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;
import jaci.pathfinder.followers.EncoderFollower;
import org.rivierarobotics.subsystems.DriveTrain;
import org.rivierarobotics.subsystems.DriveTrainSide;

public class ForwardBackRoutine {

    Waypoint[] points = new Waypoint[] {      // Waypoint @ x=-4, y=-1, exit angle=-45 degrees
            new Waypoint(2, 2, 0),                        // Waypoint @ x=-2, y=-2, exit angle=0 radians
            new Waypoint(0, 0, 0)                           // Waypoint @ x=0, y=0,   exit angle=0 radians
    };
    Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC, Trajectory.Config.SAMPLES_HIGH, 0.05, 1.7, 2.0, 60.0);
    Trajectory trajectory = Pathfinder.generate(points, config);

    public void run() {
        DriveTrainSide leftTrain = new DriveTrainSide(new DriveTrainSide.MotorIds(4,5,6), true);
        DriveTrainSide rightTrain = new DriveTrainSide(new DriveTrainSide.MotorIds(1,2,3), false);

        EncoderFollower leftFollower = new EncoderFollower(trajectory);
        EncoderFollower rightFollower = new EncoderFollower(trajectory);

        leftFollower.configureEncoder((int)leftTrain.getPositionTicks(), 4096, 0.15);
        rightFollower.configureEncoder((int)rightTrain.getPositionTicks(), 4096, 0.15);

        leftFollower.configurePIDVA(1.0, 0.0, 0.0, 1 / 4, 0);
        rightFollower.configurePIDVA(1.0, 0.0, 0.0, 1 / 4, 0);

        leftTrain.setPower(leftFollower.calculate((int)leftTrain.getPositionTicks()));
        rightTrain.setPower(rightFollower.calculate((int)rightTrain.getPositionTicks()));
    }
}
