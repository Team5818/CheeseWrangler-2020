package org.rivierarobotics.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.rivierarobotics.util.RobotMap;

public class DriveTrain extends SubsystemBase {
    private final DriveTrainSide left, right;

    public DriveTrain() {
        this.left = new DriveTrainSide(RobotMap.LEFT_TALON_MASTER, RobotMap.LEFT_SPARK_SLAVE_ONE,
                RobotMap.LEFT_SPARK_SLAVE_TWO, RobotMap.LEFT_INVERT);
        this.right = new DriveTrainSide(RobotMap.RIGHT_TALON_MASTER, RobotMap.RIGHT_SPARK_SLAVE_ONE,
                RobotMap.RIGHT_SPARK_SLAVE_TWO, RobotMap.RIGHT_INVERT);
    }

    public void setPower(double l, double r) {
        left.setPower(l);
        right.setPower(r);
    }

    public DriveTrainSide getLeft() {
        return left;
    }

    public DriveTrainSide getRight() {
        return right;
    }

    @Override
    public void setDefaultCommand(Command defaultCommand) {

    }
}
