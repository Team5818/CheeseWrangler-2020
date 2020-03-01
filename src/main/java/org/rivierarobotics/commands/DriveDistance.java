package org.rivierarobotics.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import net.octyl.aptcreator.GenerateCreator;
import net.octyl.aptcreator.Provided;
import org.rivierarobotics.subsystems.DriveTrain;
import org.rivierarobotics.util.MathUtil;

@GenerateCreator
public class DriveDistance extends CommandBase {
    private final DriveTrain driveTrain;
    private final double finalMeters;
    private double startMeters;

    public DriveDistance(@Provided DriveTrain driveTrain, double finalMeters) {
        this.driveTrain = driveTrain;
        this.finalMeters = finalMeters;
        addRequirements(driveTrain);
    }

    @Override
    public void initialize() {
        startMeters = driveTrain.getLeft().getPosition();
    }

    @Override
    public void execute() {
        double sign = Math.signum(finalMeters);
        driveTrain.setPower(sign * 0.3, sign * 0.3);
    }

    @Override
    public boolean isFinished() {
        return MathUtil.isWithinTolerance(driveTrain.getLeft().getPosition() - startMeters, finalMeters, 0.1);
    }
}
