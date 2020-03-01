package org.rivierarobotics.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import net.octyl.aptcreator.GenerateCreator;
import net.octyl.aptcreator.Provided;
import org.rivierarobotics.subsystems.DriveTrain;

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
        driveTrain.setPower(sign * 0.5, sign * 0.5);
    }

    @Override
    public void end(boolean interrupted) {
        driveTrain.setPower(0, 0);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(driveTrain.getLeft().getPosition() - startMeters) > Math.abs(finalMeters);
    }
}
