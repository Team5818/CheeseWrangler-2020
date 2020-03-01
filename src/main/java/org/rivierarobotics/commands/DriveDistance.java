package org.rivierarobotics.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import net.octyl.aptcreator.GenerateCreator;
import net.octyl.aptcreator.Provided;
import org.rivierarobotics.subsystems.DriveTrain;

@GenerateCreator
public class DriveDistance extends CommandBase {
    private final DriveTrain driveTrain;
    private final double finalMeters;
    private final double power;
    private double startMeters;

    public DriveDistance(@Provided DriveTrain driveTrain, double finalMeters, double power) {
        this.driveTrain = driveTrain;
        this.finalMeters = finalMeters;
        this.power = power;
        addRequirements(driveTrain);
    }

    @Override
    public void initialize() {
        startMeters = driveTrain.getLeft().getPosition();
    }

    @Override
    public void execute() {
        double sign = Math.signum(finalMeters);
        double directedPower = sign * power;
        driveTrain.setPower(directedPower, directedPower);
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
