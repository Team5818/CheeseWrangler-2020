package org.rivierarobotics.commands;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.CommandBase;
import net.octyl.aptcreator.GenerateCreator;
import net.octyl.aptcreator.Provided;
import org.rivierarobotics.subsystems.Hood;

@GenerateCreator
public class HoodAlignQuadrature extends CommandBase {
    private final Hood hood;
    private final DigitalInput limit;

    public HoodAlignQuadrature(@Provided Hood hood) {
        this.hood = hood;
        this.limit = hood.di;
        addRequirements(hood);
    }

    @Override
    public void execute() {
        hood.setPower(-0.25);
    }

    @Override
    public void end(boolean interrupted) {
        hood.setPower(0.0);
        hood.getHoodTalon().getSensorCollection().setQuadraturePosition(0, 10);
    }

    @Override
    public boolean isFinished() {
        return limit.get();
    }
}
