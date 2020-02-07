package org.rivierarobotics.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import net.octyl.aptcreator.Provided;
import org.rivierarobotics.subsystems.Ejector;

public class EjectorEjectCheese extends InstantCommand {

    private final Ejector ejector;

    public EjectorEjectCheese(@Provided Ejector ejector) {
        this.ejector = ejector;
        addRequirements(ejector);
    }

    @Override
    public void execute() {
        ejector.setPower(0.8);
    }
}
