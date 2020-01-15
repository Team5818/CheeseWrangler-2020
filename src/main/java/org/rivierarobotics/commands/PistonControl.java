package org.rivierarobotics.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import org.rivierarobotics.subsystems.Piston;

public class PistonControl extends InstantCommand {
    private final Piston pist;
    private final Boolean extendPiston;

    public PistonControl(Boolean extendPiston) {
        pist = new Piston();
        this.extendPiston = extendPiston;
    }

    @Override
    public void execute() {
        pist.extend(extendPiston);
    }
}
