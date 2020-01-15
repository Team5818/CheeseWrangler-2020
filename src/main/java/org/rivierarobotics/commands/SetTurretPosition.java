package org.rivierarobotics.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import org.rivierarobotics.subsystems.Turret;

public class SetTurretPosition extends InstantCommand {
    private final Turret turret;
    private final int position;

    public SetTurretPosition(Turret turret, int position) {
        this.turret = turret;
        this.position = position;
    }

    @Override
    public void initialize() {
        turret.setPosition(position);
    }
}
