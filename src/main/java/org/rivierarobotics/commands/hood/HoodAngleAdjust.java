package org.rivierarobotics.commands.hood;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import org.rivierarobotics.util.PhysicsUtil;

public class HoodAngleAdjust extends InstantCommand {
    private final double angle;
    public HoodAngleAdjust(double angle) {
        this.angle = angle;
    }

    @Override
    public void initialize() {
        PhysicsUtil.hoodAng += angle;
    }
}
