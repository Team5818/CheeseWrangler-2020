package org.rivierarobotics.commands.hood;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import net.octyl.aptcreator.GenerateCreator;
import net.octyl.aptcreator.Provided;
import org.rivierarobotics.commands.turret.TurretCommands;
import org.rivierarobotics.subsystems.Hood;

@GenerateCreator
public class SetTrench extends SequentialCommandGroup {
    public SetTrench(@Provided HoodCommands hoodCommands, @Provided TurretCommands turretCommands, boolean set) {
        super(
            turretCommands.setAngle(0),
            hoodCommands.setAngle(0)
        );
    }
}
