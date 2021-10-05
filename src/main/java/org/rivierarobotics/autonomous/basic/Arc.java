package org.rivierarobotics.autonomous.basic;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import net.octyl.aptcreator.Provided;
import org.rivierarobotics.commands.cheesewheel.CheeseWheelCommands;
import org.rivierarobotics.commands.drive.DriveCommands;
import org.rivierarobotics.commands.flywheel.FlywheelCommands;
import org.rivierarobotics.subsystems.DriveTrain;

public class Arc extends SequentialCommandGroup {
    public Arc(@Provided DriveCommands drive) {
        super(drive.driveArc(0.5, 10));
    }
}
