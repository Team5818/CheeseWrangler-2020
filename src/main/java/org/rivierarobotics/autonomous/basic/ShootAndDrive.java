package org.rivierarobotics.autonomous.basic;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import net.octyl.aptcreator.GenerateCreator;
import net.octyl.aptcreator.Provided;
import org.rivierarobotics.commands.CheeseWheelCommands;
import org.rivierarobotics.commands.DriveCommands;
import org.rivierarobotics.util.VisionTarget;

@GenerateCreator
public class ShootAndDrive extends SequentialCommandGroup {

    public ShootAndDrive(@Provided DriveCommands drive,
                         @Provided CheeseWheelCommands cheeseWheel) {
        super(
            cheeseWheel.shootNWedges(VisionTarget.INNER, 5),
            drive.driveDistance(-1, 0.25)
        );
    }

}
