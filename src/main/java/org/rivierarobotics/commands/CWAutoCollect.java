package org.rivierarobotics.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import net.octyl.aptcreator.GenerateCreator;
import net.octyl.aptcreator.Provided;
import org.rivierarobotics.subsystems.CheeseWheel;
import org.rivierarobotics.subsystems.Intake;

@GenerateCreator
public class CWAutoCollect extends CommandBase {
    private final CheeseWheel cheeseWheel;
    private final Intake intake;
    private final boolean side;

    public CWAutoCollect(@Provided CheeseWheel cheeseWheel, @Provided Intake intake, boolean side) {
        this.cheeseWheel = cheeseWheel;
        this.intake = intake;
        this.side = side;
        addRequirements(cheeseWheel, intake);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {

    }
}
