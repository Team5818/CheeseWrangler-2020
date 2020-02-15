package org.rivierarobotics.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import net.octyl.aptcreator.GenerateCreator;
import net.octyl.aptcreator.Provided;
import org.rivierarobotics.subsystems.ColorWheel;

@GenerateCreator
public class ColorWheelSpin extends InstantCommand {
    ColorWheel colorWheel;
    int rotations;
    double circumference = 1.0;

    public ColorWheelSpin(@Provided ColorWheel colorWheel, int rotations){
        this.colorWheel = colorWheel;
        this.rotations = rotations;
    }

    @Override
    public void execute(){
        colorWheel.setPosition((circumference * rotations)/colorWheel.colorWheelRadius);
    }


}
