package org.rivierarobotics.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import net.octyl.aptcreator.GenerateCreator;
import net.octyl.aptcreator.Provided;
import org.rivierarobotics.subsystems.ColorWheel;
import org.rivierarobotics.util.ColorWheelColor;

@GenerateCreator
public class ColorWheelSetColor extends InstantCommand {
    ColorWheel colorWheel;
    ColorWheelColor color;
    //the color that we are setting it to

    public ColorWheelSetColor(@Provided ColorWheel colorWheel, ColorWheelColor color){
        this.colorWheel = colorWheel;
        this.color = color;
    }

    @Override
    public void execute(){


    }
    //TODO be able to rotate wheel until it reaches certain color



}
