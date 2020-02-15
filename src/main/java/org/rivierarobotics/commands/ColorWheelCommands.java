package org.rivierarobotics.commands;
import org.rivierarobotics.util.ColorWheelColor;

public class ColorWheelCommands {
    private ColorWheelSpinCreator colorWheelSpinCreator;
    private ColorWheelSetColorCreator colorWheelSetColorCreator;

    public ColorWheelCommands(ColorWheelSpinCreator colorWheelSpinCreator, ColorWheelSetColorCreator colorWheelSetColorCreator  ) {
        this.colorWheelSpinCreator = colorWheelSpinCreator;
        this.colorWheelSetColorCreator = colorWheelSetColorCreator;
    }

    public ColorWheelSpin spin(int rotations) {
        return colorWheelSpinCreator.create(rotations);
    }
    public ColorWheelSetColor setColor(ColorWheelColor color){
        return  colorWheelSetColorCreator.create(color);
    }
}
