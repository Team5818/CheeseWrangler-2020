package org.rivierarobotics.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import org.rivierarobotics.util.ColorWheelColor;

//import com.revrobotics.
import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;

public class ColorWheel extends BasePIDSubsystem {
    private final WPI_TalonSRX ColorWheelTalon;
    private ColorWheelColor colorInit;
    private ColorSensorV3 colorSensorV3;
    //initial color
    public final double colorWheelRadius;
    //TODO Input some kind of Color sensor

    public ColorWheel(int id, double colorWheelRadius, ColorSensorV3 colorSensorV3) {
        super( 0.0, 0.0, 0.0, 1);
        ColorWheelTalon = new WPI_TalonSRX(id);
        ColorWheelTalon.configFactoryDefault();
        ColorWheelTalon.setNeutralMode(NeutralMode.Brake);
        this.colorWheelRadius = colorWheelRadius;
        this.colorSensorV3 = colorSensorV3;
    }
    public ColorWheelColor getColor(){
        if(colorSensorV3.getRed() == 255 && colorSensorV3.getGreen() == 0 && colorSensorV3.getBlue() == 0){
            return ColorWheelColor.RED;
        }else if(colorSensorV3.getRed() == 255 && colorSensorV3.getGreen() == 255 && colorSensorV3.getBlue() == 0){
            return ColorWheelColor.YELLOW;
        }else if(colorSensorV3.getRed() == 0 && colorSensorV3.getGreen() == 255 && colorSensorV3.getBlue() == 0){
            return  ColorWheelColor.GREEN;
        }else if (colorSensorV3.getRed() == 255 && colorSensorV3.getGreen() == 255 && colorSensorV3.getBlue() == 255){
            return  ColorWheelColor.BLUE;
        }
        return null;
    }

    @Override
    public double getPositionTicks() {
        return ColorWheelTalon.getSensorCollection().getQuadratureVelocity();
    }

    @Override
    protected void setPower(double pwr) {
        ColorWheelTalon.set(pwr);
    }
}