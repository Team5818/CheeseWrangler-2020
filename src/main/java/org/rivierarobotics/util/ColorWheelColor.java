package org.rivierarobotics.util;

public enum ColorWheelColor {
    GREEN(0, 255, 0), BLUE(0, 255, 255), YELLOW(255, 255, 0), RED(255, 0, 0);
    int r, g, b;
    ColorWheelColor(int r, int g, int b) {
        this.r = r;
        this.g = g;
        this.b = b;
    }
}
