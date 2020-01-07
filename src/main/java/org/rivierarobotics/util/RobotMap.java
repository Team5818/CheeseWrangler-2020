package org.rivierarobotics.util;

public interface RobotMap {
    int LEFT_TALON_MASTER = 1;
    int LEFT_SPARK_SLAVE_ONE = 2;
    int LEFT_SPARK_SLAVE_TWO = 3;
    boolean LEFT_INVERT = true;

    int RIGHT_TALON_MASTER = 4;
    int RIGHT_SPARK_SLAVE_ONE = 5;
    int RIGHT_SPARK_SLAVE_TWO = 6;
    boolean RIGHT_INVERT = false;

    int LEFT_JS = 0;
    int RIGHT_JS = 1;
    int BUTTONS = 2;
}
