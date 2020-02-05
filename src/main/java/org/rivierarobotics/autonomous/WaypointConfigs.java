package org.rivierarobotics.autonomous;

import jaci.pathfinder.Waypoint;

public enum WaypointConfigs {

    CONFIG_ONE(Waypoint[] {
        new Waypoint(2, 2, 0),
                new Waypoint(0, 0, 0) };);

    private Waypoint[] waypointConfigOne;

    WaypointConfigs(Waypoint... points) {

    }
}
