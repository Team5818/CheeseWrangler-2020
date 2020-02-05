package org.rivierarobotics.autonomous;

import jaci.pathfinder.Waypoint;

public enum WaypointConfig {
    CONFIG_ONE(new Waypoint(2, 2, 0),
            new Waypoint(0, 0, 0));

    public final Waypoint[] waypoints;

    //TODO note: the ... indicates any number of arguments, automatically turned into an array of that type
    WaypointConfig(Waypoint... points) {
        this.waypoints = points;
    }
}
