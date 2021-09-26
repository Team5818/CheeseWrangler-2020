/*
 * This file is part of CheeseWrangler-2020, licensed under the GNU General Public License (GPLv3).
 *
 * Copyright (c) Riviera Robotics <https://github.com/Team5818>
 * Copyright (c) contributors
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

package org.rivierarobotics.autonomous;

import org.rivierarobotics.autonomous.advanced.ChallengePath;
import org.rivierarobotics.autonomous.advanced.ShootLoop;
import org.rivierarobotics.autonomous.advanced.ShootLoopCreator;
import org.rivierarobotics.autonomous.advanced.TrenchRun;
import org.rivierarobotics.autonomous.advanced.TrenchRunCreator;
import org.rivierarobotics.autonomous.basic.TestCreator;
import org.rivierarobotics.autonomous.basic.*;

import javax.inject.Inject;

public class AutonomousCommands {
    private final PathTracerExecutorCreator pathTracerExecutorCreator;
    private final ForwardAutoCreator forwardAutoCreator;
    private final ShootAndDriveCreator shootAndDriveCreator;
    private final ShootLoopCreator shootLoopCreator;
    private final TrenchRunCreator trenchRunCreator;
    private final RecordPathCreator recordPathCreator;
    private final PowerReplayCreator powerReplayCreator;
    private final TestCreator testCreator;

    @Inject
    public AutonomousCommands(PathTracerExecutorCreator pathTracerExecutorCreator,
                              ForwardAutoCreator forwardAutoCreator,
                              ShootAndDriveCreator shootAndDriveCreator,
                              ShootLoopCreator shootLoopCreator,
                              TrenchRunCreator trenchRunCreator,
                              RecordPathCreator recordPathCreator,
                              PowerReplayCreator powerReplayCreator,
                              TestCreator testCreator) {
        this.pathTracerExecutorCreator = pathTracerExecutorCreator;
        this.forwardAutoCreator = forwardAutoCreator;
        this.shootAndDriveCreator = shootAndDriveCreator;
        this.shootLoopCreator = shootLoopCreator;
        this.trenchRunCreator = trenchRunCreator;
        this.recordPathCreator = recordPathCreator;
        this.powerReplayCreator = powerReplayCreator;
        this.testCreator = testCreator;
    }

    public PathTracerExecutor challengePath(ChallengePath cPath) {
        return pathtracer(cPath.getPath());
    }

    public PathTracerExecutor pathtracer(Pose2dPath path) {
        return pathtracer(path, PathConstraints.create());
    }

    public PathTracerExecutor pathtracer(Pose2dPath path, PathConstraints constraints) {
        return pathtracer(new SplinePath(path.getSpline(), constraints));
    }

    public PathTracerExecutor pathtracer(SplinePath path) {
        return pathTracerExecutorCreator.create(path);
    }

    public ForwardAuto forwardAuto(boolean useVisionAim) {
        return forwardAutoCreator.create(useVisionAim);
    }

    public ShootAndDrive shootAndDrive() {
        return shootAndDriveCreator.create();
    }

    public Test test() {
        return testCreator.create();
    }

    public ShootLoop shootLoop(Pose2dPath loop) {
        return shootLoopCreator.create(loop);
    }

    public TrenchRun trenchRun() {
        return trenchRunCreator.create();
    }

    public RecordPath recordPath(boolean isPower) {
        return recordPathCreator.create(isPower);
    }

    public PowerReplay powerReplay() {
        return powerReplayCreator.create();
    }
}
