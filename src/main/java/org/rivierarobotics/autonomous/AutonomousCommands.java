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

import org.rivierarobotics.autonomous.advanced.*;
import org.rivierarobotics.autonomous.advanced.CenterShootCreator;
import org.rivierarobotics.autonomous.advanced.EnemyTrench5Ball;
import org.rivierarobotics.autonomous.basic.ForwardAuto;
import org.rivierarobotics.autonomous.basic.ForwardAutoCreator;
import org.rivierarobotics.autonomous.basic.ShootAndDrive;
import org.rivierarobotics.autonomous.basic.ShootAndDriveCreator;
import org.rivierarobotics.autonomous.basic.ShootThreeBalls;
import org.rivierarobotics.autonomous.basic.ShootThreeBallsCreator;

import javax.inject.Inject;

public class AutonomousCommands {
    private final PathTracerExecutorCreator pathTracerExecutorCreator;
    private final ForwardAutoCreator forwardAutoCreator;
    private final ShootAndDriveCreator shootAndDriveCreator;
    private final ShootLoopCreator shootLoopCreator;
    private final TrenchRunCreator trenchRunCreator;
    private final RecordPathCreator recordPathCreator;
    private final PowerReplayCreator powerReplayCreator;
    private final ShootThreeBallsCreator shootThreeBallsCreator;
    private final TriangleAdvancedCreator triangleAdvancedCreator;
    private final SixBallTrenchCreator sixBallTrenchCreator;
    private final EnemyTrench5BallCreator enemyTrench5BallCreator;
    private final CenterShootCreator centerShootCreator;
    private final OffsetSixBallTrenchCreator offsetSixBallTrenchCreator;

    @Inject
    public AutonomousCommands(PathTracerExecutorCreator pathTracerExecutorCreator,
                              ForwardAutoCreator forwardAutoCreator,
                              ShootAndDriveCreator shootAndDriveCreator,
                              ShootLoopCreator shootLoopCreator,
                              TrenchRunCreator trenchRunCreator,
                              RecordPathCreator recordPathCreator,
                              PowerReplayCreator powerReplayCreator,
                              ShootThreeBallsCreator shootThreeBallsCreator,
                              TriangleAdvancedCreator triangleAdvancedCreator,
                              SixBallTrenchCreator sixBallTrenchCreator,
                              EnemyTrench5BallCreator enemyTrench5BallCreator,
                              OffsetSixBallTrenchCreator offsetSixBallTrenchCreator,
                              CenterShootCreator centerShootCreator) {
        this.pathTracerExecutorCreator = pathTracerExecutorCreator;
        this.forwardAutoCreator = forwardAutoCreator;
        this.shootAndDriveCreator = shootAndDriveCreator;
        this.shootLoopCreator = shootLoopCreator;
        this.trenchRunCreator = trenchRunCreator;
        this.recordPathCreator = recordPathCreator;
        this.powerReplayCreator = powerReplayCreator;
        this.shootThreeBallsCreator = shootThreeBallsCreator;
        this.triangleAdvancedCreator = triangleAdvancedCreator;
        this.sixBallTrenchCreator = sixBallTrenchCreator;
        this.offsetSixBallTrenchCreator = offsetSixBallTrenchCreator;
        this.enemyTrench5BallCreator = enemyTrench5BallCreator;
        this.centerShootCreator = centerShootCreator;
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

    public ShootThreeBalls shootThreeBalls() {
        return shootThreeBallsCreator.create();
    }

    public TriangleAdvanced triangleAdvanced() {
        return triangleAdvancedCreator.create();
    }

    public SixBallTrench sixBallTrench() {
        return sixBallTrenchCreator.create();
    }

    public EnemyTrench5Ball enemyTrench5Ball() {
        return enemyTrench5BallCreator.create();
    }

    public OffsetSixBallTrench offsetSixBallTrench() {
        return offsetSixBallTrenchCreator.create();
    }

    public CenterShoot centerShoot() {
        return centerShootCreator.create();
    }
}
