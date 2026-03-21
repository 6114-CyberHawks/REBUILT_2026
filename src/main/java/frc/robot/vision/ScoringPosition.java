// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision;

public class ScoringPosition {
    private final String name;
    private final double targetDistance;
    private final double targetAngle;  // Horizontal offset from center

    public ScoringPosition(String name, double targetDistance, double targetAngle) {
        this.name = name;
        this.targetDistance = targetDistance;
        this.targetAngle = targetAngle;
    }

    // Constructor for straight-on shots (no angle offset)
    public ScoringPosition(String name, double targetDistance) {
        this(name, targetDistance, 0.0);
    }

    public String getName() { return name; }
    public double getTargetDistance() { return targetDistance; }
    public double getTargetAngle() { return targetAngle; }

    @Override
    public String toString() {
        return String.format("%s (%.1f\", %.1f°)", name, targetDistance, targetAngle);
    }
}