package org.jumprobotics.swervedrive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

// Copied from team3181's REVSwerve2023 repo
public class YepSwerveModuleState extends SwerveModuleState {
    public double speedMetersPerSecond;

    /** Rad per sec */
    public double omegaRadPerSecond = 0;

    public Rotation2d angle = Rotation2d.fromDegrees(0);

    /** Constructs a SwerveModuleState with zeros for speed and angle. */
    public YepSwerveModuleState() {}

    /**
     * Constructs a SwerveModuleState.
     *
     * @param speedMetersPerSecond The speed of the wheel of the module.
     * @param angle The angle of the module.
     * @param omegaRadPerSecond The angular velocity of the module.
     */
    public YepSwerveModuleState(double speedMetersPerSecond, Rotation2d angle, double omegaRadPerSecond) {
        this.speedMetersPerSecond = speedMetersPerSecond;
        this.angle = angle;
        this.omegaRadPerSecond = omegaRadPerSecond;
    }
}