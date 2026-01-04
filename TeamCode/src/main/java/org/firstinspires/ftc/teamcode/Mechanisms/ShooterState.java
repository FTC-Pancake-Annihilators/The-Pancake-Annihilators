package org.firstinspires.ftc.teamcode.Mechanisms;

/**
 * Enum representing the possible states of the shooter subsystem.
 *
 * STOP    - Shooter is off (motors stopped)
 * AUTO_AIM - Shooter velocity is automatically adjusted based on distance to goal
 * MANUAL   - Driver manually controls shooter velocity
 */
public enum ShooterState {
    STOP,
    AUTO_AIM,
    MANUAL
}