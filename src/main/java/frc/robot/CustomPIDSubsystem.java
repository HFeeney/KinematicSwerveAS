// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * A subsystem that uses a {@link CustomPIDController} to control an output. The controller is run
 * synchronously from the subsystem's periodic() method.
 * This class is a modified version of PIDSubsystem that uses a CustomPIDController
 * The only modifications are to replace all PIDController objects with CustomPIDController objects
 */
public abstract class CustomPIDSubsystem extends SubsystemBase {
  protected final CustomPIDController m_controller;
  protected boolean m_enabled;

  private double m_setpoint;

  /**
   * Creates a new PIDSubsystem.
   *
   * @param controller the CustomPIDController to use
   * @param initialPosition the initial setpoint of the subsystem
   */
  public CustomPIDSubsystem(CustomPIDController controller, double initialPosition) {
    setSetpoint(initialPosition);
    m_controller = controller;
    addChild("PID Controller", m_controller);
  }

  /**
   * Creates a new PIDSubsystem. Initial setpoint is zero.
   *
   * @param controller the CustomPIDController to use
   */
  public CustomPIDSubsystem(CustomPIDController controller) {
    this(controller, 0);
  }

  @Override
  public void periodic() {
    if (m_enabled) {
      useOutput(m_controller.calculate(getMeasurement(), m_setpoint), m_setpoint);
    }
  }

  public CustomPIDController getController() {
    return m_controller;
  }

  /**
   * Sets the setpoint for the subsystem.
   *
   * @param setpoint the setpoint for the subsystem
   */
  public void setSetpoint(double setpoint) {
    m_setpoint = setpoint;
  }

  /**
   * Returns the current setpoint of the subsystem.
   *
   * @return The current setpoint
   */
  public double getSetpoint() {
    return m_setpoint;
  }

  /**
   * Uses the output from the CustomPIDController.
   *
   * @param output the output of the CustomPIDController
   * @param setpoint the setpoint of the CustomPIDController (for feedforward)
   */
  protected abstract void useOutput(double output, double setpoint);

  /**
   * Returns the measurement of the process variable used by the CustomPIDController.
   *
   * @return the measurement of the process variable
   */
  protected abstract double getMeasurement();

  /** Enables the PID control. Resets the controller. */
  public void enable() {
    m_enabled = true;
    m_controller.reset();
  }

  /** Disables the PID control. Sets output to zero. */
  public void disable() {
    m_enabled = false;
    useOutput(0, 0);
  }

  /**
   * Returns whether the controller is enabled.
   *
   * @return Whether the controller is enabled.
   */
  public boolean isEnabled() {
    return m_enabled;
  }
}
