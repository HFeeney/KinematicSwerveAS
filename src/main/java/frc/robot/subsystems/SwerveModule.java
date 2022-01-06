// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.CustomPIDController;
import frc.robot.CustomPIDSubsystem;
import frc.robot.Constants.Swerve.*;
import frc.robot.Constants.Swerve.PIDGains;

public class SwerveModule extends CustomPIDSubsystem {
  


  private CANSparkMax speedMotor;
  private WPI_TalonSRX angleMotor;
  private AnalogInput angleEncoder;
  private int moduleNumber;
  private double speed;

  /** Creates a new SwerveModule. */
  public SwerveModule(int moduleNumber, int speedMotorPort, int angleMotorPort, int angleEncoderPort) {
    // initialize a pid controller using the super constructor
    super(new CustomPIDController(PIDGains.KP, PIDGains.KI, PIDGains.KD));

    this.moduleNumber = moduleNumber;

    // NOTE: PIDController deals in native units

    // make continuous input enabled, since the motor can rotate past the extreme encoder count values
    getController().enableContinuousInput(0, AngleEncoder.CPR);

    speedMotor = new CANSparkMax(speedMotorPort, MotorType.kBrushless);
    angleMotor = new WPI_TalonSRX(angleMotorPort);
    angleEncoder = new AnalogInput(angleEncoderPort);

    speed = 0.0;
  }



  /**
   * Sets the speed of the wheel
   * @param metersPerSecond how fast the wheel should rotate in meters per second
   */
  public void setSpeed(double metersPerSecond) {
    // we use the velocity mode for setting motor speed, which requires motor counts per 100 ms as the speed parameter
    // double motorCountsPerDecisecond = metersPerSecond / getMetersPerCount() / 10; // TODO disabled for simplicity
    
    // TODO: added for armstrong
    // scale input for testing; armstrong can't easily use above 2 lines bcs no falcons
    // the scale is from 0 to MAX_WHEEL_SPEED usually, must be scaled to 0 to 1
    metersPerSecond /= Constants.Swerve.MAX_WHEEL_SPEED;
    
    // add scale factor for testing
    speedMotor.set(metersPerSecond * 0.2);
    speed = metersPerSecond;
  }



  /**
   * @param encoderCounts number of encoder counts
   * @return absolute encoder counts converted to radians
   */
  public static double nativeToRadians(double encoderCounts) {
    return encoderCounts * 2 * Math.PI / AngleEncoder.CPR;
  }



  /**
   * 
   * @param radians number of radians
   * @return radians converted to absolute encoder counts
   */
  public static double radiansToNative(double radians) {
    return radians / (2 * Math.PI) * AngleEncoder.CPR;
  }



  /**
   * The maximum angle the swerve module should have to travel to get to the right orientation
   * is 90 degrees, π / 2 radians, etc. If traveling to the setpoint would require exceeding this 
   * amount, it would be faster to rotate to the angle opposite the setpoint. This method checks
   * whether this adjustment is needed.
   * @return whether this setpoint needs adjustment
   */
  public boolean setpointAdjustmentNecessary() {
    double currentSetpoint = getSetpoint();
    double currentPosition = angleEncoder.pidGet();

    // subtract the current position from the current setpoint to obtain the angle difference 
    // the easiest way to determine whether the angle difference exceeds 1/4 a rotation is to test 
    // whether the cosine of the angle is negative after converting the angle difference to radians
    // this method makes it easier to handle the continuous nature of angles
    double absAngleDifference = nativeToRadians(Math.abs(currentSetpoint - currentPosition));

    if (Math.cos(absAngleDifference) < 0) {
      return true;
    }
    
    // if the angle difference was under a quarter turn, the setpoint doesn't need to change 
    return false;
  }



  /**
   * If the setpoint needs to be adjusted so the module takes a faster route to an angle,
   * this method can readjust the setpoint to be the opposite angle
   */
  public void flipSetpoint() {
    double currentSetpoint = getSetpoint();
    double halfEncoderCircle = AngleEncoder.CPR / 2;

    // we need to flip the setpoint by half a circle, but we need to keep it within the range of
    // valid angle encoder values, so we make checks to determine whether to add or subtract a half
    // circle
    if (currentSetpoint < halfEncoderCircle) {
      setSetpoint(currentSetpoint + halfEncoderCircle);
    } else {
      setSetpoint(currentSetpoint - halfEncoderCircle);
    }
  }


  // TODO: add in getter methods
  public double getSpeed() {
    return speed;
  }



  @Override
  public void useOutput(double output, double setpoint) {
    
    if (this.moduleNumber == 1) {
      angleMotor.set(ControlMode.PercentOutput, -output);
    } else {
      angleMotor.set(ControlMode.PercentOutput, output); // TODO: disabled for testing
    }
    // TODO: why doesn't motor actually take fastest route
    // TODO: module should flip setpoint for self?
    SmartDashboard.putNumber("Module " + moduleNumber + " PID output", output);
  }



  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return angleEncoder.pidGet();
  }

  @Override
  public void periodic() {
    super.periodic();
    SmartDashboard.putNumber("Module " + this.moduleNumber + " setpoint", this.getSetpoint());
    SmartDashboard.putNumber("Module " + this.moduleNumber + " angle", this.angleEncoder.pidGet());
    SmartDashboard.putBoolean("Module " + this.moduleNumber + " at setpoint", this.getController().atSetpoint());
    SmartDashboard.putNumber("Module " + this.moduleNumber + " speed", this.speedMotor.get());
  }
}
