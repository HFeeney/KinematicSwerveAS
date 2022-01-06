// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Swerve;
import frc.robot.Constants.Swerve.*;

public class SwerveDrivetrain extends SubsystemBase {

  private SwerveDriveKinematics mSwerveDriveKinematics;
  private SwerveModule[] swerveModules;
  private boolean fieldCentricActive = true;
  private AHRS gyro;
  SwerveModuleState[] moduleStates;
  private Pose2d mSwervePose;
  private SwerveDriveOdometry mSwerveDriveOdometry; // untested



  /** Creates a new SwerveDrivetrain. */
  public SwerveDrivetrain() {
    mSwerveDriveKinematics = new SwerveDriveKinematics(
      ModulePositions.FRONT_LEFT,
      ModulePositions.FRONT_RIGHT,
      ModulePositions.BACK_LEFT,
      ModulePositions.BACK_RIGHT
    );

    swerveModules = new SwerveModule[Swerve.NUM_MODULES];
    
    for (int i = 0; i < Swerve.NUM_MODULES; i++) {
      swerveModules[i] = new SwerveModule(
        i, 
        Ports.SPEED_MOTORS[i],
        Ports.ANGLE_MOTORS[i],
        Ports.ANGLE_ENCODERS[i]
      );
      
      swerveModules[i].enable();
    }

    gyro = new AHRS(Ports.GYRO);
    gyro.reset();
    
    // construct swerve pose with values from constants as starting point
    mSwervePose = new Pose2d(Swerve.INITAL_POSE.getX(), Swerve.INITAL_POSE.getY(), Swerve.INITAL_POSE.getRotation());
    // mSwerveDriveOdometry = new SwerveDriveOdometry(mSwerveDriveKinematics, new Rotation2d(getGyroAngle()), mSwervePose);
  }



  /**
   * This method makes the swerve drivetrain drive
   * @param forward the forward velocity of the robot
   * @param strafe the left velocity of the robot
   * @param rotate the counter-clockwise angular velocity of the robot
   */
  public void drive(double forward, double strafe, double rotate) {

    // only drive if there is input above a certain deadband
    if (
      Math.abs(forward) > Swerve.JOYSTICK_DEADBAND ||
      Math.abs(strafe) > Swerve.JOYSTICK_DEADBAND ||
      Math.abs(rotate) > Swerve.JOYSTICK_DEADBAND
    ) {
      
      // transform the input forward strafe and rotate for field centric drive
      if (fieldCentricActive) {
        // convert the gyro angle from clockwise degrees to counterclockwise radians
        double gyroAngle = getGyroAngle();

        double tempForward = forward * Math.cos(gyroAngle) + strafe * Math.sin(gyroAngle);
        strafe = -forward * Math.sin(gyroAngle) + strafe * Math.cos(gyroAngle);
        forward = tempForward;
      }
      
      // scale the max speeds by factors of the corresponding parameters
      ChassisSpeeds input = new ChassisSpeeds(
        forward * Swerve.MAX_WHEEL_SPEED,
        strafe * Swerve.MAX_WHEEL_SPEED, 
        rotate * Swerve.MAX_ANGULAR_SPEED
      );

      // use the kinematics class to turn robot velocities into wheel speeds and angles
      // also normalize the speeds so they do not exceed the current max wheel speed
      moduleStates = mSwerveDriveKinematics.toSwerveModuleStates(input);
      SwerveDriveKinematics.normalizeWheelSpeeds(moduleStates, Swerve.MAX_WHEEL_SPEED);

      for (int i = 0; i < Swerve.NUM_MODULES; i++) {
        // make the setpoint of the rotation pid the encoder amount of radians the kinematics class determined 
        // offset this amount of encoder counts by the offset corresponding to this swerve module
        // likewise set the speed equal to the value determined by the kinematics class
        double setpoint = SwerveModule.radiansToNative(moduleStates[i].angle.getRadians()) + AngleEncoder.OFFSETS[i];
        double speed = moduleStates[i].speedMetersPerSecond;

        
        swerveModules[i].setSetpoint(setpoint);
        swerveModules[i].setSpeed(speed);

        // check if that setpoint needs adjustment, flipping the setpoint and the speed of the module if it does
        if (swerveModules[i].setpointAdjustmentNecessary()) {
          swerveModules[i].flipSetpoint();
          swerveModules[i].setSpeed(-speed);
        }
      }
    } else {
      for (int i = 0; i < Swerve.NUM_MODULES; i++) {
        swerveModules[i].setSpeed(0);
      }
    }
  }


  
  /**
   * @return gyro angle but in radians
   */
  public double getGyroAngle() {
    return -Math.toRadians(gyro.getAngle());
  }



  /**
   * resets the gyro to 0 degrees
   */
  public void resetGyro() {
    gyro.reset();
  }



  /**
   * @return Pose2d object representing the robot's position and rotation
   */
  public Pose2d getSwervePose() {
    return mSwervePose;
  }



  public void setFieldCentricActive(boolean fieldCentricActive) {
    this.fieldCentricActive = fieldCentricActive;
  }



  public boolean getFieldCentricActive() {
    return this.fieldCentricActive;
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // mSwervePose = mSwerveDriveOdometry.update(new Rotation2d(getGyroAngle()), moduleStates);
    for (int i = 0; i < this.swerveModules.length; i++) {
      SmartDashboard.putBoolean("Module " + i + " enabled", this.swerveModules[i].isEnabled());
    }

    SmartDashboard.putBoolean("Field centric active", this.getFieldCentricActive());
    SmartDashboard.putNumber("Gyro angle in Radians", this.getGyroAngle());
  }
}
