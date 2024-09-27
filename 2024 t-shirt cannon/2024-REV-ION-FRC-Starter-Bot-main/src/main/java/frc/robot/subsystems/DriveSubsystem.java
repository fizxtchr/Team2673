// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {
  private MecanumDrive m_robotDrive;

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    CANSparkMax frontLeft = new CANSparkMax(DriveConstants.kFrontLeftChannel, MotorType.kBrushless);
    CANSparkMax rearLeft = new CANSparkMax(DriveConstants.kRearLeftChannel, MotorType.kBrushless);
    CANSparkMax frontRight =
        new CANSparkMax(DriveConstants.kFrontRightChannel, MotorType.kBrushless);
    CANSparkMax rearRight = new CANSparkMax(DriveConstants.kRearRightChannel, MotorType.kBrushless);

    frontLeft.setSmartCurrentLimit(DriveConstants.kSmartCurrentLimit);
    frontRight.setSmartCurrentLimit(DriveConstants.kSmartCurrentLimit);
    rearLeft.setSmartCurrentLimit(DriveConstants.kSmartCurrentLimit);
    rearRight.setSmartCurrentLimit(DriveConstants.kSmartCurrentLimit);

    frontRight.setInverted(true);
    rearRight.setInverted(true);

    m_robotDrive = new MecanumDrive(frontLeft, rearLeft, frontRight, rearRight);
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block

  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   * @param rateLimit Whether to enable rate limiting for smoother control.
   */
  public void drive(double xSpeed, double ySpeed, double rot) {

    m_robotDrive.driveCartesian(xSpeed, ySpeed, rot);
  }
}
