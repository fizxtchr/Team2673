// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {
  private CANSparkMax m_motor;

  // private double m_manualValue;

  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {
    // create a new SPARK MAX and configure it
    m_motor = new CANSparkMax(9, MotorType.kBrushed);
    m_motor.setInverted(false);
  }

  public void runClimber(double power) {
    m_motor.set(power);
  }

  public void stopClimber() {
    m_motor.set(0.0);
  }

  @Override
  public void periodic() { // This method will be called once per scheduler run
  }
}
