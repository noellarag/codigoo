// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Axis extends SubsystemBase {
  private final CANSparkMax m_axisMotor = new CANSparkMax(3, MotorType.kBrushless);
  private final Encoder encoder = new Encoder(6, 7, false, EncodingType.k4X);

  public Axis() {
    m_axisMotor.setInverted(true);
    m_axisMotor.setIdleMode(IdleMode.kBrake);
  }

  @Override
  public void periodic() {
    encoder.reset();
    // This method will be called once per scheduler run
  }

  public void move(double speed) {
    m_axisMotor.set(speed / 10);
  }

  public void setLimit(double limit) {
    if (m_axisMotor.get() > limit) {
      m_axisMotor.set(limit);
    }
    else if (m_axisMotor.get() < -limit) {
      m_axisMotor.set(-limit);
    }
  }

  public void reset() {
   
  }
  public void stop() {
    m_axisMotor.stopMotor();
  }
}
