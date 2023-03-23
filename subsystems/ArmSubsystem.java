// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.motorcontrol.Spark;

public class ArmSubsystem extends SubsystemBase {
  private final Spark leftIntake = new Spark(9);
  private final Spark rightIntake = new Spark(8);
  private final Solenoid m_solenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 0);
  private final Compressor compressor = new Compressor(PneumaticsModuleType.CTREPCM);

  private Timer timerA = new Timer();
  private Timer timerCompressor = new Timer();
  boolean armOpenedOnce;
  boolean buttonPressed;
  
  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
    rightIntake.setInverted(true);
    leftIntake.setInverted(false);
    armOpenedOnce = false;
    buttonPressed = false;
  }

  @Override
  public void periodic() {
    timerCompressor.reset();
    timerCompressor.start();
    timerA.reset();
    compressor.enableDigital();
    if (compressor.getPressureSwitchValue() == false) {
      compressor.disable();
    }
  }

  public void start(boolean solenoidButton, boolean inButton, boolean outButton, double velocity) {


    if (solenoidButton) {
      
      if (armOpenedOnce) {
        m_solenoid.set(false);
        armOpenedOnce = false;
      }
      else {
        m_solenoid.set(true);
        armOpenedOnce = true;
      }
    }



    if(outButton){
      set(velocity);
      buttonPressed = true;
    }
    else if (inButton) {
      set(-velocity);
      buttonPressed = true;
    }
    else {
      set(0);
      buttonPressed = false;
    }

    if (timerCompressor.hasElapsed(10)) {
      compressor.disable();
    }
  
  }

  public void setArm(boolean closeOrOpen) {
    m_solenoid.set(closeOrOpen);
  }

  public void intakeSet(double velocity, double timeout) {
    timerCompressor.start();
    if(timerA.hasElapsed(timeout)) {
      set(0);
    }
    else {
      set(velocity);
    }
  }

  public void set(double speed) {
    leftIntake.set(speed);
    rightIntake.set(speed);
  }


}
