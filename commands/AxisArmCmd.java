package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Axis;

public class AxisArmCmd extends CommandBase {
  private final Axis m_axis;
  XboxController m_xboxController = new XboxController(0);
  /** Creates a new ElevatorMovementCmd. */
  public AxisArmCmd(Axis m_axis, XboxController m_xboxController) {
    this.m_axis = m_axis;
    this.m_xboxController = m_xboxController;
    addRequirements(m_axis);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_axis.move(m_xboxController.getRightY());
    if (m_xboxController.getRightY() > 0.8 || m_xboxController.getRightY() < -0.8) {
      m_axis.move(m_xboxController.getRightY());
    }
    else if (m_xboxController.getRightY() < 0.8 || m_xboxController.getRightY() > -0.8) {
      m_axis.move(0);
    }
    m_axis.setLimit(0.25);
    

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_axis.move(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
