/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.turret;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Vision.LightMode;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class TurnToAngle extends PIDCommand {

  private Turret m_turret;

  /**
   * Creates a new TurnToAngle.
   * 
   */
  // new PIDCommand(m_driveTrain.turnController, m_driveTrain,       90.0f,        m_driveTrain,       m_driveTrain);
  public TurnToAngle(Turret t) {
    super(
        // The controller that the command will use
        t.m_turnController,
        // This should return the measurement
        () -> t.getYaw(),
        // This should return the setpoint (can also be a constant)
        () -> (t.getYaw() + Vision.getTx()),
        // This uses the output
        output -> t.voltageDrive(output)
        );
    t.setLastTurnNumber(Vision.getTx());
    m_turret = t;
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
  }

  public TurnToAngle(Turret t, double setpoint) {
    super(
        // The controller that the command will use
        t.m_turnController,
        // This should return the measurement
        () -> t.getYaw(),
        // This should return the setpoint (can also be a constant)
        () -> (t.getYaw() + setpoint),
        // This uses the output
        output -> t.voltageDrive(output + Math.signum(output)*AutoConstants.TURN_KF)
        );
    t.setLastTurnNumber(setpoint);
    m_turret = t;
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
  }

  @Override
  public void initialize(){
    Vision.setLedMode(LightMode.eOn);
  }
  // Returns true when the command should end.

  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }

  @Override
  public void end(boolean interrupted) {
    m_turret.voltageDrive(0);
  }
}