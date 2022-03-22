/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.turret;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Vision.LightMode;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class TurnToAngle2 extends CommandBase {

  private Turret m_turret;
  private double m_input;
  private boolean direction; //false is left. true is right

  /**
   * Creates a new TurnToAngle.
   * 
   */
  // new PIDCommand(m_driveTrain.turnController, m_driveTrain,       90.0f,        m_driveTrain,       m_driveTrain);
  public TurnToAngle2(Turret t, double input) {
    m_turret = t;
    m_input = input;
    addRequirements(m_turret);
  }

  @Override
  public void initialize(){
    Vision.setLedMode(LightMode.eOn);
    if (Vision.getTx() >= 0) {
      direction = true;
    } else {
      direction = false;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double value = m_input;
    if (Math.abs(Vision.getTx()) < 3) {
      value = m_input/2;
    } else {
      value = m_input;
    }
    if (direction) {
      m_turret.turretControl(value);
    } else {
      m_turret.turretControl(-value);
    }
  }

  // Returns true when the command should end.

  @Override
  public boolean isFinished() {
    return Math.abs(Vision.getTx()) < 1.0;
  }

  @Override
  public void end(boolean interrupted) {
    m_turret.turretControl(0);
  }
}