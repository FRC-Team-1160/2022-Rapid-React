// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commandgroups;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandGroupBase;
import frc.robot.commands.shoot.ShooterControl;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;

public class ManualFire extends CommandGroupBase {
  /** Add your docs here. */
  private Shooter m_shooter;
  private Intake m_intake;
  private Turret m_turret;
  public ManualFire(Shooter shooter, Intake intake, Turret turret) {
    m_shooter = shooter;
    m_intake = intake;
    m_turret = turret;
    /*new ParallelCommandGroup(
        new ShooterControl(m_shooter, -0.3 * 12).withTimeout(3.2),
        new SequentialCommandGroup(
          new IntakeControl(m_intake, -0.4 * 12).withTimeout(0.2),
          new MiddleIndexerControl(m_intake, -0.6 * 12).withTimeout(0.2),
          new MiddleIndexerControl(m_intake, 0 * 12).withTimeout(1.6),
          new FinalIndexerControl(m_intake, -0.65 * 12).withTimeout(0.7)
        )
      )*/

  }
  @Override
  public void addCommands(Command... commands) {
    // TODO Auto-generated method stub
    
  }
}
