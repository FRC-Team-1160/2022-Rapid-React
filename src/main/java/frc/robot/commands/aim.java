package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Vision;
/**don't know how this will work */
public class Aim extends CommandBase{
    private Vision m_vision;
    private double m_input;
    float kp = 0.0f;
    float tx = m_vision.get("tx").getDouble(0.0);

    public Aim(Vision m_vision, float tx, float kp){
        float steering_adjust = 0.0f;
        float heading_error = -tx;
        float min_command = 0.0f;
        /** makes sure that robot at least turns certain amout*/ 
        if (tx > 1.0){
            steering_adjust = kp * heading_error - min_command;
        }else{
            steering_adjust = kp * heading_error + min_command;
        }
        /** adjusts robot to turn */
        
        left_command += steering_adjust;
        right_command -= steering adjust;
        /**change inputs to rotate the bot
         * in container code where tank moves by left command and right command
         */


    }


    @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_vision.Aim(tx, kp);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

}
