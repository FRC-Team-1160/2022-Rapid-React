package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;

//Subsystems
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Turret;

//Commands
import frc.robot.commands.climb.ClimbControl;
import frc.robot.commands.drive.Drive;
import frc.robot.commands.turret.TurretControl;
import frc.robot.commands.intake.MiddleIndexerControl;
import frc.robot.commands.intake.FinalIndexerControl;
import frc.robot.commands.intake.IntakeControl;
import frc.robot.commands.shoot.ShooterControl;
/*
import frc.robot.commands.vision.LimelightCameraToggle;
import frc.robot.commands.vision.LimelightLightToggle;
import frc.robot.commands.vision.LimelightSnapshotToggle;
import frc.robot.commands.vision.LimelightStreamToggle;
*/

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    // The robot's subsystems
    public final DriveTrain m_driveTrain = DriveTrain.getInstance(); 
    public final Shooter m_shooter = Shooter.getInstance();
    public final Intake m_intake = Intake.getInstance();
    public final Climber m_climber = Climber.getInstance();
    public final Turret m_turret = Turret.getInstance();
  
    // The driver's controller
    private Joystick m_mainStick = new Joystick(OIConstants.mainStickPort);
  
    // Secondary controllers
    private Joystick m_firstStick = new Joystick(OIConstants.firstStickPort);
    private Joystick m_secondStick = new Joystick(OIConstants.secondStickPort);

    /**
     * The container for the robot.  Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {

      // Configure the button bindings
      configureButtonBindings();
  
      // Configure default commands
      m_driveTrain.setDefaultCommand(new RunCommand(
        () -> m_driveTrain.tankDrive(m_mainStick.getRawAxis(1), m_mainStick.getRawAxis(4), m_mainStick.getRawAxis(2), m_mainStick.getRawAxis(3)),
        m_driveTrain)
      );

      m_climber.setDefaultCommand(new RunCommand(
        () -> m_climber.climbControl((0.5) * m_secondStick.getRawAxis(1) * 12),
        m_climber)
      );
    }
  
    /**
     * Use this method to define your button->command mappings.  Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
     * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    
    private void configureButtonBindings() {
      
      //Config for main stick
      new JoystickButton(m_mainStick, Button.kB.value)
        .whileHeld(
          new IntakeControl(m_intake, 0.5 * 12)
        );
      
      new JoystickButton(m_mainStick, 12)
        .whileHeld(
          new IntakeControl(m_intake, -0.4 * 12)//-0.3,-0.5
        );

      //Config for first stick
      new JoystickButton(m_mainStick, Button.kX.value)
        .whileHeld(
          new ShooterControl(m_shooter, -0.41 * 12)
        );
      
      new JoystickButton(m_firstStick, 2)
        .whileHeld(
          new MiddleIndexerControl(m_intake, 0.5 * 12)
        );
      
      new JoystickButton(m_firstStick, 2)
        .whileHeld(
          new MiddleIndexerControl(m_intake, -0.65 * 12)
        );
      
      new JoystickButton(m_firstStick, 3)
        .whileHeld(
          new FinalIndexerControl(m_intake, 0.5 * 12)
        );
    
      new JoystickButton(m_firstStick, 3)
        .whileHeld(
          new FinalIndexerControl(m_intake, -0.65 * 12)
        );
      
      new JoystickButton(m_firstStick, 9)
        .whenPressed(
          new TurretControl(m_turret, 0.5 * 12)
        );

      new JoystickButton(m_firstStick, 9)
        .whenPressed(
          new TurretControl(m_turret, -0.65 * 12)
        );

       // Run Shooter Mid Speed
       new JoystickButton(m_mainStick, 3)     // it was squared preserving sign so these are the true values from before
       .whileHeld(
         new ShooterControl(m_shooter, -0.5 * 12) // 0.5184 // -0.6
       );

      // Run Shooter Low Speed
      new JoystickButton(m_mainStick, 2)
       .whileHeld(
         new ShooterControl(m_shooter, -0.35 * 12) // 0.4096 // -0.38
       );

      // Run Shooter High Speed
      new JoystickButton(m_mainStick, 1)
       .whileHeld(
         new ShooterControl(m_shooter, -1 * 12)  // 0.81
       );
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */

    /*
    public Command getAutonomousCommand() {
      return m_chooser.getSelected().withTimeout(15);
    }
    */
    
}