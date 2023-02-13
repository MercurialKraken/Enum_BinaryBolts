// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotState.BotState;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.HangerSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class UniversalCommand extends SequentialCommandGroup {
  /** Creates a new UniversalCommand. */
  BotState botState;
  Arm arm;
  public UniversalCommand(ArmSubsystem armSubsystem, HangerSubsystem hangerSubsystem, GrabberSubsystem grabberSubsystem, BotState botState) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new HangerAndArm(armSubsystem, hangerSubsystem, botState));
    addCommands(new InstantCommand(()->grabberSubsystem.enumGrabber(botState.grabberState)));
  }
}
