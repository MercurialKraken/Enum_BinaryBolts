// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.RobotState.BotState;
import frc.robot.subsystems.Arm;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class HangerAndArm extends ParallelCommandGroup {
  /** Creates a new HangerAndArm. */
  BotState botState;
  Arm arm;
  public HangerAndArm(Arm arm, BotState botState) {
    this.arm = arm;
    this.botState = botState;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new InstantCommand(()->arm.enumArm(botState.armState)));
    addCommands(new InstantCommand(()->arm.enumHanger(botState.hangerState)));
  }
}
