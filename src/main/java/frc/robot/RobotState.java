// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Arm.ArmState;
import frc.robot.subsystems.Arm.GrabberState;
import frc.robot.subsystems.Arm.HangerState;

/** Add your docs here. */
public class RobotState extends SubsystemBase 
{

    private Arm m_arm;
    private BotState robotState;

    public RobotState(Arm arm)
    {
        m_arm = arm;

    }

    public void setBotState (BotState botState)
    {
        robotState = botState;
    }

    public BotState getBotState ()
    {
        return robotState;
    }

    public enum BotState
    {
        INTAKE_CONE(ArmState.CONE_LOW, HangerState.CONE_LOW, GrabberState.CONE_INTAKE, true),
        INTAKE_CUBE(ArmState.CUBE_LOW, HangerState.CUBE_LOW, GrabberState.CUBE_INTAKE, false),
        LOW_CONE(ArmState.CONE_LOW, HangerState.CONE_LOW, GrabberState.CONE_OUTTAKE, true),
        LOW_CUBE(ArmState.CUBE_LOW, HangerState.CUBE_LOW, GrabberState.CUBE_OUTTAKE, false),
        MID_CONE(ArmState.CONE_MID, HangerState.CONE_MID, GrabberState.CONE_OUTTAKE, true),
        MID_CUBE(ArmState.CUBE_MID, HangerState.CUBE_MID, GrabberState.CUBE_OUTTAKE, false),
        CONE_HIGH(ArmState.CONE_HIGH, HangerState.CONE_HIGH, GrabberState.CONE_OUTTAKE, true),
        CUBE_HIGH(ArmState.CUBE_HIGH, HangerState.CUBE_HIGH, GrabberState.CUBE_OUTTAKE, false),
        DEFAULT(ArmState.ZERO, HangerState.ZERO, GrabberState.ZERO, false);

        public ArmState armState;
        public HangerState hangerState;
        public GrabberState grabberState;
        public boolean isCone;

        private BotState(ArmState armState, HangerState hangerState, GrabberState grabberState, boolean isCone)
        {
            this.armState = armState;
            this.hangerState = hangerState;
            this.grabberState = grabberState;
            this.isCone = isCone;
        }


    }

}
