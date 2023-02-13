// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import javax.swing.text.StyledEditorKit.BoldAction;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
  TalonFX arm;
  TalonFX hanger;
  double pos,pos1;
  double increment,increment1;
  CANSparkMax Intake;
  Solenoid SolGrip;
  Compressor comp;
  I2C.Port i2c;
  ColorSensorV3 m_colorSensor;
  ColorMatch m_colorMatcher;
  CANCoder CancoderArm;
  boolean flagReset= false;
  


  /** Creates a new Arm. */
  public Arm() {
    this.arm =new TalonFX(1);
    this.arm.configFactoryDefault();
    this.arm.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0,30);
    this.arm.setSensorPhase(true);
    this.arm.setInverted(true);
    this.arm.configNominalOutputForward(0);
    this.arm.configNominalOutputReverse(0);
    this.arm.configPeakOutputForward(1);
    this.arm.configPeakOutputReverse(-1);
    this.arm.configAllowableClosedloopError(0,0,30);
    this.arm.config_kF(0, 0);
    this.arm.config_kP(0, 2);
    this.arm.config_kI(0, 0);
    this.arm.config_kD(0, 12);

    this.hanger = new TalonFX(3);
    this.hanger.configFactoryDefault();
    this.hanger.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0,30);
    this.hanger.setSensorPhase(true);
    this.hanger.setInverted(false);
    this.hanger.configNominalOutputForward(0);
    this.hanger.configNominalOutputReverse(0);
    this.hanger.configPeakOutputForward(0.05);
    this.hanger.configPeakOutputReverse(-0.05);
    this.hanger.configAllowableClosedloopError(0,0,30);
    this.hanger.config_kF(0, 0); 
    this.hanger.config_kP(0, 0.15);
    this.hanger.config_kI(0, 0);
    this.hanger.config_kD(0, 12);

    this.CancoderArm = new CANCoder(10);
    this.CancoderArm.configFactoryDefault();
    this.CancoderArm.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
    
    this.arm.setSelectedSensorPosition(33.54-CancoderArm.getAbsolutePosition()*2275.5);
    this.hanger.setSelectedSensorPosition(0);

    // SolGrip = new Solenoid(PneumaticsModuleType.CTREPCM, 1);
    // comp = new Compressor(PneumaticsModuleType.CTREPCM);
    // Intake = new CANSparkMax(2, MotorType.kBrushless);

    i2c = I2C.Port.kMXP;
    m_colorSensor = new ColorSensorV3(i2c);
    m_colorMatcher = new ColorMatch();
    // comp.enableDigital();

    // this.Intake.setInverted(true);

    
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Position", pos);
    SmartDashboard.putNumber("Power", this.hanger.getMotorOutputPercent());
    SmartDashboard.putNumber("Position1", pos1);
    SmartDashboard.putNumber("Current",this.hanger.getStatorCurrent());
    SmartDashboard.putNumber("color sensor Proximity", m_colorSensor.getProximity());
    SmartDashboard.putNumber("arm Sensor Position", this.arm.getSelectedSensorPosition());
    SmartDashboard.putNumber("hanger Sensor Position", this.hanger.getSelectedSensorPosition());
    SmartDashboard.putBoolean("limit", homeSensorCheck());
    SmartDashboard.putNumber("Cancoder Arm", CancoderArm.getAbsolutePosition());
    // if(m_colorSensor.getProximity()>80){
    //   SolGrip.set(true);
    //   Intake(0);
    // }
    // SmartDashboard.putNumber("Intake Current", Intake.getOutputCurrent());
    // This method will be called once per scheduler run
  }
  public void armUp(boolean is_up) {
    if(is_up == false)
    return;
    else if(is_up == true)
    pos = 400;
    increment = ((pos/(100*3.22))*2048)*50;
   
    this.arm.set(TalonFXControlMode.Position,increment);
   }

   public void ManualarmUp(boolean manualis_up) {
    if(manualis_up == false)
    return;
    else if(manualis_up == true)
    pos +=1;
    increment = ((pos/(100*4))*2048)*50;
    // if (pos >= 500){
    //   pos = 500;
    //   }
    this.arm.set(TalonFXControlMode.Position,increment);
   }

  //  public void setAutoTarget(double Target) 
  //  {
  //  pos = Target;
  //  increment = ((pos/(100*3.22))*2048)*50;
  //  this.arm.set(TalonFXControlMode.Position,increment);
  //  }
   public void HomePostion(){
    this.arm.set(TalonFXControlMode.Position,0);
   }

   public void MoveArm(double degrees){
    this.arm.set(TalonFXControlMode.Position, this.arm.getSelectedSensorPosition() + CountsFromArmCancoder(degrees) );
   }
   


   public void armMiddle(boolean arm_middle) {
    if(arm_middle == false)
    return;
    else if(arm_middle == true)
    if(pos1<250){
      pos = 242;
      increment = ((pos/(100*3.22))*2048)*50;
      increment1 = ((pos1/15)*2048)*4;
    }
    else if(pos1>250){
      pos = 242;
      pos1 = 274;
      increment = ((pos/(100*3.22))*2048)*50;
      increment1 = ((pos1/15)*2048)*4;
    }
    
    this.arm.set(TalonFXControlMode.Position,increment);
    this.hanger.set(TalonFXControlMode.Position,increment1);
   }

   public void armDown(boolean is_down) {
     if(is_down == false)
     return;
     else if(is_down == true)
     pos = 30;
     increment = ((pos/(100*3.22))*2048)*50;
     
     this.arm.set(TalonFXControlMode.Position,increment);
    }

    public void ManualarmDown(boolean manualis_down) {
      if(manualis_down == false)
      return;
      else if(manualis_down == true)
      pos -=1;
      increment = ((pos/(100*4 ))*2048)*50;
      if (pos <= 0){
        pos = 0;
        }
      this.arm.set(TalonFXControlMode.Position,increment);
     }

    public void ManualHangerUp(boolean is_up) {
      if(is_up == false)
      return;
      else if(is_up == true)
      pos1 +=1;
      increment1 = ((pos1/9)*2048)*4;
      // if (pos1 >= 420){
      //   pos1 = 420;
      //   }
      this.hanger.set(TalonFXControlMode.Position,increment1);
     }

     public void HangerUp(boolean is_up) {
      if(is_up == false)
      return;
      else if(is_up == true)
     
      this.hanger.set(TalonFXControlMode.Position,330000);
     }

     public double Position(){
      return pos;
     }
   
     public void ManualHangerDown(boolean is_down) {
       if(is_down == false)
       return;
       else if(is_down == true)
       pos1 -= 1;
       increment1 = ((pos1/9)*2048)*4;
      //  if (pos1 <= 0){
      //    pos1 = 0;
      //    }
       this.hanger.set(TalonFXControlMode.Position,increment1);
      }

      public void HangerDown(boolean is_down) {
        if(is_down == false)
        return;
        else if(is_down == true)
      
        this.hanger.set(TalonFXControlMode.Position,0);
       }

       public boolean homeSensorCheck(){
        if(hanger.isRevLimitSwitchClosed() == 1){
          // this.hanger.setSelectedSensorPosition(0);
          return true;
        }
        else{
          return false;
        }
       }

      public void ZeroArm(){
        pos=0;
        pos1=0;
        this.arm.set(TalonFXControlMode.Position, pos);
        this.hanger.set(TalonFXControlMode.Position,pos1);
       }

       public void ResetHanger(boolean resetflag, boolean pointflag){
        while(resetflag == true){
        if(hanger.isRevLimitSwitchClosed() == 1 && !homeSensorCheck()==false){
          this.hanger.set(TalonFXControlMode.Position,30000 );
         }
     else if(hanger.isRevLimitSwitchClosed() == 0){    
           this.hanger.set(TalonFXControlMode.Velocity, -1000);
           pointflag=true;
          SmartDashboard.putString("output", "4");
            while(hanger.isRevLimitSwitchClosed() == 1 && pointflag == true){
              this.hanger.set(TalonFXControlMode.Velocity, 0);
              this.hanger.setSelectedSensorPosition(0);
              SmartDashboard.putString("output", "5");
              return;
             
            }
        }
       }
       return;
    }
      
      public double CountsFromArmCancoder(double CancoderArmValue){
        return CancoderArmValue*2275.5;
      }

      // public void Intake(double grippingspeed){
      //   Intake.set(grippingspeed); 
      // }

      // public void Grip(boolean Gripped){
      //   SolGrip.set(Gripped);
      // }



      // public void IntakeType(boolean intake_type){
      //   if(intake_type==true){
      //   if(m_colorSensor.getProximity()>80){
      //     Intake.set(0);
      //   }
      //  }else if(intake_type==false){
      //     SolGrip.toggle();
      //   }

      public void enumArm(ArmState armState)
      {
        pos = armState.armSetpoint;
        increment = ((pos/(100*3.22))*2048)*50;
        this.arm.set(TalonFXControlMode.Position,increment);
      }



      public void enumGrabber(GrabberState grabberState)
      {
        //Intake.set(grabberState.grabberPercentOutput); //compliant wheels
        //SolGrip.set(grabberState.grabberPercentOutput>0); //solenoid based grabber
      }

      
      public enum ArmState
      {
        ZERO(Constants.ArmConstants.DEFAULT),
        CUBE_LOW(Constants.ArmConstants.CUBE_LOW),
        CUBE_MID(Constants.ArmConstants.CUBE_MID),
        CUBE_HIGH(Constants.ArmConstants.CUBE_HIGH),
        CONE_LOW(Constants.ArmConstants.CONE_LOW),
        CONE_MID(Constants.ArmConstants.CONE_MID),
        CONE_HIGH(Constants.ArmConstants.CONE_HIGH);

        public final int armSetpoint;
        private ArmState(int setpoint)
        {
          this.armSetpoint = setpoint;
        }
      }

      public enum HangerState
      {
        ZERO(Constants.HangerConstants.DEFAULT),
        CUBE_LOW(Constants.HangerConstants.CUBE_LOW),
        CUBE_MID(Constants.HangerConstants.CUBE_MID),
        CUBE_HIGH(Constants.HangerConstants.CUBE_HIGH),
        CONE_LOW(Constants.HangerConstants.CONE_LOW),
        CONE_MID(Constants.HangerConstants.CONE_MID),
        CONE_HIGH(Constants.HangerConstants.CONE_HIGH);

        public final int hangerSetpoint;
        private HangerState(int setpoint)
        {
          this.hangerSetpoint = setpoint;
        }
      }

      public enum GrabberState
      {
        ZERO(Constants.GrabberConstants.STOP),
        CONE_INTAKE(Constants.GrabberConstants.INTAKE_CONE),
        CONE_OUTTAKE(Constants.GrabberConstants.OUTTAKE_CONE),
        CUBE_INTAKE(Constants.GrabberConstants.INTAKE_CUBE),
        CUBE_OUTTAKE(Constants.GrabberConstants.OUTTAKE_CUBE);

        public final double grabberPercentOutput;
        private GrabberState(double p_output)
        {
          this.grabberPercentOutput = p_output;
        }
      }


    }




