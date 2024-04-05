/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* 
Я опять что то пишу ну уже сверху вthe project.                                                               */
/*----------------------------------------------------------------------------*/


// Доска эс 2 левый соник: 420
// Доска эс 1 правый соник: 327
// Расстояние от стены у полки эс 1 левый соник: 252
// Расстояние между черными линиями левый соник: 452
// 652 
// 852
//
//




package frc.robot.commands;

import frc.robot.subsystems.ExampleSubsystem;


import org.opencv.photo.AlignExposures;

import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj2.command.CommandBase;



/**
 * An example command that uses an example subsystem.
 */
public class ExampleCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ExampleSubsystem m_subsystem;
  public static int state;
  public static int cylinderS1;
  public static int pyramideS1;
  public static int cubeS1;
  private long timeToWait;
  private long elapsedTime;
  

  
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ExampleCommand(ExampleSubsystem subsystem) {
    m_subsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
    
  }

  private void resetAll(){
    m_subsystem.resetRightEnc();
    m_subsystem.resetLeftEnc();
    m_subsystem.resetBackEnc();
    m_subsystem.resetYaw();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

     m_subsystem.resetYaw();
    cubeS1 = 0;
    pyramideS1 = 0;
    m_subsystem.setServoAngle(45);
    cylinderS1 = 0;
    state = 7;
    timeToWait = System.currentTimeMillis();
    
  }

  public Double[] getCords() {
    double R = m_subsystem.getRightEnc();
    double L = m_subsystem.getLeftEnc();
    double B = m_subsystem.getBackEnc();

    Double[] arr = new Double[3];
    arr[0] = -(R - L);
    arr[1] = ((R + L) * 0.5 - B);
    arr[2] = -(- R - L - B);
    return arr; 
  }


  public boolean resetLift1() {
    boolean reached = false;
    timeToWait = System.currentTimeMillis();
    m_subsystem.setLiftMotor(0.35);
    if(m_subsystem.getStart()) {
      m_subsystem.resetLiftEnc();
      m_subsystem.setLiftMotor(0);
      return true;
    }
    
    else {
      return false;
    }
  }
  public boolean resetLift2() {
    boolean reached = false;
    timeToWait = System.currentTimeMillis();
    m_subsystem.setLiftMotor(-0.35);
    if(m_subsystem.getStart()) {
      m_subsystem.resetLiftEnc();
      m_subsystem.setLiftMotor(0);
      reached = setEncValue(4362);
    }
    if(reached) {
      return true;
    }
    else {
      return false;
    }
  }

  public boolean setEncValue(double liftEncValue) {
    boolean reached = false;
    m_subsystem.setLiftMotor(-0.35);
    if(liftEncValue <= Math.abs(m_subsystem.getLiftEnc())) {
      m_subsystem.setLiftMotor(0);
      return true;
    }
    else {
      return false;
    }
  }


  // 2491 Пирамиды
  // 4362 Кубики  
  public boolean driveWithRSonic(double needDistance, double yaw) {
    double distance = m_subsystem.getRightSonicDistance();
    // boolean reached = distance < needDistance;
    double zSpeed = ( yaw - m_subsystem.getYaw())*0.016;
    double ySpeed  = (needDistance - distance)*0.01;

  
    if( Math.abs(ySpeed) > 0.2)
    {
      ySpeed = 0.2 * (Math.abs(ySpeed)/ ySpeed);
    }
    driveAxis(0, ySpeed, zSpeed);
    
    boolean reached = Math.abs(needDistance - distance) < 5;
    
    return reached;
  }

  public boolean driveWithRSonicPID(double needDistance, double yaw) {
    double distance = m_subsystem.getRightSonicDistance();
    // boolean reached = distance < needDistance;
    double zSpeed = ( yaw - m_subsystem.getYaw())*0.04;
    double ySpeed  = (needDistance - distance)*0.01;

  
    if( Math.abs(ySpeed) > 0.2)
    {
      ySpeed = 0.2 * (Math.abs(ySpeed)/ ySpeed);
    }
    driveAxis(0, ySpeed, zSpeed);
    
    boolean reached = Math.abs(needDistance - distance) < 5;
    
    return reached;
  }

  public boolean driveWithRSonicToBlackLine(double needDistance, double yaw) {
    double distance = m_subsystem.getRightSonicDistance();
    // boolean reached = distance < needDistance;
    double zSpeed = ( yaw - m_subsystem.getYaw())*0.04;
    double ySpeed  = (needDistance - distance)*0.01;

  
    if( Math.abs(ySpeed) > 0.2)
    {
      ySpeed = 0.2 * (Math.abs(ySpeed)/ ySpeed);
    }
    driveAxis(0, ySpeed, zSpeed);
    
    boolean reached = m_subsystem.getCobra(0) >= 1000 && m_subsystem.getCobra(1) >= 1000 && m_subsystem.getCobra(2) >= 1000 && m_subsystem.getCobra(3) >= 1000;
    
    return reached;
  }


  public boolean DriveWithSharps(double needDistance, double yaw) {
    double zSpeed = ( yaw - m_subsystem.getYaw())*0.0085;
    double distance = (m_subsystem.getLeftSharpDistance() + m_subsystem.getRightSharpDistance()) / 2;
    boolean reached = false;
    double xSpeed = (needDistance - distance) * 0.05;
    timeToWait = System.currentTimeMillis();
    if(Math.abs(xSpeed) > 0.2) 
    {
      xSpeed = 0.2 * (Math.abs(xSpeed) / xSpeed);
    }      
    


    driveAxis(xSpeed, 0, zSpeed);

    if( Math.abs(needDistance - distance) < 2 )
       {
      reached = System.currentTimeMillis() - elapsedTime >= 3000;
    }
    else
    {
      elapsedTime = System.currentTimeMillis();
    }
    if (reached)
    {

      driveAxis(0, 0, 0);
      return true;
    }

    
    return false;
  }

  public boolean DriveWithSharpsToBlackLine(double needDistance, double yaw) {
    double zSpeed = ( yaw - m_subsystem.getYaw())*0.0085;
    double distance = (m_subsystem.getLeftSharpDistance() + m_subsystem.getRightSharpDistance()) / 2;
    boolean reached = false;
    double xSpeed = (needDistance - distance) * 0.05;
    timeToWait = System.currentTimeMillis();
    if(Math.abs(xSpeed) > 0.2) 
    {
      xSpeed = 0.2 * (Math.abs(xSpeed) / xSpeed);
    }      
    


    driveAxis(xSpeed, 0, zSpeed);

    if( Math.abs(needDistance - distance) < 2 )
       {
      reached = m_subsystem.getCobra(0) >= 1000 && m_subsystem.getCobra(1) >= 1000 && m_subsystem.getCobra(2) >= 1000 && m_subsystem.getCobra(3) >= 1000;
    }
    else
    {
      elapsedTime = System.currentTimeMillis();
    }
    if (reached)
    {

      driveAxis(0, 0, 0);
      return true;
    }

    
    return false;
  }

  public boolean RotateToYaw90( double yaw) {
    double zSpeed = ( yaw - m_subsystem.getYaw())*0.0085;
    double distance = (m_subsystem.getLeftSharpDistance() + m_subsystem.getRightSharpDistance()) / 2;
    boolean reached = false;
    
    timeToWait = System.currentTimeMillis();
      
    


    driveAxis(0, 0, zSpeed);

    if( (Math.abs(yaw - m_subsystem.getYaw()))*0.0170 <= 1) //0.0085
    {
      reached = System.currentTimeMillis() - elapsedTime >= 3000;
    }
    else
    {
      elapsedTime = System.currentTimeMillis();
    }
    if (reached)
    {

      driveAxis(0, 0, 0);
      return true;
    }

    
    return false;
  }



  public boolean RotateToYaw( double yaw) {
    double zSpeed = ( yaw - m_subsystem.getYaw())*0.007  ;
    double distance = (m_subsystem.getLeftSharpDistance() + m_subsystem.getRightSharpDistance()) / 2;
    boolean reached = false;
    
    timeToWait = System.currentTimeMillis();
    if(zSpeed>=0.5){
      zSpeed=0.5;
    }
    if(zSpeed<=-0.5){
      zSpeed=-0.5;
    }
      
    

;
    driveAxis(0, 0, zSpeed);


    if( (Math.abs(yaw - m_subsystem.getYaw()))*0.004 <= 0.6) //0.0085
    {
      reached = System.currentTimeMillis() - elapsedTime >= 3000;
    }
    else
    {
      elapsedTime = System.currentTimeMillis();
    }
    if (reached)
    {

      driveAxis(0, 0, 0);
      return true;
    }

    
    return false;
  }
  


  public boolean driveWithLSonic(double needDistance, double yaw) {
    double distance = m_subsystem.getLeftSonicDistance();
    //boolean reached = distance < needDistance;
    double zSpeed = ( yaw - m_subsystem.getYaw())*0.0085; //0.0085
    double ySpeed  = (needDistance - distance)*0.01;

  
    if( Math.abs(ySpeed) > 0.2)
    {
      ySpeed = 0.2 * (Math.abs(ySpeed)/ ySpeed);
    }
    driveAxis(0, -ySpeed, zSpeed);
    
    boolean reached = Math.abs(needDistance - distance) < 5;
    
    return reached;
  }

  public boolean driveWithLSonicPID(double needDistance, double yaw) {
    double distance = m_subsystem.getLeftSonicDistance();
    //boolean reached = distance < needDistance;
    double zSpeed = ( yaw - m_subsystem.getYaw())*0.04 ; //0.0085
    double ySpeed  = (needDistance - distance)*0.01;
    

  
    if( Math.abs(ySpeed) > 0.2)
    {
      ySpeed = 0.2 * (Math.abs(ySpeed)/ ySpeed);
    }
    driveAxis(0, -ySpeed, zSpeed);
    
    boolean reached = Math.abs(needDistance - distance) < 5;
    
    return reached;
    
  }

  public boolean alignWithSharps(double needDistance) {
    double diffSharp = (m_subsystem.getRightSharpDistance() - m_subsystem.getLeftSharpDistance())*0.08; //0.08
    double distance = (m_subsystem.getLeftSharpDistance() + m_subsystem.getRightSharpDistance()) / 2;
    boolean reached = false;
    double xSpeed = (needDistance - distance) * 0.05;
    timeToWait = System.currentTimeMillis();
    if(Math.abs(xSpeed) > 0.2) 
    {
      xSpeed = 0.2 * (Math.abs(xSpeed) / xSpeed);
    }      

    if (Math.abs(diffSharp) > 0.2) //0.2 s obuchenia
    {
      diffSharp = 0.2 * (Math.abs(diffSharp) / diffSharp);
    }

    driveAxis(xSpeed, 0, -diffSharp);

    if( Math.abs(needDistance - distance) < 2 && (diffSharp < 0.2 && diffSharp > -0.2))
       {
      reached = System.currentTimeMillis() - elapsedTime >= 5000;
    }
    else
    {
      elapsedTime = System.currentTimeMillis();
    }
    if (reached)
    {

      driveAxis(0, 0, 0);
      return true;
    }
    return false;
  }

  public boolean alignWithSonics(double needDistance) {
    double diffSharp = (m_subsystem.getRightSonicDistance() - m_subsystem.getLeftSonicDistance())*0.08; //0.08
    double distance = (m_subsystem.getLeftSonicDistance() + m_subsystem.getRightSonicDistance()) / 2;
    boolean reached = false;
    double xSpeed = (needDistance - distance) * 0.05;
    timeToWait = System.currentTimeMillis();
    if(Math.abs(xSpeed) > 0.2) 
    {
      xSpeed = 0.2 * (Math.abs(xSpeed) / xSpeed);
    }      
    

    if (Math.abs(diffSharp) > 2) //0.2 s obuchenia
    {
      diffSharp = 2 * (Math.abs(diffSharp) / diffSharp);
    }

    driveAxis(xSpeed, 0, -diffSharp);

    if( Math.abs(needDistance - distance) < 10 && (diffSharp < 2 && diffSharp > -2))
       {
      reached = System.currentTimeMillis() - elapsedTime >= 5000;
    }
    else
    {
      elapsedTime = System.currentTimeMillis();
    }
    if (reached)
    {

      driveAxis(0, 0, 0);
      return true;
    }

    
    return false;
  }



    public boolean alignWithSharpsPID (double needDistance) {
    double diffSharp = (m_subsystem.getRightSharpDistance() - m_subsystem.getLeftSharpDistance())*0.085  ; //0.08
    double distance = (m_subsystem.getLeftSharpDistance() + m_subsystem.getRightSharpDistance()) / 2;
    boolean reached = false;
    double xSpeed = (needDistance - distance) * 0.05;
    timeToWait = System.currentTimeMillis();
    if(Math.abs(xSpeed) > 0.2) 
    {
      xSpeed = 0.2 * (Math.abs(xSpeed) / xSpeed);
    }      
    

    if (Math.abs(diffSharp) > 0.2) //0.2 s obuchenia
    {
      diffSharp = 0.2 * (Math.abs(diffSharp) / diffSharp);
    }

    driveAxis(xSpeed, 0, -diffSharp);

    if( Math.abs(needDistance - distance) < 2 && (diffSharp < 0.2 && diffSharp > -0.2))
       {
      reached = System.currentTimeMillis() - elapsedTime >= 3000;
    }
    else
    {
      elapsedTime = System.currentTimeMillis();
    }
    if (reached)
    {

      driveAxis(0, 0, 0);
      return true;
    }

    
    return false;
  }

  public Boolean cordDriver(double x, double y, double z) {
    Double[] cords = new Double[3];
    cords = getCords();

    Double xSpeed = (cords[0] - x)*0.0005;
    Double ySpeed = (cords[1] - y)*0.0005;
    Double zSpeed = (z - m_subsystem.getYaw())*0.009;

    driveAxis(xSpeed, ySpeed, zSpeed);

    return ((Math.abs(x - cords[0]) <= 1000 && Math.abs(y - cords[1]) <= 1000));
  }

  public Double[] getCordsYaw() {
    double R = m_subsystem.getRightEnc();
    double L = m_subsystem.getLeftEnc();
    double B = m_subsystem.getBackEnc();
  

    Double[] arr = new Double[3];
    arr[0] = -(R - L);
    arr[1] = ((R + L) * 0.5 - B);
    arr[2] = -(- R - L - B);
    return arr; 
  }
  
  public Boolean cordDriverOne(Double x, Double y, double z) {
    Double[] cords = new Double[3];
    cords = getCords();

    Double xSpeed = (cords[0] - x)*0.0005;
    Double ySpeed = (cords[1] - y)*0.0005;
    Double zSpeed = (z - m_subsystem.getYaw())*0.0085;

    driveAxis(xSpeed, ySpeed, zSpeed);

    return ((Math.abs(z - m_subsystem.getYaw()) <= 20));
  }

  private void driveAxis(double x, double y, double z) {
    m_subsystem.setRightMotor((x) - (y * 0.5) - (z / -1.5f));
    m_subsystem.setLeftMotor((-x) - (y * 0.5) - (z / -1.5f));
    m_subsystem.setBackMotor((0) + (y) - (z / -1.5f));
  }




 

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute()
  

  {
      
    switch (state)
     {
      case 0:
      timeToWait= System.currentTimeMillis();
      if(RotateToYaw(185)){
        state++;
      };
      break;
      case 1:
      driveAxis(0, 0, 0);
      if(System.currentTimeMillis()-timeToWait>=500){
        m_subsystem.resetYaw();
        state++;
      };
      break;
      case 2:
      timeToWait= System.currentTimeMillis();
      if(RotateToYaw(185)){
        state++;
      };
      break;
      case 3:
      driveAxis(0, 0, 0);
      if(System.currentTimeMillis()-timeToWait>=500){
        m_subsystem.resetYaw();
        state++;
      };
      break;
      case 4:
      timeToWait= System.currentTimeMillis();
      if(RotateToYaw(185)){
        state++;
      };
      break;
      case 5:
      driveAxis(0, 0, 0);
      if(System.currentTimeMillis()-timeToWait>=500){
        m_subsystem.resetYaw();
        state++;
      };
      break;
      case 6:
      timeToWait= System.currentTimeMillis();
      if(RotateToYaw(185)){
        state++;
      };
      break;
      case 7:
      driveAxis(0, 0, 0);
      if(System.currentTimeMillis()-timeToWait>=1500){
        m_subsystem.resetYaw();
        state++;
      };
      break;
      case 8:
      timeToWait=System.currentTimeMillis();
      if(DriveWithSharps(15, 0)){
        state++;
      };
      break;
      case 9:
      driveAxis(0, 0, 0);
      if(System.currentTimeMillis()-timeToWait>=500){
        m_subsystem.resetYaw();
        state++;
      };
      break;
      case 10:
      timeToWait=System.currentTimeMillis();
      if(alignWithSharps(12)){
        state++;
      };
      break;
      case 11:
      driveAxis(0, 0, 0);
      if(System.currentTimeMillis()-timeToWait>=500){
        m_subsystem.resetYaw();
        state++;
      };
      break;
      case 12:
      timeToWait=System.currentTimeMillis();
      if(RotateToYaw90(-94)){
        state++;
      
      }break;
      case 13:
      driveAxis(0, 0, 0);
      if(System.currentTimeMillis()-timeToWait>=500){
        m_subsystem.resetYaw();
        state++;
      };
      break;
      case 14:
      timeToWait=System.currentTimeMillis();
      if(DriveWithSharps(26, 0)){//можно поменять растояние до талого
        state++;
      }
      break;
      case 15:
      driveAxis(0, 0, 0);
      if(System.currentTimeMillis()-timeToWait>=500){
        m_subsystem.resetYaw();
        state++;
      };
      break;
      case 16:
      timeToWait=System.currentTimeMillis();
      if(driveWithRSonic(500, 0)){// nuzhno menyat'
        state++;

      };
      break;
      case 17:
      driveAxis(0, 0, 0);
      if(System.currentTimeMillis()-timeToWait>=500){
        m_subsystem.resetYaw();
        state++;
      };
      break;
      case 18:
      timeToWait=System.currentTimeMillis();
      if(DriveWithSharps(17, 0)){
        state++;
      };
      break;
      case 19:
      driveAxis(0, 0, 0);
      if(System.currentTimeMillis()-timeToWait>=500){
        m_subsystem.resetYaw();
        state++;
      };
      break;
      case 20:
      timeToWait=System.currentTimeMillis();
      if(driveWithRSonic(350, 0)){//tozhe nuzhno menyat'
        state++;
      };
      break;
      case 21:
      driveAxis(0, 0, 0);
      if(System.currentTimeMillis()-timeToWait>=500){
        m_subsystem.resetYaw();
        state++;
      };
      break;
      case 22:
      timeToWait=System.currentTimeMillis();
      if(RotateToYaw(180)){
        state++;
      };
      break;
      case 23:
      driveAxis(0, 0, 0);
      if(System.currentTimeMillis()-timeToWait>=500){
        m_subsystem.resetYaw();
        state++;
      };
      break;
      case 24:
      timeToWait=System.currentTimeMillis();
      if(DriveWithSharps(25, 0)){
        state++;
      };
      break;
      case 25:
      driveAxis(0, 0, 0);
      if(System.currentTimeMillis()-timeToWait>=500){
        m_subsystem.resetYaw();
        state++;
      };
      break;
      case 26:
      timeToWait=System.currentTimeMillis();
      if(RotateToYaw90(94)){
        state++;
      };
      break;
      case 27:
      driveAxis(0, 0, 0);
      if(System.currentTimeMillis()-timeToWait>=500){
        m_subsystem.resetYaw();
        state++;
      };
      break;
      case 28:
      timeToWait=System.currentTimeMillis();
      if(alignWithSharps(15)){
        state++;
      };
      break;
      case 29:
      driveAxis(0, 0, 0);
      if(System.currentTimeMillis()-timeToWait>=500){
        m_subsystem.resetYaw();
        state++;
      };
      break;
      case 30:
      timeToWait=System.currentTimeMillis();
      if(driveWithLSonic(1000, 0)){
        state++;
      };
      break;
      case 31:
      driveAxis(0, 0, 0);
      if(System.currentTimeMillis()-timeToWait>=500){
        m_subsystem.resetYaw();
        state++;
      };
      break;
      case 32:
      timeToWait=System.currentTimeMillis();
      if(DriveWithSharps(20, 0)){
        state++;
      };
      break;
      case 33:
      driveAxis(0, 0, 0);
      if(System.currentTimeMillis()-timeToWait>=500){
        m_subsystem.resetYaw();
        state++;
      };
      break;
      case 34:
      timeToWait=System.currentTimeMillis();
      if(alignWithSharps(15));{
        state++;
      };
      break;
      case 35:
      driveAxis(0, 0, 0);
      if(System.currentTimeMillis()-timeToWait>=500){
        m_subsystem.resetYaw();
        state++;
      };
      break;
      case 36:
      timeToWait=System.currentTimeMillis();
      if(RotateToYaw90(-94)){
        state++;
      };
      break;
      case 37:
      driveAxis(0, 0, 0);
      if(System.currentTimeMillis()-timeToWait>=500){
        m_subsystem.resetYaw();
        state++;
      };
      break;
      case 38:
      timeToWait=System.currentTimeMillis();
      if(driveWithRSonicPID(600, 0)){//nuzhno menyat'
        state++;
      };
      break;
      case 39:
      driveAxis(0, 0, 0);
      if(System.currentTimeMillis()-timeToWait>=500){
        m_subsystem.resetYaw();
        state++;
      };
      break;
      case 40:
      timeToWait=System.currentTimeMillis();
      if(DriveWithSharps(20, 0)){
        state++;
      };
      break;
      case 41:
      driveAxis(0, 0, 0);
      if(System.currentTimeMillis()-timeToWait>=500){
        m_subsystem.resetYaw();
        state++;
      };
      break;
      case 42:
      timeToWait=System.currentTimeMillis();
      if(alignWithSharps(15)){
        state++;
      };
      break;
      case 43:
      driveAxis(0, 0, 0);
      if(System.currentTimeMillis()-timeToWait>=500){
        m_subsystem.resetYaw();
        state++;
      };
      break;
      case 44:
      timeToWait=System.currentTimeMillis();
      if(RotateToYaw90(94)){
        state++;
      };
      break;
      case 45:
      driveAxis(0, 0, 0);
      if(System.currentTimeMillis()-timeToWait>=500){
        m_subsystem.resetYaw();
        state++;
      };
      break;
      case 46:
      timeToWait=System.currentTimeMillis();
      if(DriveWithSharps(12, 0)){
        state++;
      };
      break;
      case 47:
      driveAxis(0, 0, 0);
      if(System.currentTimeMillis()-timeToWait>=500){
        m_subsystem.resetYaw();
        state++;
      };
      break;
      case 48:
      timeToWait= System.currentTimeMillis();
      if(RotateToYaw(187)){
        state++;
      };
      break;
      case 49:
      driveAxis(0, 0, 0);
      if(System.currentTimeMillis()-timeToWait>=500){
        m_subsystem.resetYaw();
        state++;
      };
      break;
      case 50:
      timeToWait= System.currentTimeMillis();
      if(RotateToYaw(187)){
        state++;
      };
      break;
      case 51:
      driveAxis(0, 0, 0);
      if(System.currentTimeMillis()-timeToWait>=500){
        m_subsystem.resetYaw();
        state++;
      };
      break;
      case 52:
      timeToWait= System.currentTimeMillis();
      if(RotateToYaw(187)){
        state++;
      };
      break;
      case 53:
      driveAxis(0, 0, 0);
      if(System.currentTimeMillis()-timeToWait>=500){
        m_subsystem.resetYaw();
        state++;
      };
      break;
      case 54:
      timeToWait=System.currentTimeMillis();
      if(DriveWithSharps(15, 0)){
        state++;
      };
      break;
      case 55:
      driveAxis(0, 0, 0);
      if(System.currentTimeMillis()-timeToWait>=500){
        m_subsystem.resetYaw();
        state++;
      };
      break;
      case 56:
      timeToWait=System.currentTimeMillis();
      if(alignWithSharps(12)){
        state++;
      };
      break;
      case 57:
      driveAxis(0, 0, 0);
      if(System.currentTimeMillis()-timeToWait>=500){
        m_subsystem.resetYaw();
        state++;
      };
      break;
      case 58:
      timeToWait=System.currentTimeMillis();
      if(RotateToYaw90(-94)){
        state++;
      
      }break;
      case 59:
      driveAxis(0, 0, 0);
      if(System.currentTimeMillis()-timeToWait>=500){
        m_subsystem.resetYaw();
        state++;
      };
      break;
      case 60:
      timeToWait=System.currentTimeMillis();
      if(DriveWithSharps(30, 0)){
        state++;
      }
      break;
      case 61:
      driveAxis(0, 0, 0);
      if(System.currentTimeMillis()-timeToWait>=500){
        m_subsystem.resetYaw();
        state++;
      };
      break;
      case 62:
      timeToWait=System.currentTimeMillis();
      if(driveWithRSonic(500, 0)){
        state++;

      };
      break;
      case 63:
      driveAxis(0, 0, 0);
      if(System.currentTimeMillis()-timeToWait>=500){
        m_subsystem.resetYaw();
        state++;
      };
      break;
      case 64:
      timeToWait=System.currentTimeMillis();
      if(RotateToYaw90(97)){
        state++;
      };
      break;
      case 65:
      driveAxis(0, 0, 0);
      if(System.currentTimeMillis()-timeToWait>=500){
        m_subsystem.resetYaw();
        state++;
      };
      break;
      case 66:
      timeToWait=System.currentTimeMillis();
      if(driveWithLSonic(750, 0)){
        state++;
      };
      break;
      case 67:
      driveAxis(0, 0, 0);
      if(System.currentTimeMillis()-timeToWait>=500){
        m_subsystem.resetYaw();
        state++;
      };
      break;
      case 68:
      timeToWait=System.currentTimeMillis();
      if(DriveWithSharps(30, 0)){
        state++;
      };
      break;
      case 69:
      driveAxis(0, 0, 0);
      if(System.currentTimeMillis()-timeToWait>=500){
        m_subsystem.resetYaw();
        state++;
      };
      break;
      case 70:
      timeToWait=System.currentTimeMillis();
      if(RotateToYaw90(94)){
        state++;
      };
      break;
      case 71:
      driveAxis(0, 0, 0);
      if(System.currentTimeMillis()-timeToWait>=500){
        m_subsystem.resetYaw();
        state++;
      };
      break;
      case 72:
      timeToWait=System.currentTimeMillis();
      if(DriveWithSharps(25, 0)){
        state++;
      };
      break;
      case 73:
      driveAxis(0, 0, 0);
      if(System.currentTimeMillis()-timeToWait>=500){
        m_subsystem.resetYaw();
        state++;
      };
      break;
      case 74:
      timeToWait=System.currentTimeMillis();
      if(RotateToYaw(187)){
        state++;
      };
      break;
      case 75:
      driveAxis(0, 0, 0);
      if(System.currentTimeMillis()-timeToWait>=500){
        m_subsystem.resetYaw();
        state++;
      };
      break;
      case 76:
      timeToWait=System.currentTimeMillis();
      if(DriveWithSharps(25, 0)){
        state++;
      };
      break;
      case 77:
      driveAxis(0, 0, 0);
      if(System.currentTimeMillis()-timeToWait>=500){
        m_subsystem.resetYaw();
        state++;
      };
      break;
      case 78:
      timeToWait=System.currentTimeMillis();
      if(RotateToYaw90(94)){
        state++;
      };
      break;
      case 79:
      driveAxis(0, 0, 0);
      if(System.currentTimeMillis()-timeToWait>=500){
        m_subsystem.resetYaw();
        state++;
      };
      break;
      case 80:
      timeToWait=System.currentTimeMillis();
      if(alignWithSharps(15)){
        state++;
      };
      break;
      case 81:
      driveAxis(0, 0, 0);
      if(System.currentTimeMillis()-timeToWait>=500){
        m_subsystem.resetYaw();
        state++;
      };
      break;
      case 82:
      timeToWait=System.currentTimeMillis();
      if(driveWithLSonic(700, 0)){
        state++;
      };
      break;
      case 83:
      driveAxis(0, 0, 0);
      if(System.currentTimeMillis()-timeToWait>=500){
        m_subsystem.resetYaw();
        state++;
      };
      break;
      case 84:
      timeToWait=System.currentTimeMillis();
      if(RotateToYaw(187)){
        state++;
      };
      break;
      case 85:
      driveAxis(0, 0, 0);
      if(System.currentTimeMillis()-timeToWait>=500){
        m_subsystem.resetYaw();
        state++;
      };
      break;
      case 86:
      timeToWait=System.currentTimeMillis();
      if(DriveWithSharps(17, 0)){
        state++;
      };
      break;
      case 87:
      driveAxis(0, 0, 0);
      if(System.currentTimeMillis()-timeToWait>=500){
        m_subsystem.resetYaw();
        state++;
      };
      break;
      case 88:
      timeToWait=System.currentTimeMillis();
      if(alignWithSharps(15));{
        state++;
      };
      break;
      case 89:
      driveAxis(0, 0, 0);
      if(System.currentTimeMillis()-timeToWait>=500){
        m_subsystem.resetYaw();
        state++;
      };
      break;
      case 90:
      timeToWait=System.currentTimeMillis();
      if(RotateToYaw90(-94)){
        state++;
      };
      break;
      case 91:
      driveAxis(0, 0, 0);
      if(System.currentTimeMillis()-timeToWait>=500){
        m_subsystem.resetYaw();
        state++;
      };
      break;
      case 92:
      timeToWait=System.currentTimeMillis();
      if(driveWithRSonicPID(600, 0)){
        state++;
      };
      break;
      case 93:
      driveAxis(0, 0, 0);
      if(System.currentTimeMillis()-timeToWait>=500){
        m_subsystem.resetYaw();
        state++;
      };
      break;
      case 94:
      timeToWait=System.currentTimeMillis();
      if(DriveWithSharps(20, 0)){
        state++;
      };
      break;
      case 95:
      driveAxis(0, 0, 0);
      if(System.currentTimeMillis()-timeToWait>=500){
        m_subsystem.resetYaw();
        state++;
      };
      break;
      case 96:
      timeToWait=System.currentTimeMillis();
      if(alignWithSharps(15)){
        state++;
      };
      break;
      case 97:
      driveAxis(0, 0, 0);
      if(System.currentTimeMillis()-timeToWait>=500){
        m_subsystem.resetYaw();
        state++;
      };
      break;
      case 98:
      timeToWait=System.currentTimeMillis();
      if(RotateToYaw90(94)){
        state++;
      };
      break;
      case 99:
      driveAxis(0, 0, 0);
      if(System.currentTimeMillis()-timeToWait>=500){
        m_subsystem.resetYaw();
        state++;
      };
      break;
      case 100:
      timeToWait=System.currentTimeMillis();
      if(DriveWithSharps(10, 0)){
        state++;
      };
      break;
      case 101:
      driveAxis(0, 0, 0);
      if(System.currentTimeMillis()-timeToWait>=500){
        m_subsystem.resetYaw();
        state++;
      };
      break;
      case 102:
      timeToWait= System.currentTimeMillis();
      if(RotateToYaw(187)){
        state++;
      };
      break;
      case 103:
      driveAxis(0, 0, 0);
      if(System.currentTimeMillis()-timeToWait>=500){
        m_subsystem.resetYaw();
        state++;
      };
      break;
      case 104:
      timeToWait= System.currentTimeMillis();
      if(RotateToYaw(187)){
        state++;
      };
      break;
      case 105:
      driveAxis(0, 0, 0);
      if(System.currentTimeMillis()-timeToWait>=500){
        m_subsystem.resetYaw();
        state++;
      };
      break;
      case 106:
      timeToWait= System.currentTimeMillis();
      if(RotateToYaw(187)){
        state++;
      };
      break;
      case 107:
      driveAxis(0, 0, 0);
      if(System.currentTimeMillis()-timeToWait>=500){
        m_subsystem.resetYaw();
        state++;
      };
      break;
      case 108:
      timeToWait=System.currentTimeMillis();
      if(DriveWithSharps(16, 0)){
        state++;
      };
      break;
      case 109:
      driveAxis(0, 0, 0);
      if(System.currentTimeMillis()-timeToWait>=500){
        m_subsystem.resetYaw();
        state++;
      };
      break;
      case 110:
      timeToWait=System.currentTimeMillis();
      if(alignWithSharps(14)){
        state++;
      };
      break;
      case 111:
      driveAxis(0, 0, 0);
      if(System.currentTimeMillis()-timeToWait>=500){
        m_subsystem.resetYaw();
        state++;
      };
      break;
      case 112:
      timeToWait=System.currentTimeMillis();
      if(driveWithRSonicPID(750, 0)){
        state++;
      };
      break;
      case 113:
      driveAxis(0, 0, 0);
      if(System.currentTimeMillis()-timeToWait>=500){
        m_subsystem.resetYaw();
        state++;
      };
      break;
      case 114:
      timeToWait=System.currentTimeMillis();
      if(DriveWithSharps(20, 0)){
        state++;
      };
      break;
      case 115:
      driveAxis(0, 0, 0);
      if(System.currentTimeMillis()-timeToWait>=500){
        m_subsystem.resetYaw();
        state++;
      };
      break;
      case 116:
      timeToWait=System.currentTimeMillis();
      if(alignWithSharps(15)){
        state++;
      };
      break;
      case 117:
      driveAxis(0, 0, 0);
      if(System.currentTimeMillis()-timeToWait>=500){
        m_subsystem.resetYaw();
        state++;
      };
      break;
      case 118:
      timeToWait=System.currentTimeMillis();
      if(RotateToYaw90(94)){
        state++;
      };
      break;
      case 119:
      driveAxis(0, 0, 0);
      if(System.currentTimeMillis()-timeToWait>=500){
        m_subsystem.resetYaw();
        state++;
      };
      break;
      case 120:
      timeToWait=System.currentTimeMillis();
      if(driveWithLSonic(1200, 0)){
        state++;
      };
      break;
      case 121:
      driveAxis(0, 0, 0);
      if(System.currentTimeMillis()-timeToWait>=500){
        m_subsystem.resetYaw();
        state++;
      };
      break;
      case 122:
      timeToWait=System.currentTimeMillis();
      if(DriveWithSharps(25, 0)){
        state++;
      };
      break;
      case 123:
      driveAxis(0, 0, 0);
      if(System.currentTimeMillis()-timeToWait>=500){
        m_subsystem.resetYaw();
        state++;
      };
      break;
      case 124:
      timeToWait=System.currentTimeMillis();
      if(RotateToYaw90(94)){
        state++;
      };
      break;
      case 125:
      driveAxis(0, 0, 0);
      if(System.currentTimeMillis()-timeToWait>=500){
        m_subsystem.resetYaw();
        state++;
      };
      break;
      case 126:
      timeToWait=System.currentTimeMillis();
      if(alignWithSharps(15)){
        state++;
      };
      break;
      case 127:
      driveAxis(0, 0, 0);
      if(System.currentTimeMillis()-timeToWait>=500){
        m_subsystem.resetYaw();
        state++;
      };
      break;
      case 128:
      timeToWait=System.currentTimeMillis();
      if(driveWithRSonic(200, 0)){
        state++;
      };
      break;
      case 129:
      driveAxis(0, 0, 0);
      if(System.currentTimeMillis()-timeToWait>=500){
        m_subsystem.resetYaw();
        state++;
      };
      break;
      case 130:
      timeToWait=System.currentTimeMillis();
      if(driveWithRSonic(700, 0)){
        state++;
      };
      break;
      case 131:
      driveAxis(0, 0, 0);
      if(System.currentTimeMillis()-timeToWait>=500){
        m_subsystem.resetYaw();
        state++;
      };
      break;
      case 132:
      timeToWait=System.currentTimeMillis();
      if(DriveWithSharps(20, 0)){
        state++;
      };
      break;
      case 133:
      driveAxis(0, 0, 0);
      if(System.currentTimeMillis()-timeToWait>=500){
        m_subsystem.resetYaw();
        state++;
      };
      break;
      case 134:
      timeToWait=System.currentTimeMillis();
      if(alignWithSharps(15)){
        state++;
      };
      break;
      case 135:
      driveAxis(0, 0, 0);
      if(System.currentTimeMillis()-timeToWait>=500){
        m_subsystem.resetYaw();
        state++;
      };
      break;
      case 136:
      timeToWait=System.currentTimeMillis();
      if(RotateToYaw90(-94)){
        state++;
      };
      break;
      case 137:
      driveAxis(0, 0, 0);
      if(System.currentTimeMillis()-timeToWait>=500){
        m_subsystem.resetYaw();
        state++;
      };
      break;
      case 138:
      timeToWait=System.currentTimeMillis();
      if(driveWithRSonicPID(600, 0)){
        state++;
      };
      break;
      case 139:
      driveAxis(0, 0, 0);
      if(System.currentTimeMillis()-timeToWait>=500){
        m_subsystem.resetYaw();
        state++;
      };
      break;
      case 140:
      timeToWait=System.currentTimeMillis();
      if(DriveWithSharps(20, 0)){
        state++;
      };
      break;
      case 141:
      driveAxis(0, 0, 0);
      if(System.currentTimeMillis()-timeToWait>=500){
        m_subsystem.resetYaw();
        state++;
      };
      break;
      case 142:
      timeToWait=System.currentTimeMillis();
      if(alignWithSharps(15)){
        state++;
      };
      break;
      case 143:
      driveAxis(0, 0, 0);
      if(System.currentTimeMillis()-timeToWait>=500){
        m_subsystem.resetYaw();
        state++;
      };
      break;
      case 144:
      timeToWait=System.currentTimeMillis();
      if(RotateToYaw90(94)){
        state++;
      };
      break;
      case 145:
      driveAxis(0, 0, 0);
      if(System.currentTimeMillis()-timeToWait>=500){
        m_subsystem.resetYaw();
        state++;
      };
      break;
      case 146:
      timeToWait=System.currentTimeMillis();
      if(DriveWithSharps(10, 0)){
        state++;
      };
      break;
      case 147:
      driveAxis(0, 0, 0);
      if(System.currentTimeMillis()-timeToWait>=500){
        m_subsystem.resetYaw();
        state++;
      };
      break;
      case 148:
      timeToWait= System.currentTimeMillis();
      if(RotateToYaw(187)){
        state++;
      };
      break;
      case 149:
      driveAxis(0, 0, 0);
      if(System.currentTimeMillis()-timeToWait>=500){
        m_subsystem.resetYaw();
        state++;
      };
      break;
      case 150:
      timeToWait= System.currentTimeMillis();
      if(RotateToYaw(187)){
        state++;
      };
      break;
      case 151:
      driveAxis(0, 0, 0);
      if(System.currentTimeMillis()-timeToWait>=500){
        m_subsystem.resetYaw();
        state++;
      };
      break;
      case 152:
      timeToWait= System.currentTimeMillis();
      if(RotateToYaw(187)){
        state++;
      };
      break;
      case 153:
      driveAxis(0, 0, 0);
      if(System.currentTimeMillis()-timeToWait>=500){
        m_subsystem.resetYaw();
        state++;
      };
      break;
      case 154:
      timeToWait=System.currentTimeMillis();
      if(DriveWithSharps(15, 0)){
        state++;
      };
      break;
      case 155:
      driveAxis(0, 0, 0);
      if(System.currentTimeMillis()-timeToWait>=500){
        m_subsystem.resetYaw();
        state++;
      };
      break;
      case 156:
      timeToWait=System.currentTimeMillis();
      if(alignWithSharps(12)){
        state++;
      };
      break;
      case 157:
      driveAxis(0, 0, 0);
      if(System.currentTimeMillis()-timeToWait>=500){
        m_subsystem.resetYaw();
        state++;
      };
      break;
      case 158:
      timeToWait=System.currentTimeMillis();
      if(RotateToYaw90(-94)){
        state++;
      };
      break;
      case 159:
      driveAxis(0, 0, 0);
      if(System.currentTimeMillis()-timeToWait>=500){
        m_subsystem.resetYaw();
        state++;
      };
      break;
      case 160:
      timeToWait=System.currentTimeMillis();
      if(DriveWithSharps(35, 0)){
        state++;
      };
      break;
      case 161:
      driveAxis(0, 0, 0);
      if(System.currentTimeMillis()-timeToWait>=500){
        m_subsystem.resetYaw();
        state++;
      };
      break;
      case 162:
      timeToWait=System.currentTimeMillis();
      if(driveWithLSonicPID(155, 0)){
        state++;
      };
      break;
      case 163:
      m_subsystem.setServoAngle(90);
      m_subsystem.setServoAngleUgol(0);
      driveAxis(0, 0, 0);
      if(System.currentTimeMillis()-timeToWait>=1000){
        m_subsystem.resetYaw();
        state++;
      };
      break;
      case 164:
      m_subsystem.setServoAngle(90);
      m_subsystem.setServoAngleUgol(0);
      timeToWait=System.currentTimeMillis();
      if(DriveWithSharps(24, 0)){
        state++;
      };
      break;
      case 165:
      m_subsystem.setServoAngle(0);
      m_subsystem.setServoAngleUgol(0);
      driveAxis(0, 0, 0);
      if(System.currentTimeMillis()-timeToWait>=1000){
        m_subsystem.resetYaw();
        state++;
      };
      break;
      case 166:
      m_subsystem.setServoAngle(0);
      m_subsystem.setServoAngleUgol(0);
       timeToWait=System.currentTimeMillis();
       if(DriveWithSharps(40, 0)){
         state++;
       };
       break;
       case 167:
       m_subsystem.setServoAngle(0);
       m_subsystem.setServoAngleUgol(0);
       driveAxis(0, 0, 0);
       if(System.currentTimeMillis()-timeToWait>=1000){
         m_subsystem.resetYaw();
         state++;
       };
       break;
       case 168:
       m_subsystem.setServoAngle(0);
       m_subsystem.setServoAngleUgol(0);
       timeToWait=System.currentTimeMillis();
       if(RotateToYaw90(-94)){
         state++;
       };
       break;
       case 169:
       m_subsystem.setServoAngle(0);
       m_subsystem.setServoAngleUgol(0);
       driveAxis(0, 0, 0);
       if(System.currentTimeMillis()-timeToWait>=1000){
         m_subsystem.resetYaw();
         state++;
       };
       break;
       case 170:
       m_subsystem.setServoAngle(0);
       m_subsystem.setServoAngleUgol(0);
       timeToWait=System.currentTimeMillis();
       if(DriveWithSharps(10, 0)){
         state++;
       };
       break;
       case 171:
       m_subsystem.setServoAngle(90);
       m_subsystem.setServoAngleUgol(0);
       driveAxis(0, 0, 0);
       if(System.currentTimeMillis()-timeToWait>=1000){
         m_subsystem.resetYaw();
         state++;
       };
       break;
       case 172:
       m_subsystem.setServoAngle(90);
       m_subsystem.setServoAngleUgol(0);
       timeToWait=System.currentTimeMillis();
       if(RotateToYaw90(94)){
         state++;
       };
       break;
       case 173:
       m_subsystem.setServoAngle(90);
       m_subsystem.setServoAngleUgol(0);
       driveAxis(0, 0, 0);
       if(System.currentTimeMillis()-timeToWait>=1000){
         m_subsystem.resetYaw();
         state++;
       };
       break;
       case 174:
       timeToWait=System.currentTimeMillis();
      if(driveWithLSonicPID(307, 0)){
        state++;
      };
      break;
      case 175:
      m_subsystem.setServoAngle(90);
      m_subsystem.setServoAngleUgol(0);
      driveAxis(0, 0, 0);
      if(System.currentTimeMillis()-timeToWait>=1000){
        m_subsystem.resetYaw();
        state++;
      };
      break;
      case 176:
      m_subsystem.setServoAngle(90);
      m_subsystem.setServoAngleUgol(0);
      timeToWait=System.currentTimeMillis();
      if(DriveWithSharps(24, 0)){
        state++;
      };
      break;
      case 177:
      m_subsystem.setServoAngle(0);
      m_subsystem.setServoAngleUgol(0);
      driveAxis(0, 0, 0);
      if(System.currentTimeMillis()-timeToWait>=1000){
        m_subsystem.resetYaw();
        state++;
      };
      break;
      case 178:
      m_subsystem.setServoAngle(0);
      m_subsystem.setServoAngleUgol(0);
       timeToWait=System.currentTimeMillis();
       if(DriveWithSharps(40, 0)){
         state++;
       };
       break;
       case 179:
       m_subsystem.setServoAngle(0);
       m_subsystem.setServoAngleUgol(0);
       driveAxis(0, 0, 0);
       if(System.currentTimeMillis()-timeToWait>=1000){
         m_subsystem.resetYaw();
         state++;
       };
       break;
       case 180:
       m_subsystem.setServoAngle(0);
       m_subsystem.setServoAngleUgol(0);
       timeToWait=System.currentTimeMillis();
       if(RotateToYaw90(-94)){
         state++;
       };
       break;
       case 181:
       m_subsystem.setServoAngle(0);
       m_subsystem.setServoAngleUgol(0);
       driveAxis(0, 0, 0);
       if(System.currentTimeMillis()-timeToWait>=1000){
         m_subsystem.resetYaw();
         state++;
       };
       break;
       case 182:
       m_subsystem.setServoAngle(0);
       m_subsystem.setServoAngleUgol(0);
       timeToWait=System.currentTimeMillis();
       if(DriveWithSharps(10, 0)){
         state++;
       };
       break;
       case 183:
       m_subsystem.setServoAngle(90);
       m_subsystem.setServoAngleUgol(0);
       driveAxis(0, 0, 0);
       if(System.currentTimeMillis()-timeToWait>=1000){
         m_subsystem.resetYaw();
         state++;
       };
       break;
       case 184:
       m_subsystem.setServoAngle(90);
       m_subsystem.setServoAngleUgol(0);
       timeToWait=System.currentTimeMillis();
       if(RotateToYaw90(94)){
         state++;
       };
       break;
       case 185:
       m_subsystem.setServoAngle(90);
       m_subsystem.setServoAngleUgol(0);
       driveAxis(0, 0, 0);
       if(System.currentTimeMillis()-timeToWait>=1000){
         m_subsystem.resetYaw();
         state++;
       };
       break;
       case 186:
       m_subsystem.setServoAngle(90);
       m_subsystem.setServoAngleUgol(0);
       timeToWait=System.currentTimeMillis();
       if(resetLift1()){
         state++;
       };
       break;
       case 187:
       m_subsystem.setServoAngle(90);
       m_subsystem.setServoAngleUgol(0);
       driveAxis(0, 0, 0);
       if(System.currentTimeMillis()-timeToWait>=1000){
         m_subsystem.resetYaw();
         state++;
       };
       break;
       case 188:
       m_subsystem.setServoAngle(90);
       m_subsystem.setServoAngleUgol(0);
       timeToWait=System.currentTimeMillis();
       if(setEncValue(2800)){
         state++;
       };
       break;
       case 189:
       m_subsystem.setServoAngle(90);
       m_subsystem.setServoAngleUgol(0);
       driveAxis(0, 0, 0);
       if(System.currentTimeMillis()-timeToWait>=1000){
         m_subsystem.resetYaw();
         state++;
       };
       break;
       case 190:
       m_subsystem.setServoAngle(90);
       m_subsystem.setServoAngleUgol(0);
       timeToWait=System.currentTimeMillis();
       if(driveWithLSonic(235, 0)){
         state++;
       };
       break;
       case 191:
      m_subsystem.setServoAngle(90);
      m_subsystem.setServoAngleUgol(0);
      driveAxis(0, 0, 0);
      if(System.currentTimeMillis()-timeToWait>=1000){
        m_subsystem.resetYaw();
        state++;
      };
      break;
      case 192:
      m_subsystem.setServoAngle(90);
      m_subsystem.setServoAngleUgol(0);
      timeToWait=System.currentTimeMillis();
      if(DriveWithSharps(24, 0)){
        state++;
      };
      break;
      case 193:
      m_subsystem.setServoAngle(0);
      m_subsystem.setServoAngleUgol(0);
      driveAxis(0, 0, 0);
      if(System.currentTimeMillis()-timeToWait>=1000){
        m_subsystem.resetYaw();
        state++;
      };
      break;
      case 194:
      m_subsystem.setServoAngle(0);
      m_subsystem.setServoAngleUgol(0);
       timeToWait=System.currentTimeMillis();
       if(DriveWithSharps(40, 0)){
         state++;
       };
       break;
       case 195:
       m_subsystem.setServoAngle(0);
       m_subsystem.setServoAngleUgol(0);
       driveAxis(0, 0, 0);
       if(System.currentTimeMillis()-timeToWait>=1000){
         m_subsystem.resetYaw();
         state++;
       };
       break;
       case 196:
       m_subsystem.setServoAngle(0);
       m_subsystem.setServoAngleUgol(0);
       timeToWait=System.currentTimeMillis();
       if(RotateToYaw90(-94)){
         state++;
       };
       break;
       case 197:
       m_subsystem.setServoAngle(0);
       m_subsystem.setServoAngleUgol(0);
       driveAxis(0, 0, 0);
       if(System.currentTimeMillis()-timeToWait>=1000){
         m_subsystem.resetYaw();
         state++;
       };
       break;
       case 198:
       m_subsystem.setServoAngle(0);
       m_subsystem.setServoAngleUgol(0);
       timeToWait=System.currentTimeMillis();
       if(resetLift1()){
         state++;
       };
       break;
       case 199:
       m_subsystem.setServoAngle(0);
       m_subsystem.setServoAngleUgol(0);
       driveAxis(0, 0, 0);
       if(System.currentTimeMillis()-timeToWait>=1000){
         m_subsystem.resetYaw();
         state++;
       };
       break;
       case 200:
       m_subsystem.setServoAngle(0);
       m_subsystem.setServoAngleUgol(0);
       timeToWait=System.currentTimeMillis();
       if(setEncValue(100)){
         state++;
       };
       break;
       case 201:
       m_subsystem.setServoAngle(0);
       m_subsystem.setServoAngleUgol(0);
       driveAxis(0, 0, 0);
       if(System.currentTimeMillis()-timeToWait>=1000){
         m_subsystem.resetYaw();
         state++;
       };
       break;
      
       case 202:
       m_subsystem.setServoAngle(0);
       m_subsystem.setServoAngleUgol(0);
       timeToWait=System.currentTimeMillis();
       if(DriveWithSharps(10, 0)){
         state++;
       };
       break;
       case 203:
       m_subsystem.setServoAngle(90);
       m_subsystem.setServoAngleUgol(0);
       driveAxis(0, 0, 0);
       if(System.currentTimeMillis()-timeToWait>=1000){
         m_subsystem.resetYaw();
         state++;
       };
       break;
       case 204:
       m_subsystem.setServoAngle(90);
       m_subsystem.setServoAngleUgol(0);
       timeToWait=System.currentTimeMillis();
       if(RotateToYaw90(94)){
         state++;
       };
       break;
       case 205:
       m_subsystem.setServoAngle(90);
       m_subsystem.setServoAngleUgol(0);
       driveAxis(0, 0, 0);
       if(System.currentTimeMillis()-timeToWait>=1000){
         m_subsystem.resetYaw();
         state++;
       };
       break;
       case 206:
       m_subsystem.setServoAngle(90);
       m_subsystem.setServoAngleUgol(0);
       timeToWait=System.currentTimeMillis();
       if(resetLift1()){
         state++;
       };
       break;
       case 207:
       m_subsystem.setServoAngle(90);
       m_subsystem.setServoAngleUgol(0);
       driveAxis(0, 0, 0);
       if(System.currentTimeMillis()-timeToWait>=1000){
         m_subsystem.resetYaw();
         state++;
       };
       break;
       case 208:
       m_subsystem.setServoAngle(90);
       m_subsystem.setServoAngleUgol(0);
       timeToWait=System.currentTimeMillis();
       if(setEncValue(2800)){
         state++;
       };
       break;
       case 209:
       m_subsystem.setServoAngle(90);
       m_subsystem.setServoAngleUgol(0);
       driveAxis(0, 0, 0);
       if(System.currentTimeMillis()-timeToWait>=1000){
         m_subsystem.resetYaw();
         state++;
       };
       break;
       case 210:
       m_subsystem.setServoAngle(90);
       m_subsystem.setServoAngleUgol(0);
       timeToWait=System.currentTimeMillis();
       if(driveWithLSonic(389, 0)){
         state++;
       };
       break;
       case 211:
      m_subsystem.setServoAngle(90);
      m_subsystem.setServoAngleUgol(0);
      driveAxis(0, 0, 0);
      if(System.currentTimeMillis()-timeToWait>=1000){
        m_subsystem.resetYaw();
        state++;
      };
      break;
      case 212:
      m_subsystem.setServoAngle(90);
      m_subsystem.setServoAngleUgol(0);
      timeToWait=System.currentTimeMillis();
      if(DriveWithSharps(24, 0)){
        state++;
      };
      break;
      case 213:
      m_subsystem.setServoAngle(0);
      m_subsystem.setServoAngleUgol(0);
      driveAxis(0, 0, 0);
      if(System.currentTimeMillis()-timeToWait>=1000){
        m_subsystem.resetYaw();
        state++;
      };
      break;
      case 214:
      m_subsystem.setServoAngle(0);
      m_subsystem.setServoAngleUgol(0);
       timeToWait=System.currentTimeMillis();
       if(DriveWithSharps(40, 0)){
         state++;
       };
       break;
       case 215:
       m_subsystem.setServoAngle(0);
       m_subsystem.setServoAngleUgol(0);
       driveAxis(0, 0, 0);
       if(System.currentTimeMillis()-timeToWait>=1000){
         m_subsystem.resetYaw();
         state++;
       };
       break;
       case 216:
       m_subsystem.setServoAngle(0);
       m_subsystem.setServoAngleUgol(0);
       timeToWait=System.currentTimeMillis();
       if(RotateToYaw90(-94)){
         state++;
       };
       break;
       case 217:
       m_subsystem.setServoAngle(0);
       m_subsystem.setServoAngleUgol(0);
       driveAxis(0, 0, 0);
       if(System.currentTimeMillis()-timeToWait>=1000){
         m_subsystem.resetYaw();
         state++;
       };
       break;
       case 218:
       m_subsystem.setServoAngle(0);
       m_subsystem.setServoAngleUgol(0);
       timeToWait=System.currentTimeMillis();
       if(resetLift1()){
         state++;
       };
       break;
       case 219:
       m_subsystem.setServoAngle(0);
       m_subsystem.setServoAngleUgol(0);
       driveAxis(0, 0, 0);
       if(System.currentTimeMillis()-timeToWait>=1000){
         m_subsystem.resetYaw();
         state++;
       };
       break;
       case 220:
       m_subsystem.setServoAngle(0);
       m_subsystem.setServoAngleUgol(0);
       timeToWait=System.currentTimeMillis();
       if(setEncValue(100)){
         state++;
       };
       break;
       case 221:
       m_subsystem.setServoAngle(0);
       m_subsystem.setServoAngleUgol(0);
       driveAxis(0, 0, 0);
       if(System.currentTimeMillis()-timeToWait>=1000){
         m_subsystem.resetYaw();
         state++;
       };
       break;
       case 222:
       m_subsystem.setServoAngle(0);
       m_subsystem.setServoAngleUgol(0);
       timeToWait=System.currentTimeMillis();
       if(DriveWithSharps(10, 0)){
         state++;
       };
       break;
       case 223:
       m_subsystem.setServoAngle(90);
       m_subsystem.setServoAngleUgol(0);
       driveAxis(0, 0, 0);
       if(System.currentTimeMillis()-timeToWait>=1000){
         m_subsystem.resetYaw();
         state++;
       };
       break;
       case 224:
       m_subsystem.setServoAngle(90);
       m_subsystem.setServoAngleUgol(0);
       timeToWait=System.currentTimeMillis();
       if(RotateToYaw90(94)){
         state++;
       };
       break;
       case 225:
       m_subsystem.setServoAngle(90);
       m_subsystem.setServoAngleUgol(0);
       driveAxis(0, 0, 0);
       if(System.currentTimeMillis()-timeToWait>=1000){
         m_subsystem.resetYaw();
         state++;
       };
       break;
       case 226:
       m_subsystem.setServoAngle(90);
       m_subsystem.setServoAngleUgol(0);
       timeToWait=System.currentTimeMillis();
       if(driveWithLSonic(235, 0)){
         state++;
       };
       break;
       case 227:
       m_subsystem.setServoAngle(90);
       m_subsystem.setServoAngleUgol(0);
       driveAxis(0, 0, 0);
       if(System.currentTimeMillis()-timeToWait>=1000){
         m_subsystem.resetYaw();
         state++;
       };
       break;
       case 228:
       m_subsystem.setServoAngle(90);
       m_subsystem.setServoAngleUgol(0);
       timeToWait=System.currentTimeMillis();
       if(resetLift1()){
         state++;
       };
       break;
       case 229:
       m_subsystem.setServoAngle(90);
       m_subsystem.setServoAngleUgol(0);
       driveAxis(0, 0, 0);
       if(System.currentTimeMillis()-timeToWait>=1000){
         m_subsystem.resetYaw();
         state++;
       };
       break;
       case 230:
       m_subsystem.setServoAngle(90);
       m_subsystem.setServoAngleUgol(0);
       timeToWait=System.currentTimeMillis();
       if(setEncValue(5400)){
         state++;
       };
       break;
       case 231:
       m_subsystem.setServoAngle(90);
       m_subsystem.setServoAngleUgol(0);
       driveAxis(0, 0, 0);
       if(System.currentTimeMillis()-timeToWait>=1000){
         m_subsystem.resetYaw();
         state++;
       };
       break;
       
       case 232:
       m_subsystem.setServoAngle(90);
       m_subsystem.setServoAngleUgol(0);
       timeToWait=System.currentTimeMillis();
       if(driveWithLSonic(235, 0)){
         state++;
       };
       break;
       case 233:
      m_subsystem.setServoAngle(90);
      m_subsystem.setServoAngleUgol(0);
      driveAxis(0, 0, 0);
      if(System.currentTimeMillis()-timeToWait>=1000){
        m_subsystem.resetYaw();
        state++;
      };
      break;
      case 234:
      m_subsystem.setServoAngle(90);
      m_subsystem.setServoAngleUgol(0);
      timeToWait=System.currentTimeMillis();
      if(DriveWithSharps(24, 0)){
        state++;
      };
      break;
      case 235:
      m_subsystem.setServoAngle(0);
      m_subsystem.setServoAngleUgol(0);
      driveAxis(0, 0, 0);
      if(System.currentTimeMillis()-timeToWait>=1000){
        m_subsystem.resetYaw();
        state++;
      };
      break;
      case 236:
      m_subsystem.setServoAngle(0);
      m_subsystem.setServoAngleUgol(0);
       timeToWait=System.currentTimeMillis();
       if(DriveWithSharps(40, 0)){
         state++;
       };
       break;
       case 237:
       m_subsystem.setServoAngle(0);
       m_subsystem.setServoAngleUgol(0);
       driveAxis(0, 0, 0);
       if(System.currentTimeMillis()-timeToWait>=1000){
         m_subsystem.resetYaw();
         state++;
       };
       break;
       case 238:
       m_subsystem.setServoAngle(0);
       m_subsystem.setServoAngleUgol(0);
       timeToWait=System.currentTimeMillis();
       if(RotateToYaw90(-94)){
         state++;
       };
       break;
       case 239:
       m_subsystem.setServoAngle(0);
       m_subsystem.setServoAngleUgol(0);
       driveAxis(0, 0, 0);
       if(System.currentTimeMillis()-timeToWait>=1000){
         m_subsystem.resetYaw();
         state++;
       };
       break;
       case 240:
       m_subsystem.setServoAngle(0);
       m_subsystem.setServoAngleUgol(0);
       timeToWait=System.currentTimeMillis();
       if(resetLift1()){
         state++;
       };
       break;
       case 241:
       m_subsystem.setServoAngle(0);
       m_subsystem.setServoAngleUgol(0);
       driveAxis(0, 0, 0);
       if(System.currentTimeMillis()-timeToWait>=1000){
         m_subsystem.resetYaw();
         state++;
       };
       break;
       case 242:
       m_subsystem.setServoAngle(0);
       m_subsystem.setServoAngleUgol(0);
       timeToWait=System.currentTimeMillis();
       if(setEncValue(100)){
         state++;
       };
       break;
       case 243:
       m_subsystem.setServoAngle(0);
       m_subsystem.setServoAngleUgol(0);
       driveAxis(0, 0, 0);
       if(System.currentTimeMillis()-timeToWait>=1000){
         m_subsystem.resetYaw();
         state++;
       };
       break;
       case 244:
       m_subsystem.setServoAngle(0);
       m_subsystem.setServoAngleUgol(0);
       timeToWait=System.currentTimeMillis();
       if(DriveWithSharps(10, 0)){
         state++;
       };
       break;
       case 245:
       m_subsystem.setServoAngle(90);
       m_subsystem.setServoAngleUgol(0);
       driveAxis(0, 0, 0);
       if(System.currentTimeMillis()-timeToWait>=1000){
         m_subsystem.resetYaw();
         state++;
       };
       break;
       case 246:
       m_subsystem.setServoAngle(90);
       m_subsystem.setServoAngleUgol(0);
       timeToWait=System.currentTimeMillis();
       if(RotateToYaw90(94)){
         state++;
       };
       break;
       case 247:
       m_subsystem.setServoAngle(90);
       m_subsystem.setServoAngleUgol(0);
       driveAxis(0, 0, 0);
       if(System.currentTimeMillis()-timeToWait>=1000){
         m_subsystem.resetYaw();
         state++;
       };
       break;
       case 248:
       m_subsystem.setServoAngle(90);
       m_subsystem.setServoAngleUgol(0);
       timeToWait=System.currentTimeMillis();
       if(driveWithLSonic(389, 0)){
         state++;
       };
       break;
       case 249:
       m_subsystem.setServoAngle(90);
       m_subsystem.setServoAngleUgol(0);
       driveAxis(0, 0, 0);
       if(System.currentTimeMillis()-timeToWait>=1000){
         m_subsystem.resetYaw();
         state++;
       };
       break;
       case 250:
       m_subsystem.setServoAngle(90);
       m_subsystem.setServoAngleUgol(0);
       timeToWait=System.currentTimeMillis();
       if(resetLift1()){
         state++;
       };
       break;
       case 251:
       m_subsystem.setServoAngle(90);
       m_subsystem.setServoAngleUgol(0);
       driveAxis(0, 0, 0);
       if(System.currentTimeMillis()-timeToWait>=1000){
         m_subsystem.resetYaw();
         state++;
       };
       break;
       case 252:
       m_subsystem.setServoAngle(90);
       m_subsystem.setServoAngleUgol(0);
       timeToWait=System.currentTimeMillis();
       if(setEncValue(5400)){
         state++;
       };
       break;
       case 253:
       m_subsystem.setServoAngle(90);
       m_subsystem.setServoAngleUgol(0);
       driveAxis(0, 0, 0);
       if(System.currentTimeMillis()-timeToWait>=1000){
         m_subsystem.resetYaw();
         state++;
       };
       break;
      case 254:
      m_subsystem.setServoAngle(90);
      m_subsystem.setServoAngleUgol(0);
      timeToWait=System.currentTimeMillis();
      if(DriveWithSharps(24, 0)){
        state++;
      };
      break;
      case 255:
      m_subsystem.setServoAngle(0);
      m_subsystem.setServoAngleUgol(0);
      driveAxis(0, 0, 0);
      if(System.currentTimeMillis()-timeToWait>=1000){
        m_subsystem.resetYaw();
        state++;
      };
      break;
      case 256:
      m_subsystem.setServoAngle(0);
      m_subsystem.setServoAngleUgol(0);
       timeToWait=System.currentTimeMillis();
       if(DriveWithSharps(40, 0)){
         state++;
       };
       break;
       case 257:
       m_subsystem.setServoAngle(0);
       m_subsystem.setServoAngleUgol(0);
       driveAxis(0, 0, 0);
       if(System.currentTimeMillis()-timeToWait>=1000){
         m_subsystem.resetYaw();
         state++;
       };
       break;
       case 258:
       m_subsystem.setServoAngle(0);
       m_subsystem.setServoAngleUgol(0);
       timeToWait=System.currentTimeMillis();
       if(RotateToYaw90(-94)){
         state++;
       };
       break;
       case 259:
       m_subsystem.setServoAngle(0);
       m_subsystem.setServoAngleUgol(0);
       driveAxis(0, 0, 0);
       if(System.currentTimeMillis()-timeToWait>=1000){
         m_subsystem.resetYaw();
         state++;
       };
       break;
       case 260:
       m_subsystem.setServoAngle(0);
       m_subsystem.setServoAngleUgol(0);
       timeToWait=System.currentTimeMillis();
       if(resetLift1()){
         state++;
       };
       break;
       case 261:
       m_subsystem.setServoAngle(0);
       m_subsystem.setServoAngleUgol(0);
       driveAxis(0, 0, 0);
       if(System.currentTimeMillis()-timeToWait>=1000){
         m_subsystem.resetYaw();
         state++;
       };
       break;
       case 262:
       m_subsystem.setServoAngle(0);
       m_subsystem.setServoAngleUgol(0);
       timeToWait=System.currentTimeMillis();
       if(setEncValue(100)){
         state++;
       };
       break;
       case 263:
       m_subsystem.setServoAngle(0);
       m_subsystem.setServoAngleUgol(0);
       driveAxis(0, 0, 0);
       if(System.currentTimeMillis()-timeToWait>=1000){
         m_subsystem.resetYaw();
         state++;
       };
       break;
       case 264:
       m_subsystem.setServoAngle(0);
       m_subsystem.setServoAngleUgol(0);
       timeToWait=System.currentTimeMillis();
       if(DriveWithSharps(10, 0)){
         state++;
       };
       break;
       case 265:
       m_subsystem.setServoAngle(90);
       m_subsystem.setServoAngleUgol(0);
       driveAxis(0, 0, 0);
       if(System.currentTimeMillis()-timeToWait>=1000){
         m_subsystem.resetYaw();
         state++;
       };
       break;
       case 266:
       m_subsystem.setServoAngle(90);
       m_subsystem.setServoAngleUgol(0);
       timeToWait=System.currentTimeMillis();
       if(RotateToYaw90(-94)){
         state++;
       };
       break;
    
       case 267:
       m_subsystem.setServoAngle(90);
       m_subsystem.setServoAngleUgol(0);
      driveAxis(0, 0, 0);
      if(System.currentTimeMillis()-timeToWait>=500){
        m_subsystem.resetYaw();
        state++;
      };
      break;
      case 268:
      m_subsystem.setServoAngle(90);
       m_subsystem.setServoAngleUgol(0);
       timeToWait=System.currentTimeMillis();
       if(DriveWithSharps(20, 0)){
         state++;
       };
       break;
       case 269:
       m_subsystem.setServoAngle(90);
       m_subsystem.setServoAngleUgol(0);
      driveAxis(0, 0, 0);
      if(System.currentTimeMillis()-timeToWait>=500){
        m_subsystem.resetYaw();
        state++;
      };
      break;
      case 270:
      m_subsystem.setServoAngle(90);
       m_subsystem.setServoAngleUgol(0);
       timeToWait=System.currentTimeMillis();
       if(alignWithSharps(15)){
         state++;
       };
       break;
       case 271:
       m_subsystem.setServoAngle(90);
       m_subsystem.setServoAngleUgol(0);
      driveAxis(0, 0, 0);
      if(System.currentTimeMillis()-timeToWait>=500){
        m_subsystem.resetYaw();
        state++;
      };
      break;
      case 272:
      m_subsystem.setServoAngle(90);
       m_subsystem.setServoAngleUgol(0);
      timeToWait=System.currentTimeMillis();
      if(driveWithRSonicPID(600, 0)){//nuzhno menyat'
        state++;
      };
      break;
      case 273:
      m_subsystem.setServoAngle(90);
       m_subsystem.setServoAngleUgol(0);
      driveAxis(0, 0, 0);
      if(System.currentTimeMillis()-timeToWait>=500){
        m_subsystem.resetYaw();
        state++;
      };
      break;
      
      case 274:
      m_subsystem.setServoAngle(90);
       m_subsystem.setServoAngleUgol(0);
      timeToWait=System.currentTimeMillis();
      if(DriveWithSharps(20, 0)){
        state++;
      };
      break;
      case 275:
      m_subsystem.setServoAngle(90);
       m_subsystem.setServoAngleUgol(0);
      driveAxis(0, 0, 0);
      if(System.currentTimeMillis()-timeToWait>=500){
        m_subsystem.resetYaw();
        state++;
      };
      break;
      case 276:
      m_subsystem.setServoAngle(90);
       m_subsystem.setServoAngleUgol(0);
      timeToWait=System.currentTimeMillis();
      if(alignWithSharps(15)){
        state++;
      };
      break;
      case 277:
      m_subsystem.setServoAngle(90);
       m_subsystem.setServoAngleUgol(0);
      driveAxis(0, 0, 0);
      if(System.currentTimeMillis()-timeToWait>=500){
        m_subsystem.resetYaw();
        state++;
      };
      break;
      case 278:
      m_subsystem.setServoAngle(90);
       m_subsystem.setServoAngleUgol(0);
      timeToWait=System.currentTimeMillis();
      if(RotateToYaw90(94)){
        state++;
      };
      break;
      case 279:
      m_subsystem.setServoAngle(90);
       m_subsystem.setServoAngleUgol(0);
      driveAxis(0, 0, 0);
      if(System.currentTimeMillis()-timeToWait>=500){
        m_subsystem.resetYaw();
        state++;
      };
      break;
      case 280:
      m_subsystem.setServoAngle(90);
       m_subsystem.setServoAngleUgol(0);
      timeToWait=System.currentTimeMillis();
      if(DriveWithSharps(12, 0)){
        state++;
      };
      break;
      case 281:
      m_subsystem.setServoAngle(90);
       m_subsystem.setServoAngleUgol(0);
      driveAxis(0, 0, 0);
      if(System.currentTimeMillis()-timeToWait>=500){
        m_subsystem.resetYaw();
        state++;
      };
      break;
       







      
   }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveAxis(0, 0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
  