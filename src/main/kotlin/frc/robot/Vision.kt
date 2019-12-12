package frc.robot

import com.ctre.phoenix.motorcontrol.can.TalonSRX
import edu.wpi.first.networktables.NetworkTableInstance
import frc.robot.Vision.getUpdate
import frc.robot.subsystems.drive.DriveCommand
import frc.robot.subsystems.drive.DriveSubsystem
import kotlin.math.absoluteValue

object Vision {
    val table = NetworkTableInstance.getDefault().getTable("limelight")
         fun getUpdate() = (table.getEntry("tx").getDouble(0.0))

     class VisionAngle(){
         val gyroCurrent = DriveSubsystem.gyro
         val gyroGo = DriveSubsystem.wantedAngle
         fun initalize() {


                    }

         fun isfinished(): Boolean {
             return getUpdate().absoluteValue < 6
         }



     }
}