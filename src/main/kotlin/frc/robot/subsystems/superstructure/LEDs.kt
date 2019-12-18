package frc.robot.subsystems.superstructure
import com.ctre.phoenix.CANifier
import edu.wpi.first.wpilibj2.command.CommandScheduler
import frc.robot.Vision
import frc.robot.subsystems.drive.DriveSubsystem
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.commands.FalconSubsystem
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.Second
import org.ghrobotics.lib.mathematics.units.millisecond
import org.ghrobotics.lib.mathematics.units.second
import org.ghrobotics.lib.wrappers.FalconSolenoid
import java.awt.Color

object LEDs {


 class LedCorrect {
  sealed class State(open val color: Color) {
   object Default : Solid(Color.red) // sets default to red
   open class Solid(override val color: Color) : State(color)
   object Off : Solid(Color.black)
   class Blink(val blinkTime: SIUnit<Second>, override val color: Color) : State(color)
  }
  private var wantedState: State = State.Default

  fun ledColor(){
   var superstructureCommand = Superstructure.currentCommand
   var driveCommand = DriveSubsystem.currentCommand
                                        //Used || as "or"
   wantedState = if(driveCommand is Vision.AimAtVisionTarget || driveCommand is Vision.VisionGoToTarget)
    State.Solid(Color.green) else State.Default

   wantedState = if(superstructureCommand is Superstructure.PresetCommand)
    State.Solid(Color.green) else State.Default


  }
 }
}





