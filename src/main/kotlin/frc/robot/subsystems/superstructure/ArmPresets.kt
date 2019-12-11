package frc.robot.subsystems.superstructure
import org.ghrobotics.lib.mathematics.units.derived.degrees
import org.ghrobotics.lib.mathematics.units.inches

object ArmPresets {
            //TODO make the proper presets


        fun cargoLow(){
                //todo Make elavator make the proximal not hit and find "0"
            //ElevatorPresets.elevatorGoToPreset(height = 30.inches)
            Proximal.ProximalPreset(ProximalAngle = 90.degrees)
            Wrist.wristPreset(WristAngle = 10.degrees)
            ElevatorPresets.elevatorGoToPreset(height = 5.inches)
        }
        fun cargoMid(){
//todo Make elavator make the proximal not hit and find "0"
            //ElevatorPresets.elevatorGoToPreset(height = 30.inches)
            Proximal.ProximalPreset(ProximalAngle = 90.degrees)
            Wrist.wristPreset(WristAngle = 10.degrees)
            ElevatorPresets.elevatorGoToPreset(height = 25.inches)
        }
        fun cargoHigh(){
//todo Make elavator make the proximal not hit and find "0"
            //ElevatorPresets.elevatorGoToPreset(height = 30.inches)
            Proximal.ProximalPreset(ProximalAngle = 90.degrees)
            Wrist.wristPreset(WristAngle = 10.degrees)
            ElevatorPresets.elevatorGoToPreset(height = 45.inches)
        }
        fun hatchLow(){
//todo Make elavator make the proximal not hit and find "0"
            //ElevatorPresets.elevatorGoToPreset(height = 30.inches)
            Proximal.ProximalPreset(ProximalAngle = 90.degrees)
            Wrist.wristPreset(WristAngle = 0.degrees)
            ElevatorPresets.elevatorGoToPreset(height = 5.inches)
        }
        fun hatchMid(){
//todo Make elavator make the proximal not hit and find "0"
            //ElevatorPresets.elevatorGoToPreset(height = 30.inches)
            Proximal.ProximalPreset(ProximalAngle = 90.degrees)
            Wrist.wristPreset(WristAngle = 0.degrees)
            ElevatorPresets.elevatorGoToPreset(height = 25.inches)
        }
        fun hatchHigh(){
//todo Make elavator make the proximal not hit and find "0"
            //ElevatorPresets.elevatorGoToPreset(height = 30.inches)
            Proximal.ProximalPreset(ProximalAngle = 90.degrees)
            Wrist.wristPreset(WristAngle = 0.degrees)
            ElevatorPresets.elevatorGoToPreset(height = 45.inches)
        }
        fun stowed(){
            //todo Make elavator make the proximal not hit and find "0"
            //ElevatorPresets.elevatorGoToPreset(height = 30.inches)
            Proximal.ProximalPreset(ProximalAngle = 80.degrees)
            Wrist.wristPreset(WristAngle = 10.degrees)
            ElevatorPresets.elevatorGoToPreset(height = -(5.inches))
        }
        fun home(){
            //todo Make elavator make the proximal not hit and find "0"
            //ElevatorPresets.elevatorGoToPreset(height = 30.inches)
            ElevatorPresets.elevatorGoToPreset(height = 40.inches)
            Proximal.ProximalPreset(ProximalAngle = 0.degrees)
            Wrist.wristPreset(WristAngle = 0.degrees)
            ElevatorPresets.elevatorGoToPreset(height = 0.inches)
        }



    }
