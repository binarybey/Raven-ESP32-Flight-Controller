#pragma once
/*
4 Degrees of Freedom Manuever
1-Throttle (Lift)
2-Roll
3-Pitch
4-Yaw
---------------------
Hybrid Control = Control of Hover + Control of Forward Flight
alpha = Nacelle Angle with Vertical
---------------------
Lift = Lift_Hover * cos²(alpha) + Lift_ForwardFlight * sin²(alpha)
Yaw_Aggression_ForwardFlight = Rudder * cos²(BankAngle) + Elevator * sin²(BankAngle)
Pitch_Aggression_ForwardFlight = Elevator * cos²(BankAngle) + Rudder * sin²(BankAngle)
Rudder_Aggression_YawFF = cos²(BankAngle) * Yaw_Error
Elevator_Aggression_YawFF = sin²(BankAngle) * Yaw_Error
Rudder_Aggression_PitchFF = sin²(BankAngle) * Pitch_Error
Elevator_Aggression_PitchFF = cos²(BankAngle) * Pitch_Error
---------------------
Yaw_Hover = Oppose Nacelles * cos²(alpha) + Rudder * sin²(alpha)
*/

extern float nacelleAngle_avg;


