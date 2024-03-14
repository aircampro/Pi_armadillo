--   Sensor_Fusion_Get_Euler.adb                  
with Ada.Text_IO; use Ada.Text_IO;
with Ada.Numerics; use Ada.Numerics;

with Ada.Numerics.Elementary_Functions;
use  Ada.Numerics.Elementary_Functions;

procedure Sensor_Fusion_Get_Euler is

   type RPY is (Roll, Pitch, Yaw);

   function sensfusion6GetEulerRPY(gx, gy, gz, q0, q1, q2, q3 : Float; axis : RPY)
                          return Standard.Float is
   begin
      return
        ( case axis is
          when Roll  => Arctan(gy, gz) * 180.0 / (4.0*Arctan(1.0)),
          when Pitch => Arcsin(gx) * 180.0 / (4.0*Arctan(1.0)),
          when Yaw   => Arctan((2.0*(q0*q3 + q1*q2)), (q0*q0 + q1*q1 - q2*q2 - q3*q3)) 180.0 / (4.0*Arctan(1.0)) );
   end sensfusion6GetEulerRPY;

   --                      (Unit vector in the estimated gravity direction
   gx : Float := 0.1;
   gy : Float := 10.0;
   gz : Float := 10.0;

   --                      quaternion of sensor frame relative to auxiliary frame
   q0 : Float := 1.0;
   q1 : Float := 0.2;
   q2 : Float := 0.1;
   q3 : Float := 0.1;

   --                      choose the axis and start with the pitch
   axis : RPY := Pitch;   
begin

   Put_Line ("sensor fusion get euler RPY pitch = "
             & Float'Image(sensfusion6GetEulerRPY(gx, gy, gz, q0, q1, q2, q3, axis)));
   axis := Yaw;
   Put_Line ("sensor fusion get euler RPY yaw = "
             & Float'Image(sensfusion6GetEulerRPY(gx, gy, gz, q0, q1, q2, q3, axis)));
   axis := Roll;
   Put_Line ("sensor fusion get euler RPY roll = "
             & Float'Image(sensfusion6GetEulerRPY(gx, gy, gz, q0, q1, q2, q3, axis)));
end Sensor_Fusion_Get_Euler;