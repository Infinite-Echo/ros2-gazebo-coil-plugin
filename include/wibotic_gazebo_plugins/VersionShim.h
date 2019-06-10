#ifndef VERSION_SHIM_H_
#define VERSION_SHIM_H_

#if GAZEBO_MAJOR_VERSION < 9
  #define GetRotation() rot.GetAsEuler().Ign()
  #define GetPosition() pos.Ign()
  #define WorldPose()   GetWorldPose()
#else
  #define GetRotation() Rot().Euler()
  #define GetPosition() Pos()
#endif

#endif // VERSION_SHIM_H_