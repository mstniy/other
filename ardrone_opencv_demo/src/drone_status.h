#ifndef CPP_DRONE_STATUS_H
#define CPP_DRONE_STATUS_H

// An enumeration of drone statuses for the tutorial "Up and flying with the AR.Drone and ROS | Getting Started"
// https://github.com/mikehamer/ardrone_tutorials_getting_started

enum class DroneStatus : int32_t
{
	Unknown   = -1,
	Emergency = 0,
	Inited    = 1,
	Landed    = 2,
	Flying    = 3,
	Hovering  = 4,
	Test      = 5,
	TakingOff = 6,
	GotoHover = 7,
	Landing   = 8,
	Looping   = 9
};

// We specialize std::hash for DroneStatus so that we can use it as the key of std::map's.

namespace std {

  template <>
  struct hash<DroneStatus>
  {
    std::size_t operator()(const DroneStatus& k) const
    {
	return static_cast<size_t>(k);
    }
  };

}

#endif
