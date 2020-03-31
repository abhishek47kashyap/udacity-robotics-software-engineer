#include <gazebo/gazebo.hh>

namespace gazebo
{
	class WorldPluginVehicleRobot : public WorldPlugin
	{
		public: WorldPluginVehicleRobot() : WorldPlugin()
			{printf("Welcome to Abhishek's world!\n");}

		public: void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
			{}
	};
	GZ_REGISTER_WORLD_PLUGIN(WorldPluginVehicleRobot)
}
