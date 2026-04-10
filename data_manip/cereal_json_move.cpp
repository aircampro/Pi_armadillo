/*

   Creates a JSON output to move a robot and arm in a sequence 

*/   
#include <iostream>
#include <sstream>
#include <string>
#include <cereal/cereal.hpp>
#include <cereal/archives/json.hpp>
#include <cereal/types/vector.hpp>

// defines 2d movement x,y plane ground robot
struct Vector2 {
	float x = 0.0f;
	float y = 0.0f;
};

// defines angles of 3DOF Arm
struct ThreeDOFArm {
	float angle1 = 0.0f;
	float angle2 = 0.0f;
	float angle3 = 0.0f;
};

template<class Archive>
void serialize(Archive & archive, Vector2 &vector)
{
	archive(cereal::make_nvp("x", vector.x), cereal::make_nvp("y", vector.y));
}

template<class Archive>
void serialize(Archive & archive, ThreeDOFArm &arm)
{
	archive(cereal::make_nvp("angle1", arm.angle1), cereal::make_nvp("angle2", arm.angle2), cereal::make_nvp("angle3", arm.angle3));
}

struct MyJson {
	std::string name;
	int hp = 0;
	Vector2 init_position;
	std::vector<std::string> moves;
	std::vector<Vector2> moves_amt;
	std::vector<ThreeDOFArm> arm_moves;
	std::vector<bool> grip;
	
	template<class Archive>
	void serialize(Archive & archive, std::uint32_t const version)
	{
		archive(CEREAL_NVP(name), CEREAL_NVP(hp), CEREAL_NVP(init_position), CEREAL_NVP(moves), CEREAL_NVP(moves_amt), CEREAL_NVP(arm_moves), CEREAL_NVP(grip));
		if (1 <= version) {
			// archive(...);
		}
	}
};
CEREAL_CLASS_VERSION(MyJson, 1);

int main()
{
	MyJson json_msg;                                    // describe a movement to get an object move arm to it and grip it then move to position and drop it.
	json_msg.name = "MOVEMSG";
	json_msg.hp = 100;                                  // fixed power 
    Vector2 mvs_amt;
    ThreeDOFArm arm_angles;
	json_msg.init_position.x = 5.0f;                    // start position
	json_msg.init_position.y = 12.0f;
	json_msg.moves.push_back("Left");                   // describe movement and co-ordinates and robot states
	mvs_amt.x = 0.0f;
	mvs_amt.y = -2.0f;
	json_msg.moves_amt.push_back(mvs_amt);
	json_msg.grip.push_back(false);
    arm_angles.angle1 = 0.0f;
    arm_angles.angle2 = 0.0f;
    arm_angles.angle3 = 0.0f;
	json_msg.arm_moves.push_back(arm_angles);	
	json_msg.moves.push_back("Forward");
	mvs_amt.x = 5.0f;
	mvs_amt.y = 0.0f;
	json_msg.moves_amt.push_back(mvs_amt);
	json_msg.grip.push_back(false);
	json_msg.arm_moves.push_back(arm_angles);	
	json_msg.moves.push_back("ForwardRight");
	mvs_amt.x = 3.0f;
	mvs_amt.y = 5.0f;
	json_msg.moves_amt.push_back(mvs_amt);
	json_msg.grip.push_back(false);
	json_msg.arm_moves.push_back(arm_angles);	
	json_msg.moves.push_back("NoMove");
	mvs_amt.x = 0.0f;
	mvs_amt.y = 0.0f;
	json_msg.moves_amt.push_back(mvs_amt);
	json_msg.grip.push_back(false);
    arm_angles.angle1 = 45.0f;
    arm_angles.angle2 = 45.0f;
    arm_angles.angle3 = 20.0f;
	json_msg.arm_moves.push_back(arm_angles);	
	json_msg.moves.push_back("NoMove");
	json_msg.moves_amt.push_back(mvs_amt);
	json_msg.grip.push_back(true);
	json_msg.arm_moves.push_back(arm_angles);
	json_msg.moves.push_back("NoMove");
	json_msg.moves_amt.push_back(mvs_amt);
	json_msg.grip.push_back(true);
    arm_angles.angle1 = 0.0f;
    arm_angles.angle2 = 0.0f;
    arm_angles.angle3 = 0.0f;
	json_msg.arm_moves.push_back(arm_angles);
	json_msg.moves.push_back("Backward");
	mvs_amt.x = -10.0f;
	mvs_amt.y = 0.0f;
	json_msg.moves_amt.push_back(mvs_amt);
	json_msg.grip.push_back(false);
	json_msg.arm_moves.push_back(arm_angles);	
	
	std::stringstream ss;
	{
		cereal::JSONOutputArchive o_archive(ss);
		o_archive(cereal::make_nvp("robot_moves", json_msg));
	}
	std::cout << "json message output for robot movements" << std::endl;
	std::cout << ss.str() << std::endl;

# ifdef _MSC_VER
	system("pause");
# endif
	return 0;
}
