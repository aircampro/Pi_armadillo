// Example of moving DJI Tello drone using this library
// https://github.com/hakuturu583/tello_driver
//
#include <tello_driver/udp_client.h>
#include <tello_driver/tello_command_builder.h>

int main()
{
	using namespace tello_driver::tello_commands;
    tello_driver::UdpClient client;
    tello_driver::TelloCommandBuilder builder;
    // init and take off
    client.send(tello_driver::TELLO_IP,tello_driver::TELLO_PORT_CMD,builder.buildInitCommand());
    client.send(tello_driver::TELLO_IP,tello_driver::TELLO_PORT_CMD,builder.buildTakeOffCommand());
    // try video on
    client.send(tello_driver::TELLO_IP,tello_driver::TELLO_PORT_VIDEO,builder.buildMoveCommand(mission_commands::STREAM_ON,0));
    // go up by 100
    client.send(tello_driver::TELLO_IP,tello_driver::TELLO_PORT_CMD,builder.buildMoveCommand(move_commands::UP,100));
    // go up forward 200
    client.send(tello_driver::TELLO_IP,tello_driver::TELLO_PORT_CMD,builder.buildMoveCommand(move_commands::FORWARD,200));
    // go left 20
    client.send(tello_driver::TELLO_IP,tello_driver::TELLO_PORT_CMD,builder.buildMoveCommand(move_commands::LEFT,20));
    // go up forward 100
    client.send(tello_driver::TELLO_IP,tello_driver::TELLO_PORT_CMD,builder.buildMoveCommand(move_commands::FORWARD,100));
    // flip
    client.send(tello_driver::TELLO_IP,tello_driver::TELLO_PORT_CMD,builder.buildMoveCommand(move_commands::FLIP,0));
    // go up by 100
    client.send(tello_driver::TELLO_IP,tello_driver::TELLO_PORT_CMD,builder.buildMoveCommand(move_commands::UP,100));
    // go right 20
    client.send(tello_driver::TELLO_IP,tello_driver::TELLO_PORT_CMD,builder.buildMoveCommand(move_commands::RIGHT,20));
    // go up backward 100
    client.send(tello_driver::TELLO_IP,tello_driver::TELLO_PORT_CMD,builder.buildMoveCommand(move_commands::BACK,100));
    // counter clockwise
    client.send(tello_driver::TELLO_IP,tello_driver::TELLO_PORT_CMD,builder.buildMoveCommand(move_commands::COUNTER_CLOCKWISE,100));
    // clockwise
    client.send(tello_driver::TELLO_IP,tello_driver::TELLO_PORT_CMD,builder.buildMoveCommand(move_commands::CLOCKWISE,100));
    // go to 12,13,400 position
    client.send(tello_driver::TELLO_IP,tello_driver::TELLO_PORT_CMD,builder.buildMoveCommand(move_commands::GO,12,13,400));
    // move in a curve
    client.send(tello_driver::TELLO_IP,tello_driver::TELLO_PORT_CMD,builder.buildMoveCommand(move_commands::CURVE,120));
    // try video off
    client.send(tello_driver::TELLO_IP,tello_driver::TELLO_PORT_VIDEO,builder.buildMoveCommand(mission_commands::STREAM_OFF,0));
    // land
    client.send(tello_driver::TELLO_IP,tello_driver::TELLO_PORT_CMD,builder.buildLandCommand());
    return 0;
}