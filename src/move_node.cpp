#include "evita_obstaculos/move_ros.hpp"

int main(int argc, char** argv) 
{
    ros::init(argc, argv, "move");
    ros::NodeHandle n;

    move_ros programa_navegacion(n);
    programa_navegacion.run();   

  return 0;
}
