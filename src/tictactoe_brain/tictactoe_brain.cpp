#include "tictactoeBrain.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "tictactoe_brain");

    panda_tictactoe::tictactoeBrain brain("ttt_controller");

    ros::spin();
    return 0;
}
