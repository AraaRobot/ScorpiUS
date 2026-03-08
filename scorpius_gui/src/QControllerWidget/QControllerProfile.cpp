#include "QControllerProfile.hpp"
#include "scorpius_main/msg/joy.hpp"

namespace PSconstants
{
    static constexpr const char* svgPath = ":/images/images/Dualshock_4_Layout.svg";
    static constexpr double BUTTON_RADIUS_PERCENT = 0.03;  // in percents of total image width
    static constexpr double JOYSTICK_RADIUS_PERCENT = 0.06;
    static constexpr double HOME_BUTTON_RADIUS_PERCENT = 0.0252;
    static constexpr double OPTION_BUTTON_RADIUS_PERCENT = 0.01;
    static constexpr double BACK_BUTTON_WIDTH = 0.03;
    static constexpr double BACK_BUTTON_LENGTH = 0.05;
}  // namespace PSconstants

sControllerProfile ps4Profile()
{
    sControllerProfile profile;

    profile.svgPath = QString(PSconstants::svgPath);

    profile.roundButtons
        = {{"cross", QPointF(0.7875, 0.413), PSconstants::BUTTON_RADIUS_PERCENT, false, scorpius_main::msg::Joy::A},
           {"circle", QPointF(0.8545, 0.3128), PSconstants::BUTTON_RADIUS_PERCENT, false, scorpius_main::msg::Joy::B},
           {"square", QPointF(0.7205, 0.3128), PSconstants::BUTTON_RADIUS_PERCENT, false, scorpius_main::msg::Joy::X},
           {"triangle", QPointF(0.7875, 0.2139), PSconstants::BUTTON_RADIUS_PERCENT, false, scorpius_main::msg::Joy::Y},
           {"PS", QPointF(0.5, 0.509), PSconstants::HOME_BUTTON_RADIUS_PERCENT, false, scorpius_main::msg::Joy::HOME},
           {"Share", QPointF(0.315, 0.1785), PSconstants::OPTION_BUTTON_RADIUS_PERCENT, false, scorpius_main::msg::Joy::SHARE},
           {"Options", QPointF(0.6885, 0.1785), PSconstants::OPTION_BUTTON_RADIUS_PERCENT, false, scorpius_main::msg::Joy::OPTS}};

    profile.polyButtons
        = {{"up",
            {{QPointF(0.19, 0.205), QPointF(0.19, 0.26), QPointF(0.215, 0.293), QPointF(0.24, 0.26), QPointF(0.24, 0.205)}},
            false,
            scorpius_main::msg::Joy::CROSS_UP},
           {"down",
            {{QPointF(0.19, 0.42), QPointF(0.19, 0.365), QPointF(0.215, 0.335), QPointF(0.24, 0.365), QPointF(0.24, 0.42)}},
            false,
            scorpius_main::msg::Joy::CROSS_DOWN},
           {"left",
            {{QPointF(0.14, 0.279), QPointF(0.18, 0.279), QPointF(0.20, 0.31), QPointF(0.18, 0.347), QPointF(0.14, 0.347)}},
            false,
            scorpius_main::msg::Joy::CROSS_LEFT},
           {"right",
            {{QPointF(0.287, 0.279), QPointF(0.247, 0.279), QPointF(0.227, 0.31), QPointF(0.247, 0.347), QPointF(0.287, 0.347)}},
            false,
            scorpius_main::msg::Joy::CROSS_RIGHT}};

    profile.joysticks = {{"left",
                          QPointF(0.354, .501),
                          PSconstants::JOYSTICK_RADIUS_PERCENT,
                          0.0f,
                          0.0f,
                          scorpius_main::msg::Joy::JOYSTICK_LEFT_HORIZ,
                          scorpius_main::msg::Joy::JOYSTICK_LEFT_VERT},
                         {"right",
                          QPointF(0.649, .501),
                          PSconstants::JOYSTICK_RADIUS_PERCENT,
                          0.0f,
                          0.0f,
                          scorpius_main::msg::Joy::JOYSTICK_RIGHT_HORIZ,
                          scorpius_main::msg::Joy::JOYSTICK_RIGHT_VERT}};

    profile.rectButtons = {{{"L1",
                             QPointF(0.010, 0.10),
                             PSconstants::BACK_BUTTON_LENGTH,
                             PSconstants::BACK_BUTTON_WIDTH,
                             false,
                             scorpius_main::msg::Joy::L1},
                            {"L2",
                             QPointF(0.010, 0.19),
                             PSconstants::BACK_BUTTON_LENGTH,
                             PSconstants::BACK_BUTTON_WIDTH,
                             false,
                             scorpius_main::msg::Joy::L2},
                            {"L3",
                             QPointF(0.329, 0.70),
                             PSconstants::BACK_BUTTON_LENGTH,
                             PSconstants::BACK_BUTTON_WIDTH,
                             false,
                             scorpius_main::msg::Joy::L3},
                            {"R1",
                             QPointF(0.940, 0.10),
                             PSconstants::BACK_BUTTON_LENGTH,
                             PSconstants::BACK_BUTTON_WIDTH,
                             false,
                             scorpius_main::msg::Joy::R1},
                            {"R2",
                             QPointF(0.940, 0.19),
                             PSconstants::BACK_BUTTON_LENGTH,
                             PSconstants::BACK_BUTTON_WIDTH,
                             false,
                             scorpius_main::msg::Joy::R2},
                            {"R3",
                             QPointF(0.624, 0.70),
                             PSconstants::BACK_BUTTON_LENGTH,
                             PSconstants::BACK_BUTTON_WIDTH,
                             false,
                             scorpius_main::msg::Joy::R3}}};

    return profile;
}

sControllerProfile xboxProfile()
{
    sControllerProfile profile;

    return profile;
}