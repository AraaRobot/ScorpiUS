#include "QControllerProfile.hpp"
#include "scorpius_main/msg/joy.hpp"
namespace PSConstants
{
    static constexpr const char* svgPath = ":/images/images/Dualshock_4_Layout.svg";
    static constexpr double BUTTON_RADIUS_PERCENT = 0.03;  // in percents of total image width
    static constexpr double JOYSTICK_RADIUS_PERCENT = 0.06;
    static constexpr double HOME_BUTTON_RADIUS_PERCENT = 0.0252;
    static constexpr double OPTION_BUTTON_RADIUS_PERCENT = 0.01;
    static constexpr double BACK_BUTTON_WIDTH = 0.03;
    static constexpr double BACK_BUTTON_LENGTH = 0.05;
}  // namespace PSConstants

namespace XboxConstants
{
    static constexpr const char* svgPath = ":/images/images/xbox-series-x.svg";
    static constexpr double BUTTON_RADIUS_PERCENT = 0.03;
    static constexpr double HOME_BUTTON_RADIUS_PERCENT = 0.0325;
    static constexpr double OPTION_BUTTON_RADIUS_PERCENT = 0.0213;
    static constexpr double JOYSTICK_RADIUS_PERCENT = 0.056;
    static constexpr double BACK_BUTTON_WIDTH = 0.03;
    static constexpr double BACK_BUTTON_LENGTH = 0.05;
}  // namespace XboxConstants

sControllerProfile ps4Profile()
{
    sControllerProfile profile;

    profile.svgPath = QString(PSConstants::svgPath);

    profile.roundButtons
        = {{"cross", QPointF(0.7875, 0.413), PSConstants::BUTTON_RADIUS_PERCENT, false, scorpius_main::msg::Joy::A},
           {"circle", QPointF(0.8545, 0.3128), PSConstants::BUTTON_RADIUS_PERCENT, false, scorpius_main::msg::Joy::B},
           {"square", QPointF(0.7205, 0.3128), PSConstants::BUTTON_RADIUS_PERCENT, false, scorpius_main::msg::Joy::X},
           {"triangle", QPointF(0.7875, 0.2139), PSConstants::BUTTON_RADIUS_PERCENT, false, scorpius_main::msg::Joy::Y},
           {"PS", QPointF(0.5, 0.509), PSConstants::HOME_BUTTON_RADIUS_PERCENT, false, scorpius_main::msg::Joy::HOME},
           {"Share", QPointF(0.315, 0.1785), PSConstants::OPTION_BUTTON_RADIUS_PERCENT, false, scorpius_main::msg::Joy::SHARE},
           {"Options", QPointF(0.6885, 0.1785), PSConstants::OPTION_BUTTON_RADIUS_PERCENT, false, scorpius_main::msg::Joy::OPTS}};

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
                          PSConstants::JOYSTICK_RADIUS_PERCENT,
                          0.0f,
                          0.0f,
                          scorpius_main::msg::Joy::JOYSTICK_LEFT_HORIZ,
                          scorpius_main::msg::Joy::JOYSTICK_LEFT_VERT},
                         {"right",
                          QPointF(0.649, .501),
                          PSConstants::JOYSTICK_RADIUS_PERCENT,
                          0.0f,
                          0.0f,
                          scorpius_main::msg::Joy::JOYSTICK_RIGHT_HORIZ,
                          scorpius_main::msg::Joy::JOYSTICK_RIGHT_VERT}};

    profile.rectButtons = {{{"L1",
                             QPointF(0.010, 0.10),
                             PSConstants::BACK_BUTTON_LENGTH,
                             PSConstants::BACK_BUTTON_WIDTH,
                             false,
                             scorpius_main::msg::Joy::L1},
                            {"L2",
                             QPointF(0.010, 0.19),
                             PSConstants::BACK_BUTTON_LENGTH,
                             PSConstants::BACK_BUTTON_WIDTH,
                             false,
                             scorpius_main::msg::Joy::L2},
                            {"L3",
                             QPointF(0.329, 0.70),
                             PSConstants::BACK_BUTTON_LENGTH,
                             PSConstants::BACK_BUTTON_WIDTH,
                             false,
                             scorpius_main::msg::Joy::L3},
                            {"R1",
                             QPointF(0.940, 0.10),
                             PSConstants::BACK_BUTTON_LENGTH,
                             PSConstants::BACK_BUTTON_WIDTH,
                             false,
                             scorpius_main::msg::Joy::R1},
                            {"R2",
                             QPointF(0.940, 0.19),
                             PSConstants::BACK_BUTTON_LENGTH,
                             PSConstants::BACK_BUTTON_WIDTH,
                             false,
                             scorpius_main::msg::Joy::R2},
                            {"R3",
                             QPointF(0.624, 0.70),
                             PSConstants::BACK_BUTTON_LENGTH,
                             PSConstants::BACK_BUTTON_WIDTH,
                             false,
                             scorpius_main::msg::Joy::R3}}};

    return profile;
}

sControllerProfile xboxProfile()
{
    sControllerProfile profile;
    profile.svgPath = QString(XboxConstants::svgPath);

    profile.roundButtons
        = {{"A", QPointF(0.757, 0.410), XboxConstants::BUTTON_RADIUS_PERCENT, false, scorpius_main::msg::Joy::A},
           {"B", QPointF(0.825, 0.345), XboxConstants::BUTTON_RADIUS_PERCENT, false, scorpius_main::msg::Joy::B},
           {"X", QPointF(0.689, 0.345), XboxConstants::BUTTON_RADIUS_PERCENT, false, scorpius_main::msg::Joy::X},
           {"Y", QPointF(0.757, 0.277), XboxConstants::BUTTON_RADIUS_PERCENT, false, scorpius_main::msg::Joy::Y},
           {"Home", QPointF(0.5, 0.242), XboxConstants::HOME_BUTTON_RADIUS_PERCENT, false, scorpius_main::msg::Joy::HOME},
           {"View", QPointF(0.429, 0.342), XboxConstants::OPTION_BUTTON_RADIUS_PERCENT, false, scorpius_main::msg::Joy::SHARE},
           {"Menu", QPointF(0.574, 0.342), XboxConstants::OPTION_BUTTON_RADIUS_PERCENT, false, scorpius_main::msg::Joy::OPTS}};

    profile.joysticks = {{"Left",
                          QPointF(0.243, 0.343),
                          XboxConstants::JOYSTICK_RADIUS_PERCENT,
                          0.0f,
                          0.0f,
                          scorpius_main::msg::Joy::JOYSTICK_LEFT_HORIZ,
                          scorpius_main::msg::Joy::JOYSTICK_LEFT_VERT},
                         {"Right",
                          QPointF(0.633, 0.493),
                          XboxConstants::JOYSTICK_RADIUS_PERCENT,
                          0.0f,
                          0.0f,
                          scorpius_main::msg::Joy::JOYSTICK_RIGHT_HORIZ,
                          scorpius_main::msg::Joy::JOYSTICK_RIGHT_VERT}};

    profile.polyButtons = {
        {"Up", {{QPointF(0.34, 0.48), QPointF(0.395, 0.48), QPointF(0.3675, 0.43)}}, false, scorpius_main::msg::Joy::CROSS_UP},
        {"Down",
         {{QPointF(0.34, 0.53), QPointF(0.395, 0.53), QPointF(0.3675, 0.58)}},
         false,
         scorpius_main::msg::Joy::CROSS_DOWN},
        {"Left", {{QPointF(0.34, 0.48), QPointF(0.34, 0.53), QPointF(0.29, 0.505)}}, false, scorpius_main::msg::Joy::CROSS_LEFT},
        {"Right",
         {{QPointF(0.395, 0.48), QPointF(0.395, 0.53), QPointF(0.445, 0.505)}},
         false,
         scorpius_main::msg::Joy::CROSS_RIGHT}};

    profile.rectButtons = {{{"LB",
                             QPointF(0.010, 0.10),
                             XboxConstants::BACK_BUTTON_LENGTH,
                             XboxConstants::BACK_BUTTON_WIDTH,
                             false,
                             scorpius_main::msg::Joy::L1},
                            {"LT",
                             QPointF(0.010, 0.19),
                             XboxConstants::BACK_BUTTON_LENGTH,
                             XboxConstants::BACK_BUTTON_WIDTH,
                             false,
                             scorpius_main::msg::Joy::L2},
                            {"L3",
                             QPointF(0.329, 0.70),
                             XboxConstants::BACK_BUTTON_LENGTH,
                             XboxConstants::BACK_BUTTON_WIDTH,
                             false,
                             scorpius_main::msg::Joy::L3},
                            {"RB",
                             QPointF(0.940, 0.10),
                             XboxConstants::BACK_BUTTON_LENGTH,
                             XboxConstants::BACK_BUTTON_WIDTH,
                             false,
                             scorpius_main::msg::Joy::R1},
                            {"RT",
                             QPointF(0.940, 0.19),
                             XboxConstants::BACK_BUTTON_LENGTH,
                             XboxConstants::BACK_BUTTON_WIDTH,
                             false,
                             scorpius_main::msg::Joy::R2},
                            {"R3",
                             QPointF(0.624, 0.70),
                             XboxConstants::BACK_BUTTON_LENGTH,
                             XboxConstants::BACK_BUTTON_WIDTH,
                             false,
                             scorpius_main::msg::Joy::R3}}};

    return profile;
}