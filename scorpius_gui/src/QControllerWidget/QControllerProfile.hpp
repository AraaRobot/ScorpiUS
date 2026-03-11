#ifndef QCONTROLLER_PROFILE_HPP
#define QCONTROLLER_PROFILE_HPP

#include <QString>
#include <QPointF>

#include <vector>

// Button & axis structures

struct sButtonRound
{
    QString name;
    QPointF posNorm;  // normalized x,y (0..1)
    double radius;
    bool pressed;
    int8_t joyIndex{-1};
};

struct sButtonRect
{
    QString name;
    QPointF posNorm;  // normalized x,y (0..1)
    double length;
    double width;
    bool pressed;
    int8_t joyIndex{-1};
};

struct sButtonPoly
{
    QString name;
    std::vector<QPointF> posNorm;
    bool pressed;
    int8_t joyIndex{-1};
};

struct sJoystick
{
    QString name;
    QPointF center;
    double radius;
    float xPos;
    float yPos;
    int8_t joyIndexHoriz{-1};
    int8_t joyIndexVert{-1};
};

struct sControllerProfile
{
    QString svgPath;
    std::vector<sButtonRound> roundButtons;
    std::vector<sButtonPoly> polyButtons;
    std::vector<sJoystick> joysticks;
    std::vector<sButtonRect> rectButtons;
};

// Factory

sControllerProfile ps4Profile();
sControllerProfile xboxProfile();

#endif