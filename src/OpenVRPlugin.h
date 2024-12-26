#ifndef __CNOID_OPENVR_PLUGIN_H__
#define __CNOID_OPENVR_PLUGIN_H__

#include <cnoid/Plugin>
#include <cnoid/Signal>
#include "Coordinates.h"
#include "exportdecl.h"

////
namespace cnoid {

struct controllerState {
    controllerState() {
        buttons.resize(5);
        axes.resize(4);
        reset();
    }
    void reset() {
        update = false;
        buttons.assign(5, 0);
    }
    bool update;
    std::vector<int> buttons;
    std::vector<float> axes;
    coordinates coords;
};

class CNOID_EXPORT OpenVRPlugin : public Plugin
{
public:
    OpenVRPlugin();
    ~OpenVRPlugin();

    static OpenVRPlugin* instance();

    virtual bool initialize() override;
    virtual bool finalize() override;
    virtual const char* description() const override;

    void setProjectionMatrix(double scale);
    void setEyeDifferenceScale(double scale);
    void setCameraOrigin(double l_joy_x,double l_joy_y,double r_joy_x,double r_joy_y);
    void rotateCamera(double joystickValue,double rotationscale);
    coordinates CameraOrigin();
    void causeVive(unsigned int sec);

    //Signal
    SignalProxy<void(const controllerState &left, const controllerState &right)> sigUpdateControllerState();
    SignalProxy<void(coordinates &headOrigin)> sigRequestHeadOrigin();

    class Impl;
private:
    Impl *impl;
};

}

#endif
