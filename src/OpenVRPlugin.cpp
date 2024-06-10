#include <fmt/format.h>
#define IRSL_DEBUG
#include "irsl_debug.h"

#include <cnoid/GLSLSceneRenderer>
#include <cnoid/SceneWidget>
#include <cnoid/SceneView>
#include <cnoid/SceneCameras>
#include <cnoid/MessageView>
#include <cnoid/App>
#include <cnoid/Timer>

#include "OpenVRPlugin.h"
#include "CameraControlJoyItem.h"

#include "OffscreenGL.h"
#ifdef _WIN32
#include <openvr.h>
#endif // _WIN32

#include <QElapsedTimer>

using namespace cnoid;

namespace {
OpenVRPlugin* instance_ = nullptr;
}

#ifdef _WIN32
void setStateToStruct(const vr::VRControllerState_t &state,
                      struct controllerState &res) {
    res.update = true;
    // ButtonA
    if((1LL << vr::k_EButton_A) & state.ulButtonPressed)
        res.buttons[0] = 1;
    // ButtonB
    if((1LL << vr::k_EButton_ApplicationMenu) & state.ulButtonPressed)
        res.buttons[1] = 1;
    // Stick
    if((1LL << vr::k_EButton_SteamVR_Touchpad) & state.ulButtonPressed)
        res.buttons[2] = 1;
    // Finger0
    if((1LL << vr::k_EButton_SteamVR_Trigger) & state.ulButtonPressed)
        res.buttons[3] = 1;
    // Finger1
    if((1LL << vr::k_EButton_Grip) & state.ulButtonPressed)
        res.buttons[4] = 1;
    // Stick
    res.axes[0] = state.rAxis[0].x;
    res.axes[1] = state.rAxis[0].y;
    // Finger0
    res.axes[2] = state.rAxis[1].x;
    // Finger1
    res.axes[3] = state.rAxis[2].x;
}
#endif // _WIN32

//// Impl
class OpenVRPlugin::Impl
{
public:
    Impl();
    void initialize();
#ifdef _WIN32
    void singleLoop();
    void updatePoses();
    bool getDeviceString(std::string &_res, int index, vr::TrackedDeviceProperty prop);

    void setToCoords(const vr::HmdMatrix34_t &hmd_mat, coordinates &cds);
public:
    Timer tm;
    unsigned long counter;
    double publishingRate;

    controllerState state_L, state_R;

    OffscreenGL offGL;

    vr::IVRSystem *m_pHMD;

    unsigned int nWidth, nHeight;
    unsigned int ui_L_TextureId;
    unsigned int ui_L_FramebufferId;
    unsigned int ui_R_TextureId;
    unsigned int ui_R_FramebufferId;

    vr::TrackedDevicePose_t TrackedDevicePoses[ vr::k_unMaxTrackedDeviceCount ];
    // Eigen // devicePoses
    std::vector<Isometry3d> devicePoses;
    std::vector<std::string> deviceNames;
    std::vector<int> deviceClasses;

    Matrix4 projection_L;
    Matrix4 projection_R;

    coordinates eyeToHead_L;
    coordinates eyeToHead_R;
    coordinates HMD_coords;
    coordinates origin_to_HMD;
    coordinates origin;

    //
    QElapsedTimer qtimer;
#endif // _WIN32
    //
    std::ostream *os_;

    Signal<bool(const controllerState &left, const controllerState &right)> updateControllerState;;
    Signal<void(coordinates &headOrigin)> requestHeadOrigin;
};

//// >>>> Impl
OpenVRPlugin::Impl::Impl()
{
#ifdef _WIN32
    m_pHMD = nullptr;
    devicePoses.resize(vr::k_unMaxTrackedDeviceCount);
    deviceNames.resize(vr::k_unMaxTrackedDeviceCount);
    deviceClasses.resize(vr::k_unMaxTrackedDeviceCount);
    counter = 0;
#endif // _WIN32
    os_ = nullptr;
}
void OpenVRPlugin::Impl::initialize()
{
    os_ = &(MessageView::instance()->cout(false));

#ifdef _WIN32
    origin.pos << -3.0, 0.0, 0.0;
    {
        Quaternion q(0.5, 0.5, -0.5, -0.5);
        origin_to_HMD.set(q);
    }

    vr::EVRInitError eError = vr::VRInitError_None;
    m_pHMD = vr::VR_Init( &eError, vr::VRApplication_Scene );
    if ( eError != vr::VRInitError_None ) {
        m_pHMD = NULL;
        *os_ << "Unable to init VR runtime:  " << vr::VR_GetVRInitErrorAsEnglishDescription( eError ) << std::endl;
        return;
    }
    m_pHMD->GetRecommendedRenderTargetSize( &nWidth, &nHeight );
    *os_ << "width x height = " << nWidth << " x  " << nHeight << std::endl;
    vr::HmdMatrix44_t l_mat = m_pHMD->GetProjectionMatrix( vr::Eye_Left,  0.01f, 15.0f );
    vr::HmdMatrix44_t r_mat = m_pHMD->GetProjectionMatrix( vr::Eye_Right, 0.01f, 15.0f );
    vr::HmdMatrix34_t l_eye = m_pHMD->GetEyeToHeadTransform( vr::Eye_Left );
    vr::HmdMatrix34_t r_eye = m_pHMD->GetEyeToHeadTransform( vr::Eye_Right );
#if 1 // DEBUG_PRINT
    *os_ << "left_projection" << std::endl;
    *os_ << l_mat.m[0][0] << ", " << l_mat.m[0][1] << ", " << l_mat.m[0][2] << ", " << l_mat.m[0][3] << std::endl;
    *os_ << l_mat.m[1][0] << ", " << l_mat.m[1][1] << ", " << l_mat.m[1][2] << ", " << l_mat.m[1][3] << std::endl;
    *os_ << l_mat.m[2][0] << ", " << l_mat.m[2][1] << ", " << l_mat.m[2][2] << ", " << l_mat.m[2][3] << std::endl;
    *os_ << l_mat.m[3][0] << ", " << l_mat.m[3][1] << ", " << l_mat.m[3][2] << ", " << l_mat.m[3][3] << std::endl;
    //
    *os_ << "left_eye_to_head" << std::endl;
    *os_ << l_eye.m[0][0] << ", " << l_eye.m[0][1] << ", " << l_eye.m[0][2] << ", " << l_eye.m[0][3] << std::endl;
    *os_ << l_eye.m[1][0] << ", " << l_eye.m[1][1] << ", " << l_eye.m[1][2] << ", " << l_eye.m[1][3] << std::endl;
    *os_ << l_eye.m[2][0] << ", " << l_eye.m[2][1] << ", " << l_eye.m[2][2] << ", " << l_eye.m[2][3] << std::endl;
#endif
    projection_L << l_mat.m[0][0], l_mat.m[0][1], l_mat.m[0][2], l_mat.m[0][3],
                    l_mat.m[1][0], l_mat.m[1][1], l_mat.m[1][2], l_mat.m[1][3],
                    l_mat.m[2][0], l_mat.m[2][1], l_mat.m[2][2], l_mat.m[2][3],
                    l_mat.m[3][0], l_mat.m[3][1], l_mat.m[3][2], l_mat.m[3][3];
    projection_R << r_mat.m[0][0], r_mat.m[0][1], r_mat.m[0][2], r_mat.m[0][3],
                    r_mat.m[1][0], r_mat.m[1][1], r_mat.m[1][2], r_mat.m[1][3],
                    r_mat.m[2][0], r_mat.m[2][1], r_mat.m[2][2], r_mat.m[2][3],
                    r_mat.m[3][0], r_mat.m[3][1], r_mat.m[3][2], r_mat.m[3][3];
    setToCoords(l_eye, eyeToHead_L);
    setToCoords(r_eye, eyeToHead_R);

    {
        bool glres = offGL.create();
        *os_ << "offGL: create: " << glres << std::endl;
        glres = offGL.makeCurrent();
        *os_ << "offGL: current: " << glres << std::endl;
        glres = offGL.makeBuffer(nWidth, nHeight, &ui_R_TextureId, &ui_R_FramebufferId);
        *os_ << "offGL: buffer: " << glres << std::endl;
        glres = offGL.makeBuffer(nWidth, nHeight, &ui_L_TextureId, &ui_L_FramebufferId);
        *os_ << "offGL: buffer: " << glres << std::endl;
        offGL.glFinish();
        offGL.glFlush();
        offGL.context->doneCurrent();
        OffscreenGL::printSurfaceFormat(offGL.context->format(), *os_);
    }

    offGL.makeCurrent();
    if ( !vr::VRCompositor() ) {
        *os_ << "Compositor initialization failed. See log file for details" << std::endl;
        return;
    }
    offGL.glFinish();
    offGL.glFlush();
    offGL.context->doneCurrent();

    //// choreonoid settings
    std::vector<SceneView *> view_instances = SceneView::instances();
    if (view_instances.size() > 2) {
        view_instances.at(1)->sceneWidget()->setScreenSize(nWidth, nHeight);
        view_instances.at(2)->sceneWidget()->setScreenSize(nWidth, nHeight);
        {
            GLSceneRenderer *glsr = view_instances.at(1)->sceneWidget()->renderer<GLSceneRenderer>();
            GLSLSceneRenderer *sl = static_cast<GLSLSceneRenderer *>(glsr);
            sl->setUserProjectionMatrix(projection_L);
        }
        {
            GLSceneRenderer *glsr = view_instances.at(2)->sceneWidget()->renderer<GLSceneRenderer>();
            GLSLSceneRenderer *sl = static_cast<GLSLSceneRenderer *>(glsr);
            sl->setUserProjectionMatrix(projection_R);
        }
    }

    tm.sigTimeout().connect( [this]() { this->singleLoop(); });

    //int interval_ms = 1000/30;
    int interval_ms = 2;

    tm.start(interval_ms);
#endif // _WIN32
}

#ifdef _WIN32
void OpenVRPlugin::Impl::singleLoop()
{
    if (!!m_pHMD) {
        if (counter % 30 == 0) {
            *os_ << "fps = " << (1000.0 * 30.0)/(qtimer.elapsed()+1) << std::endl;
            qtimer.start();
        }
        counter++;

        std::vector<SceneView *> view_instances = SceneView::instances();
        if (view_instances.size() < 2) {
            *os_ << "scene less than 3" << std::endl;
            return;
        }

        /// update camera pose
        // set_camera
        impl->requestHeadOrigin(origin);

        coordinates cds = origin;
        cds.transform(origin_to_HMD);
        cds.transform(HMD_coords);
        // left camera
        coordinates cds_l = cds;
        cds_l.transform(eyeToHead_L);
        {
            Isometry3 cur;
            cds_l.toPosition(cur);
            view_instances.at(1)->sceneWidget()->builtinCameraTransform()->setPosition(cur);
        }
        // right camera
        coordinates cds_r = cds;
        cds_r.transform(eyeToHead_R);
        {
            Isometry3 cur;
            cds_r.toPosition(cur);
            view_instances.at(2)->sceneWidget()->builtinCameraTransform()->setPosition(cur);
        }
        view_instances.at(1)->sceneWidget()->renderScene(true);//
        view_instances.at(2)->sceneWidget()->renderScene(true);//
        QImage tmp_im_l = view_instances.at(1)->sceneWidget()->getImage();
        QImage tmp_im_r = view_instances.at(2)->sceneWidget()->getImage();

        offGL.makeCurrent();
        QImage im_l = tmp_im_l.convertToFormat(QImage::Format_RGB888).mirrored(false, true);
        QImage im_r = tmp_im_r.convertToFormat(QImage::Format_RGB888).mirrored(false, true);
        //QImage im_r.convertTo(QImage::Format_RGB888);
        //QImage im_l.convertTo(QImage::Format_RGB888);
        ////
        //*os_ << "up: " << im_r.width() << " x " << im_r.height() << " / " << im_r.bytesPerLine() << std::endl;
        ////
        offGL.writeTexture(ui_R_TextureId, im_r.bits(), nWidth, nHeight, 0, 0);
        offGL.writeTexture(ui_L_TextureId, im_l.bits(), nWidth, nHeight, 0, 0);
        vr::Texture_t leftEyeTexture =  {(void*)(uintptr_t)ui_L_TextureId, vr::TextureType_OpenGL, vr::ColorSpace_Gamma };
        auto resL = vr::VRCompositor()->Submit(vr::Eye_Left,  &leftEyeTexture );
        vr::Texture_t rightEyeTexture = {(void*)(uintptr_t)ui_R_TextureId, vr::TextureType_OpenGL, vr::ColorSpace_Gamma };
        auto resR = vr::VRCompositor()->Submit(vr::Eye_Right, &rightEyeTexture );
        if (resL != 0) {
            *os_ << "L: " << resL << std::endl;
        }
        if (resR != 0) {
            *os_ << "R: " << resR << std::endl;
        }
        updatePoses();
        //vr::Compositor_FrameTiming tmg;
        //bool tm_q = vr::VRCompositor()->GetFrameTiming(&tmg);
    }
}

bool OpenVRPlugin::Impl::getDeviceString(std::string &_res, int index, vr::TrackedDeviceProperty prop)
{
    // prop
    //vr::Prop_RenderModelName_String
    //vr::Prop_TrackingSystemName_String
    //vr::Prop_SerialNumber_String
    vr::TrackedPropertyError p_error;
    uint32_t len = m_pHMD->GetStringTrackedDeviceProperty( index, prop, NULL, 0, &p_error );
    if( len == 0 ) {
        return false;
    }
    char *buf_ = new char[ len ];
    len = m_pHMD->GetStringTrackedDeviceProperty( index, prop, buf_, len, &p_error );
    _res = buf_;
    delete [] buf_;
    return true;
}

void OpenVRPlugin::Impl::updatePoses()
{
    state_L.reset();
    state_R.reset();

    if ( !m_pHMD ) {
        return;
    }

    vr::VRCompositor()->WaitGetPoses(TrackedDevicePoses, vr::k_unMaxTrackedDeviceCount, NULL, 0 );

    int validPoseCount = 0;
    for ( int idx = 0; idx < vr::k_unMaxTrackedDeviceCount; idx++ ) {
        int cls = m_pHMD->GetTrackedDeviceClass(idx);
        if ( TrackedDevicePoses[idx].bPoseIsValid ) {
            *os_ << "t: " << idx << " / " << cls << " ";
            *os_ << TrackedDevicePoses[idx].bDeviceIsConnected << " " << TrackedDevicePoses[idx].eTrackingResult << std::endl;
            validPoseCount++;
            deviceClasses[idx] = cls;
            const vr::HmdMatrix34_t &mat = TrackedDevicePoses[idx].mDeviceToAbsoluteTracking;

            Matrix4d eMat;
            eMat << mat.m[0][0], mat.m[0][1], mat.m[0][2], mat.m[0][3],
                    mat.m[1][0], mat.m[1][1], mat.m[1][2], mat.m[1][3],
                    mat.m[2][0], mat.m[2][1], mat.m[2][2], mat.m[2][3],
                    0.0, 0.0, 0.0, 1.0;

            devicePoses[idx].matrix() = eMat;
#if 0
            if (device_poses_[idx].bDeviceIsConnected &&
                device_poses_[idx].eTrackingResult == vr::TrackingResult_Running_OK) {
                ///
            }
#endif
        } else {
            continue;
        }
        ////
        if(cls == 0) {
            // do nothing
            continue;
        }
        if(cls == vr::TrackedDeviceClass_HMD) {
            // update HMD pose
            HMD_coords = devicePoses[idx];
            continue;
        }
        if(cls == vr::TrackedDeviceClass_Controller) {
#if 0
            {
                std::string res;
                if(getDeviceString(res, idx, vr::Prop_RenderModelName_String)) {
                    *os_ << "model: " << idx << " / " << res << std::endl;
                }
            }
            {
                std::string res;
                if(getDeviceString(res, idx, vr::Prop_TrackingSystemName_String)) {
                    *os_ << "system: " << idx << " / " << res << std::endl;
                }
            }
            {
                std::string res;
                if(getDeviceString(res, idx, vr::Prop_SerialNumber_String)) {
                    *os_ << "serial: " << idx << " / " << res << std::endl;
                }
            }
#endif
            if (!TrackedDevicePoses[idx].bDeviceIsConnected ||
                !(TrackedDevicePoses[idx].eTrackingResult == vr::TrackingResult_Running_OK)) {
                //// default controller pose
                continue;
            }
            // update controller pose
            if (idx == 1) {
                state_L.coords = devicePoses[idx];
            } else if (idx == 2) {
                state_R.coords = devicePoses[idx];
            } else {
                /// more then 3 controller?
                continue;
            }
            // update controller state(button etc.)
            vr::VRControllerState_t state;
            m_pHMD->GetControllerState(idx, &state, sizeof(vr::VRControllerState_t));
            if (idx == 1) {
                setStateToStruct(state, state_L);
            } else if (idx == 2) {
                setStateToStruct(state, state_R);
            } else {
                /// more then 3 controller?
                continue;
            }
            continue;
        }
        if(cls == vr::TrackedDeviceClass_GenericTracker) {
            continue;
        }
        if(cls == vr::TrackedDeviceClass_TrackingReference) {
            continue;
        }
    }

    impl->updateControllerState(state_L, state_R);

    vr::VREvent_t event;
    while( m_pHMD->PollNextEvent( &event, sizeof( event ) ) ) {
        switch( event.eventType ) {
        case vr::VREvent_TrackedDeviceDeactivated:
            break;
        case vr::VREvent_TrackedDeviceUpdated:
            break;
        }
    }
}
void OpenVRPlugin::Impl::setToCoords(const vr::HmdMatrix34_t &hmd_mat, coordinates &cds)
{
    cds.rot << hmd_mat.m[0][0], hmd_mat.m[0][1], hmd_mat.m[0][2],
               hmd_mat.m[1][0], hmd_mat.m[1][1], hmd_mat.m[1][2],
               hmd_mat.m[2][0], hmd_mat.m[2][1], hmd_mat.m[2][2];
    cds.pos << hmd_mat.m[0][3], hmd_mat.m[1][3], hmd_mat.m[2][3];
}
#endif // _WIN32
//// <<<< Impl

////
OpenVRPlugin* OpenVRPlugin::instance()
{
    return instance_;
}

////
OpenVRPlugin::OpenVRPlugin()
    : Plugin("OpenVR")
{
    instance_ = this;
    impl = new Impl();
}
OpenVRPlugin::~OpenVRPlugin()
{
}

bool OpenVRPlugin::initialize()
{
    DEBUG_PRINT();

    // classes
    CameraControlJoyItem::initializeClass(this);

    App::sigExecutionStarted().connect( [this]() {
                                            impl->initialize();
                                        });
    return true;
}

bool OpenVRPlugin::finalize()
{
    DEBUG_PRINT();
    return true;
}

const char* OpenVRPlugin::description() const
{
    static std::string text =
        fmt::format("OpenVR Plugin Version {}\n", CNOID_FULL_VERSION_STRING) +
        "\n" +
        "Copyrigh (c) 2024 IRSL-tut Development Team.\n"
        "\n" +
        MITLicenseText() +
        "\n"  ;

    return text.c_str();
}

SignalProxy<bool(const controllerState &left, const controllerState &right)> OpenVRPlugin::sigUpdateControllerState()
{
    return impl->updateControllerState;
}
SignalProxy<void(coordinates &headOrigin)> OpenVRPlugin::sigRequestHeadOrigin()
{
    return impl->requestHeadOrigin;
}

CNOID_IMPLEMENT_PLUGIN_ENTRY(OpenVRPlugin);
