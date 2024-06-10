#include "CameraControlJoyItem.h"
#include "OpenVRPlugin.h"

#include <cnoid/RootItem>
#include <cnoid/ItemManager>

#include <cnoid/MainWindow>
#include <cnoid/SceneWidget>
#include <cnoid/SceneView>
#include <cnoid/SceneCameras>

#include <cnoid/PutPropertyFunction>
#include <cnoid/Archive>
#include <cnoid/EigenArchive>

#include "Coordinates.h"

//#define IRSL_DEBUG
#include "irsl_debug.h"

using namespace std;
using namespace cnoid;

//// TODO
/// option
// useOriginalCameraPosition
// renderCamera
// updateCameraPose
// cameraPosition (x, y)
// cameraHeiht z
// cameraDirection theta
/// API
// getCoords

namespace cnoid {

class CameraControlJoyItem::Impl
{
public:
    Impl(CameraControlJoyItem* _self);
    Impl(CameraControlJoyItem* _self, const Impl& org);
    ~Impl();

    CameraControlJoyItem* self;

    void itemSelected(bool on)
    {
    }

    void initialize();
    void keyRelease(QKeyEvent *ev);

    coordinates w_T_pos;
    coordinates pos_T_head;
    coordinates head_T_cambase;
    coordinates cambase_T_cam;

    void setCameraPosition(bool render);
    void initializeCameraPosition();

    double translate_unit;
    double rotate_unit;
    double up_down_unit;
    double pan_tilt_unit;

    bool setUseOriginalCameraPosition(bool _on)
    {
        useOriginalCameraPosition = _on;
        return true;
    }
    bool setRenderCamera(bool _on)
    {
        renderCamera = _on;
        return true;
    }
    bool setUpdateCameraPose(bool _on)
    {
        updateCameraPose = _on;
        return true;
    }

    //// property
    bool useOriginalCameraPosition;
    bool renderCamera;
    bool updateCameraPose;
    //
    //double cameraPosition_X;
    //double cameraPosition_Y;
    //double cameraHeight;
    Vector3 cameraPosition;
    double cameraDirection;

    ////
    void doPutProperties(PutPropertyFunction& putProperty);
    bool store(Archive& archive);
    bool restore(const Archive& archive);
};

}
//// Impl
CameraControlJoyItem::Impl::Impl(CameraControlJoyItem* _self)
    : self(_self)
{
    self->sigSelectionChanged().connect([this](bool on){ itemSelected(on); });
    initialize();
}

CameraControlJoyItem::Impl::Impl(CameraControlJoyItem* _self, const Impl& org)
    : self(_self)
{
    // copy from org
    initialize();
}

CameraControlJoyItem::Impl::~Impl()
{
}
void CameraControlJoyItem::Impl::initialize()
{
    DEBUG_PRINT();
    //// property
    useOriginalCameraPosition = false;
    renderCamera = false;
    updateCameraPose = false;

    cameraPosition.x() = -3.0;
    cameraPosition.y() = 0.0;
    cameraPosition.z() = 1.5;
    cameraDirection = 0.0;

    translate_unit = 0.15;
    rotate_unit  = 0.1;
    up_down_unit = 0.05;
    pan_tilt_unit = 0.07;

    MainWindow *mw = MainWindow::instance();

    mw->sigKeyReleased().connect( [this](QKeyEvent *ev) {
                                      this->keyRelease(ev);
                                  });

    OpenVRPlugin::instance()->sigRequestHeadOrigin().connect( [this] (coordinates &head_origin) {
        head_origin = w_T_pos;
        head_origin.transform(pos_T_head);
    });
}
void CameraControlJoyItem::Impl::initializeCameraPosition()
{
    //// load from original camera position
    w_T_pos.pos = Vector3(cameraPosition.x(), cameraPosition.y(), 0);
    w_T_pos.rotate(cameraDirection, Vector3::UnitZ());

    pos_T_head.pos = Vector3(0, 0, cameraPosition.z());
    Quaternion q(0.5, 0.5, -0.5, -0.5);//w, x, y, z
#if 0
    std::cerr << "x: " << q.x() << std::endl;
    std::cerr << "y: " << q.y() << std::endl;
    std::cerr << "z: " << q.z() << std::endl;
    std::cerr << "w: " << q.w() << std::endl;
#endif
    head_T_cambase.set(q);
    //cambase_T_cam

    if (updateCameraPose) {
        setCameraPosition(renderCamera);
    }
}
void CameraControlJoyItem::Impl::setCameraPosition(bool render)
{
    coordinates cds = w_T_pos;
    cds.transform(pos_T_head);
    cds.transform(head_T_cambase);
    cds.transform(cambase_T_cam);
    Isometry3 cur;
    cds.toPosition(cur);
    SceneView::instance()->sceneWidget()->builtinCameraTransform()->setPosition(cur);
    if (render) {
        SceneView::instance()->sceneWidget()->renderScene(true);
    }
}
void CameraControlJoyItem::Impl::keyRelease(QKeyEvent *ev)
{
    DEBUG_STREAM(" camp: " << useOriginalCameraPosition);
    DEBUG_STREAM(" rcam: " << renderCamera);
    DEBUG_STREAM(" ucam: " << updateCameraPose);
    bool shift = ev->modifiers() & Qt::ShiftModifier;
    double scl = 1.0;
    if (shift) {
        scl *= 3;
    }
    bool ctrl = ev->modifiers() & Qt::ControlModifier;
    //ev->modifiers() | Qt::AltModifier
    DEBUG_STREAM(" key(" << shift << ") : " << ev->key());
    switch(ev->key()) {
    case 81: // Q
        w_T_pos.rotate( rotate_unit*scl, Vector3::UnitZ());
        break;
    case 69: // E
        w_T_pos.rotate(-rotate_unit*scl, Vector3::UnitZ());
        break;
    ////
    case 87: // W
        w_T_pos.translate(Vector3::UnitX()* translate_unit*scl);
        break;
    case 83: // S
        w_T_pos.translate(Vector3::UnitX()*-translate_unit*scl);
        break;
    case 65: // A
        w_T_pos.translate(Vector3::UnitY()* translate_unit*scl);
        break;
    case 68: // D
        w_T_pos.translate(Vector3::UnitY()*-translate_unit*scl);
        break;
    ////
    case 74: // J
        cambase_T_cam.rotate( pan_tilt_unit*scl, Vector3::UnitX());
        break;
    case 75: // K
        cambase_T_cam.rotate(-pan_tilt_unit*scl, Vector3::UnitX());
        break;
    case 72: // H
        cambase_T_cam.rotate( pan_tilt_unit*scl, Vector3::UnitY());
        break;
    case 76: // L
        cambase_T_cam.rotate(-pan_tilt_unit*scl, Vector3::UnitY());
        break;
    ////
    case 85: // U
        head_T_cambase.translate(Vector3::UnitZ()* up_down_unit*scl);
        break;
    case 73: // I
        head_T_cambase.translate(Vector3::UnitZ()*-up_down_unit*scl);
        break;
    ////
    case 78: // N
        if (ctrl) {
            cambase_T_cam = coordinates();
        }
        break;
    }
    if (updateCameraPose) {
        setCameraPosition(renderCamera);
    }
}
void CameraControlJoyItem::Impl::doPutProperties(PutPropertyFunction& putProperty)
{
    putProperty("useOriginalCameraPosition", useOriginalCameraPosition,
                [&](bool on){ return setUseOriginalCameraPosition(on); });
    putProperty("renderCamera", renderCamera,
                [&](bool on){ return setRenderCamera(on); });
    putProperty("updateCameraPose", updateCameraPose,
                [&](bool on){ return setUpdateCameraPose(on); });
    //// why here
    initializeCameraPosition();
}
bool CameraControlJoyItem::Impl::store(Archive& archive)
{
    archive.writeFileInformation(self);
    archive.write("useOriginalCameraPosition", useOriginalCameraPosition);
    archive.write("renderCamera", renderCamera);
    archive.write("updateCameraPose", updateCameraPose);
    write(archive, "cameraPosition", cameraPosition);
    archive.write("cameraDirection", cameraDirection);
    return true;
}
bool CameraControlJoyItem::Impl::restore(const Archive& archive)
{
    bool on;
    if (archive.read("useOriginalCameraPosition", on)) {
        setUseOriginalCameraPosition(on);
    }
    if (archive.read("renderCamera", on)) {
        setRenderCamera(on);
    }
    if (archive.read("updateCameraPose", on)) {
        setUpdateCameraPose(on);
    }
    Vector3 p = Vector3::Zero();
    if (read(archive, "cameraPosition", p)) {
        cameraPosition = p;
    }
    double dbl;
    if (archive.read("cameraDirection", dbl)) {
        cameraDirection = dbl;
    }

    return true;
}
////
CameraControlJoyItem::CameraControlJoyItem()
{
    impl = new Impl(this);
}
CameraControlJoyItem::CameraControlJoyItem(const std::string &name)
    : Item(name)
{
    impl = new Impl(this);
}
CameraControlJoyItem::CameraControlJoyItem(const CameraControlJoyItem& org)
    : Item(org)
{
    impl = new Impl(this, *org.impl);
    setChecked(org.isChecked());
}

CameraControlJoyItem::~CameraControlJoyItem()
{
    DEBUG_PRINT();
    delete impl;
}
////
void CameraControlJoyItem::initializeClass(ExtensionManager* ext)
{
    DEBUG_PRINT();
    ItemManager* im = &ext->itemManager();
    im->registerClass<CameraControlJoyItem>("CameraControlJoyItem")
        .addCreationPanel<CameraControlJoyItem>();
}
//// protected / override Item Class
Item* CameraControlJoyItem::doDuplicate() const
{
    DEBUG_PRINT();
    return new CameraControlJoyItem(*this);
}
bool CameraControlJoyItem::doAssign(const Item* srcItem)
{
    DEBUG_PRINT();
    return false;
    //??
    //return impl->doAssign(srcItem);
}
void CameraControlJoyItem::onTreePathChanged()
{
    DEBUG_PRINT();
}
void CameraControlJoyItem::onConnectedToRoot()
{
    DEBUG_PRINT();
}
void CameraControlJoyItem::doPutProperties(PutPropertyFunction& putProperty)
{
    DEBUG_PRINT();
    impl->doPutProperties(putProperty);
}
bool CameraControlJoyItem::store(Archive& archive)
{
    DEBUG_PRINT();
    return impl->store(archive);
}
bool CameraControlJoyItem::restore(const Archive& archive)
{
    DEBUG_PRINT();
    return impl->restore(archive);
}
//// protected
