#ifndef CNOID_CAMERA_CONTROL_JOY_ITEM_H
#define CNOID_CAMERA_CONTROL_JOY_ITEM_H

#include <cnoid/Item>

namespace cnoid {

class ItemManager;

class CameraControlJoyItem : public Item
{
public:
    static void initializeClass(ExtensionManager* ext);

    CameraControlJoyItem();
    CameraControlJoyItem(const std::string& name);
    CameraControlJoyItem(const CameraControlJoyItem& org);
    virtual ~CameraControlJoyItem();

    virtual bool store(Archive& archive) override;
    virtual bool restore(const Archive& archive) override;
protected:
    virtual Item* doDuplicate() const override;
    virtual bool doAssign(const Item* item) override;
    virtual void onTreePathChanged() override;
    virtual void onConnectedToRoot() override;
    virtual void doPutProperties(PutPropertyFunction& putProperty) override;

private:
    class Impl;
    Impl* impl;
};

}

#endif
