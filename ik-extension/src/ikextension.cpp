// myextension.cpp
// Extension lib defines
#define LIB_NAME "ikExtension"
#define MODULE_NAME "ikext"

// include the Defold SDK
#include <dmsdk/sdk.h>

static dmTransform::Transform  transforms[2];
static dmGameObject::HInstance root;
static bool instance_set = false;

extern void two_joint_ik(
    dmVMath::Vector3 hip_pos, dmVMath::Vector3 knee_pos, dmVMath::Vector3 heel_pos, dmVMath::Vector3 t, float eps,
    dmVMath::Quat hip_gr, dmVMath::Quat knee_gr, dmVMath::Quat &hip_lr, dmVMath::Quat &knee_lr);


static int IKTwoJoint(lua_State* L)
{
    // The number of expected items to be on the Lua stack
    // once this struct goes out of scope
    DM_LUA_STACK_CHECK(L, 0);

    dmGameObject::HInstance hip = dmScript::CheckGOInstance(L, 1);
    dmGameObject::HInstance knee = dmScript::CheckGOInstance(L, 2);
    dmGameObject::HInstance heel = dmScript::CheckGOInstance(L, 3);

    dmVMath::Vector3* tgt = dmScript::CheckVector3(L, 4);

    // Get the bone structure (maybe store in a map)
    // Check the instances are bones - just so normal nodes are not attempted to be modified.
    if(!dmGameObject::IsBone(hip) || !dmGameObject::IsBone(knee) || !dmGameObject::IsBone(heel))
    {
        printf("[IK Error] Instance provided is not a Bone.\n");
        return 0;
    }

    dmVMath::Vector3 pos_hip    = dmVMath::Vector3(dmGameObject::GetPosition(hip));
    dmVMath::Vector3 pos_knee   = dmVMath::Vector3(dmGameObject::GetPosition(knee));
    dmVMath::Vector3 pos_heel   = dmVMath::Vector3(dmGameObject::GetPosition(heel));

    dmVMath::Quat rot_hip       = dmGameObject::GetWorldRotation(hip);
    dmVMath::Quat rot_knee      = dmGameObject::GetWorldRotation(knee);
    
    dmVMath::Quat hip_lr        = dmGameObject::GetRotation(hip);
    dmVMath::Quat knee_lr       = dmGameObject::GetRotation(knee);
    two_joint_ik( pos_hip, pos_knee, pos_heel, *tgt, 0.001f, rot_hip, rot_knee, hip_lr, knee_lr);
    
    transforms[0] = dmTransform::Transform( pos_hip, hip_lr, dmVMath::Vector3(1, 1, 1));
    transforms[1] = dmTransform::Transform( pos_knee, knee_lr, dmVMath::Vector3(1, 1, 1));
    root = hip;
    instance_set = true;
    return 0;
}

void applyIKChanges()
{    
    if(instance_set)
        // Set the bone transforms for the two bones we care about.
        dmGameObject::SetBoneTransforms(root, transforms[0],transforms,2);
}

// Functions exposed to Lua
static const luaL_reg Module_methods[] =
{
    {"two_joint", IKTwoJoint},
    {0, 0}
};

static void LuaInit(lua_State* L)
{
    int top = lua_gettop(L);

    // Register lua names
    luaL_register(L, MODULE_NAME, Module_methods);

    lua_pop(L, 1);
    assert(top == lua_gettop(L));
}

static dmExtension::Result AppInitializeIKExtension(dmExtension::AppParams* params)
{
    dmLogInfo("AppInitializeIKExtension");
    return dmExtension::RESULT_OK;
}

static dmExtension::Result InitializeIKExtension(dmExtension::Params* params)
{
    // Init Lua
    LuaInit(params->m_L);
    dmLogInfo("Registered %s Extension", MODULE_NAME);
    return dmExtension::RESULT_OK;
}

static dmExtension::Result AppFinalizeIKExtension(dmExtension::AppParams* params)
{
    dmLogInfo("AppFinalizeIKExtension");
    return dmExtension::RESULT_OK;
}

static dmExtension::Result FinalizeIKExtension(dmExtension::Params* params)
{
    dmLogInfo("FinalizeIKExtension");
    return dmExtension::RESULT_OK;
}

static dmExtension::Result OnUpdateIKExtension(dmExtension::Params* params)
{
    //dmLogInfo("OnUpdateIKExtension");
    applyIKChanges();
    return dmExtension::RESULT_OK;
}

static void OnEventIKExtension(dmExtension::Params* params, const dmExtension::Event* event)
{
    switch(event->m_Event)
    {
        case dmExtension::EVENT_ID_ACTIVATEAPP:
            dmLogInfo("OnEventIKExtension - EVENT_ID_ACTIVATEAPP");
            break;
        case dmExtension::EVENT_ID_DEACTIVATEAPP:
            dmLogInfo("OnEventIKExtension - EVENT_ID_DEACTIVATEAPP");
            break;
        case dmExtension::EVENT_ID_ICONIFYAPP:
            dmLogInfo("OnEventIKExtension - EVENT_ID_ICONIFYAPP");
            break;
        case dmExtension::EVENT_ID_DEICONIFYAPP:
            dmLogInfo("OnEventIKExtension - EVENT_ID_DEICONIFYAPP");
            break;
        default:
            dmLogWarning("OnEventIKExtension - Unknown event id");
            break;
    }
}

// Defold SDK uses a macro for setting up extension entry points:
//
// DM_DECLARE_EXTENSION(symbol, name, app_init, app_final, init, update, on_event, final)

// IKExtension is the C++ symbol that holds all relevant extension data.
// It must match the name field in the `ext.manifest`
DM_DECLARE_EXTENSION(IKExtension, LIB_NAME, AppInitializeIKExtension, AppFinalizeIKExtension, InitializeIKExtension, OnUpdateIKExtension, OnEventIKExtension, FinalizeIKExtension)
