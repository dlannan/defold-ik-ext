// myextension.cpp
// Extension lib defines
#define LIB_NAME "ikExtension"
#define MODULE_NAME "ikext"

// include the Defold SDK
#include <dmsdk/sdk.h>

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

    static dmTransform::Transform  transforms[2];
    // dmTransform::Transform component_transform = dmGameObject::GetWorldTransform(hip);

    // pos_knee = dmVMath::Rotate(hip_lr, pos_knee);
    // pos_heel = dmVMath::Rotate(knee_lr, pos_heel);

    printf("pos_hip %g %g %g\n", pos_hip.getX(), pos_hip.getY(), pos_hip.getZ());
    printf("pos_knee %g %g %g\n", pos_knee.getX(), pos_knee.getY(), pos_knee.getZ());
    
//    printf("hip_lr %g %g %g %g\n", hip_lr.getX(), hip_lr.getY(), hip_lr.getZ(), hip_lr.getW());
//    printf("knee_lr %g %g %g %g\n", knee_lr.getX(), knee_lr.getY(), knee_lr.getZ(), knee_lr.getW());

    transforms[0] = dmTransform::Transform( pos_hip, hip_lr, dmGameObject::GetScale(hip) );
    transforms[1] = dmTransform::Transform( pos_knee, knee_lr, dmGameObject::GetScale(knee) );
    transforms[2] = dmTransform::Transform( pos_heel, dmGameObject::GetRotation(heel), dmGameObject::GetScale(heel) );

    dmTransform::Transform t[2];
    t[0].SetIdentity();
    t[1].SetIdentity();
    t[1].SetTranslation(*tgt);

    dmTransform::Transform component_transform;
    component_transform.SetIdentity();    
    
    dmGameObject::SetBoneTransforms(hip, component_transform, t, 2);
    return 0;
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
