using System.Collections;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using System;



public partial class NativeConstants
{

    /// PHYSICS_CLIENT_C_API_H ->
    /// Error generating expression: Value cannot be null.
    ///Parameter name: node
    public const string PHYSICS_CLIENT_C_API_H = "";

    /// SHARED_MEMORY_PUBLIC_H ->
    /// Error generating expression: Value cannot be null.
    ///Parameter name: node
    public const string SHARED_MEMORY_PUBLIC_H = "";

    /// SHARED_MEMORY_KEY -> 12347
    public const int SHARED_MEMORY_KEY = 12347;

    public const int SHARED_MEMORY_KEY2 = 12348;

    /// SHARED_MEMORY_MAGIC_NUMBER -> 201801170
    public const int SHARED_MEMORY_MAGIC_NUMBER = 202002030;

    /// MAX_VR_ANALOG_AXIS -> 5
    public const int MAX_VR_ANALOG_AXIS = 5;

    /// MAX_VR_BUTTONS -> 64
    public const int MAX_VR_BUTTONS = 64;

    /// MAX_VR_CONTROLLERS -> 8
    public const int MAX_VR_CONTROLLERS = 8;

    /// MAX_RAY_INTERSECTION_BATCH_SIZE -> 256
    public const int MAX_RAY_INTERSECTION_BATCH_SIZE = 256;

    /// MAX_RAY_HITS -> MAX_RAY_INTERSECTION_BATCH_SIZE
    public const int MAX_RAY_HITS = NativeConstants.MAX_RAY_INTERSECTION_BATCH_SIZE;

    /// MAX_KEYBOARD_EVENTS -> 256
    public const int MAX_KEYBOARD_EVENTS = 256;

    /// MAX_MOUSE_EVENTS -> 256
    public const int MAX_MOUSE_EVENTS = 256;

    /// MAX_SDF_BODIES -> 512
    public const int MAX_SDF_BODIES = 512;

    /// VISUAL_SHAPE_MAX_PATH_LEN -> 1024
    public const int VISUAL_SHAPE_MAX_PATH_LEN = 1024;

    /// B3_MAX_PLUGIN_ARG_SIZE -> 128
    public const int B3_MAX_PLUGIN_ARG_SIZE = 128;

    /// B3_MAX_PLUGIN_ARG_TEXT_LEN -> 1024
    public const int B3_MAX_PLUGIN_ARG_TEXT_LEN = 1024;

    /// Warning: Generation of Method Macros is not supported at this time
    /// B3_DECLARE_HANDLE -> "(name) typedef struct name##__ { int unused; } *name"
    public const string B3_DECLARE_HANDLE = "(name) typedef struct name##__ { int unused; } *name";

    /// B3_SHARED_API -> __declspec(dllexport)
    /// Error generating expression: Error generating function call.  Operation not implemented
    public const string B3_SHARED_API = "__declspec(dllexport)";

    /// PHYSICS_CLIENT_SHARED_MEMORY_H ->
    /// Error generating expression: Value cannot be null.
    ///Parameter name: node
    public const string PHYSICS_CLIENT_SHARED_MEMORY_H = "";

    /// PHYSICS_CLIENT_SHARED_MEMORY2_H ->
    /// Error generating expression: Value cannot be null.
    ///Parameter name: node
    public const string PHYSICS_CLIENT_SHARED_MEMORY2_H = "";

    /// PHYSICS_DIRECT_C_API_H ->
    /// Error generating expression: Value cannot be null.
    ///Parameter name: node
    public const string PHYSICS_DIRECT_C_API_H = "";

    /// IN_PROCESS_PHYSICS_C_API_H ->
    /// Error generating expression: Value cannot be null.
    ///Parameter name: node
    public const string IN_PROCESS_PHYSICS_C_API_H = "";
}

// Error: Error processing procedure b3ConfigureOpenGLVisualizerSetLightPosition: Expected token of type BracketClose but found Number instead.
// Error: Error processing procedure b3InitCreateSoftBodyAnchorConstraintCommand: Expected token of type BracketClose but found Number instead.

public enum EnumSharedMemoryClientCommand
{

    /// CMD_INVALID -> 0
    CMD_INVALID = 0,

    CMD_LOAD_SDF,

    CMD_LOAD_URDF,

    CMD_LOAD_BULLET,

    CMD_SAVE_BULLET,

    CMD_LOAD_MJCF,

    CMD_LOAD_SOFT_BODY,

    CMD_SEND_BULLET_DATA_STREAM,

    CMD_CREATE_BOX_COLLISION_SHAPE,

    CMD_CREATE_RIGID_BODY,

    CMD_DELETE_RIGID_BODY,

    CMD_CREATE_SENSOR,

    CMD_INIT_POSE,

    CMD_SEND_PHYSICS_SIMULATION_PARAMETERS,

    CMD_SEND_DESIRED_STATE,

    CMD_REQUEST_ACTUAL_STATE,

    CMD_REQUEST_DEBUG_LINES,

    CMD_REQUEST_BODY_INFO,

    CMD_REQUEST_INTERNAL_DATA,

    CMD_STEP_FORWARD_SIMULATION,

    CMD_RESET_SIMULATION,

    CMD_PICK_BODY,

    CMD_MOVE_PICKED_BODY,

    CMD_REMOVE_PICKING_CONSTRAINT_BODY,

    CMD_REQUEST_CAMERA_IMAGE_DATA,

    CMD_APPLY_EXTERNAL_FORCE,

    CMD_CALCULATE_INVERSE_DYNAMICS,

    CMD_CALCULATE_INVERSE_KINEMATICS,

    CMD_CALCULATE_JACOBIAN,

    CMD_CALCULATE_MASS_MATRIX,

    CMD_USER_CONSTRAINT,

    CMD_REQUEST_CONTACT_POINT_INFORMATION,

    CMD_REQUEST_RAY_CAST_INTERSECTIONS,

    CMD_REQUEST_AABB_OVERLAP,

    CMD_SAVE_WORLD,

    CMD_REQUEST_VISUAL_SHAPE_INFO,

    CMD_UPDATE_VISUAL_SHAPE,

    CMD_LOAD_TEXTURE,

    CMD_SET_SHADOW,

    CMD_USER_DEBUG_DRAW,

    CMD_REQUEST_VR_EVENTS_DATA,

    CMD_SET_VR_CAMERA_STATE,

    CMD_SYNC_BODY_INFO,

    CMD_STATE_LOGGING,

    CMD_CONFIGURE_OPENGL_VISUALIZER,

    CMD_REQUEST_KEYBOARD_EVENTS_DATA,

    CMD_REQUEST_OPENGL_VISUALIZER_CAMERA,

    CMD_REMOVE_BODY,

    CMD_CHANGE_DYNAMICS_INFO,

    CMD_GET_DYNAMICS_INFO,

    CMD_PROFILE_TIMING,

    CMD_CREATE_COLLISION_SHAPE,

    CMD_CREATE_VISUAL_SHAPE,

    CMD_CREATE_MULTI_BODY,

    CMD_REQUEST_COLLISION_INFO,

    CMD_REQUEST_MOUSE_EVENTS_DATA,

    CMD_CHANGE_TEXTURE,

    CMD_SET_ADDITIONAL_SEARCH_PATH,

    CMD_CUSTOM_COMMAND,

    CMD_REQUEST_PHYSICS_SIMULATION_PARAMETERS,

    CMD_SAVE_STATE,

    CMD_RESTORE_STATE,

    CMD_REMOVE_STATE,

    CMD_REQUEST_COLLISION_SHAPE_INFO,

    CMD_SYNC_USER_DATA,

    CMD_REQUEST_USER_DATA,

    CMD_ADD_USER_DATA,

    CMD_REMOVE_USER_DATA,

    CMD_COLLISION_FILTER,

    CMD_REQUEST_MESH_DATA,

    CMD_MAX_CLIENT_COMMANDS,
}

public enum EnumSharedMemoryServerStatus
{

    /// CMD_SHARED_MEMORY_NOT_INITIALIZED -> 0
    CMD_SHARED_MEMORY_NOT_INITIALIZED = 0,

    CMD_WAITING_FOR_CLIENT_COMMAND,

    CMD_CLIENT_COMMAND_COMPLETED,

    CMD_UNKNOWN_COMMAND_FLUSHED,

    CMD_SDF_LOADING_COMPLETED,

    CMD_SDF_LOADING_FAILED,

    CMD_URDF_LOADING_COMPLETED,

    CMD_URDF_LOADING_FAILED,

    CMD_BULLET_LOADING_COMPLETED,

    CMD_BULLET_LOADING_FAILED,

    CMD_BULLET_SAVING_COMPLETED,

    CMD_BULLET_SAVING_FAILED,

    CMD_MJCF_LOADING_COMPLETED,

    CMD_MJCF_LOADING_FAILED,

    CMD_REQUEST_INTERNAL_DATA_COMPLETED,

    CMD_REQUEST_INTERNAL_DATA_FAILED,

    CMD_BULLET_DATA_STREAM_RECEIVED_COMPLETED,

    CMD_BULLET_DATA_STREAM_RECEIVED_FAILED,

    CMD_BOX_COLLISION_SHAPE_CREATION_COMPLETED,

    CMD_RIGID_BODY_CREATION_COMPLETED,

    CMD_SET_JOINT_FEEDBACK_COMPLETED,

    CMD_ACTUAL_STATE_UPDATE_COMPLETED,

    CMD_ACTUAL_STATE_UPDATE_FAILED,

    CMD_DEBUG_LINES_COMPLETED,

    CMD_DEBUG_LINES_OVERFLOW_FAILED,

    CMD_DESIRED_STATE_RECEIVED_COMPLETED,

    CMD_STEP_FORWARD_SIMULATION_COMPLETED,

    CMD_RESET_SIMULATION_COMPLETED,

    CMD_CAMERA_IMAGE_COMPLETED,

    CMD_CAMERA_IMAGE_FAILED,

    CMD_BODY_INFO_COMPLETED,

    CMD_BODY_INFO_FAILED,

    CMD_INVALID_STATUS,

    CMD_CALCULATED_INVERSE_DYNAMICS_COMPLETED,

    CMD_CALCULATED_INVERSE_DYNAMICS_FAILED,

    CMD_CALCULATED_JACOBIAN_COMPLETED,

    CMD_CALCULATED_JACOBIAN_FAILED,

    CMD_CALCULATED_MASS_MATRIX_COMPLETED,

    CMD_CALCULATED_MASS_MATRIX_FAILED,

    CMD_CONTACT_POINT_INFORMATION_COMPLETED,

    CMD_CONTACT_POINT_INFORMATION_FAILED,

    CMD_REQUEST_AABB_OVERLAP_COMPLETED,

    CMD_REQUEST_AABB_OVERLAP_FAILED,

    CMD_CALCULATE_INVERSE_KINEMATICS_COMPLETED,

    CMD_CALCULATE_INVERSE_KINEMATICS_FAILED,

    CMD_SAVE_WORLD_COMPLETED,

    CMD_SAVE_WORLD_FAILED,

    CMD_VISUAL_SHAPE_INFO_COMPLETED,

    CMD_VISUAL_SHAPE_INFO_FAILED,

    CMD_VISUAL_SHAPE_UPDATE_COMPLETED,

    CMD_VISUAL_SHAPE_UPDATE_FAILED,

    CMD_LOAD_TEXTURE_COMPLETED,

    CMD_LOAD_TEXTURE_FAILED,

    CMD_USER_DEBUG_DRAW_COMPLETED,

    CMD_USER_DEBUG_DRAW_PARAMETER_COMPLETED,

    CMD_USER_DEBUG_DRAW_FAILED,

    CMD_USER_CONSTRAINT_COMPLETED,

    CMD_USER_CONSTRAINT_INFO_COMPLETED,

    CMD_USER_CONSTRAINT_REQUEST_STATE_COMPLETED,

    CMD_REMOVE_USER_CONSTRAINT_COMPLETED,

    CMD_CHANGE_USER_CONSTRAINT_COMPLETED,

    CMD_REMOVE_USER_CONSTRAINT_FAILED,

    CMD_CHANGE_USER_CONSTRAINT_FAILED,

    CMD_USER_CONSTRAINT_FAILED,

    CMD_REQUEST_VR_EVENTS_DATA_COMPLETED,

    CMD_REQUEST_RAY_CAST_INTERSECTIONS_COMPLETED,

    CMD_SYNC_BODY_INFO_COMPLETED,

    CMD_SYNC_BODY_INFO_FAILED,

    CMD_STATE_LOGGING_COMPLETED,

    CMD_STATE_LOGGING_START_COMPLETED,

    CMD_STATE_LOGGING_FAILED,

    CMD_REQUEST_KEYBOARD_EVENTS_DATA_COMPLETED,

    CMD_REQUEST_KEYBOARD_EVENTS_DATA_FAILED,

    CMD_REQUEST_OPENGL_VISUALIZER_CAMERA_FAILED,

    CMD_REQUEST_OPENGL_VISUALIZER_CAMERA_COMPLETED,

    CMD_REMOVE_BODY_COMPLETED,

    CMD_REMOVE_BODY_FAILED,

    CMD_GET_DYNAMICS_INFO_COMPLETED,

    CMD_GET_DYNAMICS_INFO_FAILED,

    CMD_CREATE_COLLISION_SHAPE_FAILED,

    CMD_CREATE_COLLISION_SHAPE_COMPLETED,

    CMD_CREATE_VISUAL_SHAPE_FAILED,

    CMD_CREATE_VISUAL_SHAPE_COMPLETED,

    CMD_CREATE_MULTI_BODY_FAILED,

    CMD_CREATE_MULTI_BODY_COMPLETED,

    CMD_REQUEST_COLLISION_INFO_COMPLETED,

    CMD_REQUEST_COLLISION_INFO_FAILED,

    CMD_REQUEST_MOUSE_EVENTS_DATA_COMPLETED,

    CMD_CHANGE_TEXTURE_COMMAND_FAILED,

    CMD_CUSTOM_COMMAND_COMPLETED,

    CMD_CUSTOM_COMMAND_FAILED,

    CMD_REQUEST_PHYSICS_SIMULATION_PARAMETERS_COMPLETED,

    CMD_SAVE_STATE_FAILED,

    CMD_SAVE_STATE_COMPLETED,

    CMD_RESTORE_STATE_FAILED,

    CMD_RESTORE_STATE_COMPLETED,

    CMD_COLLISION_SHAPE_INFO_COMPLETED,

    CMD_COLLISION_SHAPE_INFO_FAILED,

    CMD_LOAD_SOFT_BODY_FAILED,

    CMD_LOAD_SOFT_BODY_COMPLETED,

    CMD_SYNC_USER_DATA_COMPLETED,

    CMD_SYNC_USER_DATA_FAILED,

    CMD_REQUEST_USER_DATA_COMPLETED,

    CMD_REQUEST_USER_DATA_FAILED,

    CMD_ADD_USER_DATA_COMPLETED,

    CMD_ADD_USER_DATA_FAILED,

    CMD_REMOVE_USER_DATA_COMPLETED,

    CMD_REMOVE_USER_DATA_FAILED,

    CMD_REMOVE_STATE_COMPLETED,

    CMD_REMOVE_STATE_FAILED,

    CMD_REQUEST_MESH_DATA_COMPLETED,

    CMD_REQUEST_MESH_DATA_FAILED,

    CMD_MAX_SERVER_COMMANDS,
}

public enum JointInfoFlags
{

    /// JOINT_HAS_MOTORIZED_POWER -> 1
    JOINT_HAS_MOTORIZED_POWER = 1,
}

public enum Anonymous_5aa26107_18ad_4adf_a929_af60854f356e
{

    /// COLLISION_SHAPE_TYPE_BOX -> 1
    COLLISION_SHAPE_TYPE_BOX = 1,

    COLLISION_SHAPE_TYPE_CYLINDER_X,

    COLLISION_SHAPE_TYPE_CYLINDER_Y,

    COLLISION_SHAPE_TYPE_CYLINDER_Z,

    COLLISION_SHAPE_TYPE_CAPSULE_X,

    COLLISION_SHAPE_TYPE_CAPSULE_Y,

    COLLISION_SHAPE_TYPE_CAPSULE_Z,

    COLLISION_SHAPE_TYPE_SPHERE,
}

public enum JointType
{

    /// eRevoluteType -> 0
    eRevoluteType = 0,

    /// ePrismaticType -> 1
    ePrismaticType = 1,

    /// eSphericalType -> 2
    eSphericalType = 2,

    /// ePlanarType -> 3
    ePlanarType = 3,

    /// eFixedType -> 4
    eFixedType = 4,

    /// ePoint2PointType -> 5
    ePoint2PointType = 5,

    /// eGearType -> 6
    eGearType = 6,
}

public enum b3JointInfoFlags
{

    /// eJointChangeMaxForce -> 1
    eJointChangeMaxForce = 1,

    /// eJointChangeChildFramePosition -> 2
    eJointChangeChildFramePosition = 2,

    /// eJointChangeChildFrameOrientation -> 4
    eJointChangeChildFrameOrientation = 4,
}

[System.Runtime.InteropServices.StructLayoutAttribute(System.Runtime.InteropServices.LayoutKind.Sequential, CharSet = System.Runtime.InteropServices.CharSet.Ansi)]
public struct b3JointInfo
{

    /// char[1024]
    [System.Runtime.InteropServices.MarshalAsAttribute(System.Runtime.InteropServices.UnmanagedType.ByValTStr, SizeConst = 1024)]
    public string m_linkName;

    /// char[1024]
    [System.Runtime.InteropServices.MarshalAsAttribute(System.Runtime.InteropServices.UnmanagedType.ByValTStr, SizeConst = 1024)]
    public string m_jointName;

    /// int
    public int m_jointType;

    /// int
    public int m_qIndex;

    /// int
    public int m_uIndex;

    /// int
    public int m_jointIndex;

    /// int
    public int m_flags;

    /// double
    public double m_jointDamping;

    /// double
    public double m_jointFriction;

    /// double
    public double m_jointLowerLimit;

    /// double
    public double m_jointUpperLimit;

    /// double
    public double m_jointMaxForce;

    /// double
    public double m_jointMaxVelocity;

    /// double[7]
    [System.Runtime.InteropServices.MarshalAsAttribute(System.Runtime.InteropServices.UnmanagedType.ByValArray, SizeConst = 7, ArraySubType = System.Runtime.InteropServices.UnmanagedType.R8)]
    public double[] m_parentFrame;

    /// double[7]
    [System.Runtime.InteropServices.MarshalAsAttribute(System.Runtime.InteropServices.UnmanagedType.ByValArray, SizeConst = 7, ArraySubType = System.Runtime.InteropServices.UnmanagedType.R8)]
    public double[] m_childFrame;

    /// double[3]
    [System.Runtime.InteropServices.MarshalAsAttribute(System.Runtime.InteropServices.UnmanagedType.ByValArray, SizeConst = 3, ArraySubType = System.Runtime.InteropServices.UnmanagedType.R8)]
    public double[] m_jointAxis;

    /// int
    public int m_parentIndex;

    /// int
    public int m_qSize;

    /// int
    public int m_uSize;
}

public enum UserDataValueType
{

    /// USER_DATA_VALUE_TYPE_BYTES -> 0
    USER_DATA_VALUE_TYPE_BYTES = 0,

    /// USER_DATA_VALUE_TYPE_STRING -> 1
    USER_DATA_VALUE_TYPE_STRING = 1,
}

[System.Runtime.InteropServices.StructLayoutAttribute(System.Runtime.InteropServices.LayoutKind.Sequential)]
public struct b3UserDataValue
{

    /// int
    public int m_type;

    /// int
    public int m_length;

    /// char*
    [System.Runtime.InteropServices.MarshalAsAttribute(System.Runtime.InteropServices.UnmanagedType.LPStr)]
    public string m_data1;
}

public enum EnumUserConstraintFlags
{

    /// USER_CONSTRAINT_ADD_CONSTRAINT -> 1
    USER_CONSTRAINT_ADD_CONSTRAINT = 1,

    /// USER_CONSTRAINT_REMOVE_CONSTRAINT -> 2
    USER_CONSTRAINT_REMOVE_CONSTRAINT = 2,

    /// USER_CONSTRAINT_CHANGE_CONSTRAINT -> 4
    USER_CONSTRAINT_CHANGE_CONSTRAINT = 4,

    /// USER_CONSTRAINT_CHANGE_PIVOT_IN_B -> 8
    USER_CONSTRAINT_CHANGE_PIVOT_IN_B = 8,

    /// USER_CONSTRAINT_CHANGE_FRAME_ORN_IN_B -> 16
    USER_CONSTRAINT_CHANGE_FRAME_ORN_IN_B = 16,

    /// USER_CONSTRAINT_CHANGE_MAX_FORCE -> 32
    USER_CONSTRAINT_CHANGE_MAX_FORCE = 32,

    /// USER_CONSTRAINT_REQUEST_INFO -> 64
    USER_CONSTRAINT_REQUEST_INFO = 64,

    /// USER_CONSTRAINT_CHANGE_GEAR_RATIO -> 128
    USER_CONSTRAINT_CHANGE_GEAR_RATIO = 128,

    /// USER_CONSTRAINT_CHANGE_GEAR_AUX_LINK -> 256
    USER_CONSTRAINT_CHANGE_GEAR_AUX_LINK = 256,

    /// USER_CONSTRAINT_CHANGE_RELATIVE_POSITION_TARGET -> 512
    USER_CONSTRAINT_CHANGE_RELATIVE_POSITION_TARGET = 512,

    /// USER_CONSTRAINT_CHANGE_ERP -> 1024
    USER_CONSTRAINT_CHANGE_ERP = 1024,

    /// USER_CONSTRAINT_REQUEST_STATE -> 2048
    USER_CONSTRAINT_REQUEST_STATE = 2048,

    /// USER_CONSTRAINT_ADD_SOFT_BODY_ANCHOR -> 4096
    USER_CONSTRAINT_ADD_SOFT_BODY_ANCHOR = 4096,
}

[System.Runtime.InteropServices.StructLayoutAttribute(System.Runtime.InteropServices.LayoutKind.Sequential)]
public struct b3UserConstraint
{

    /// int
    public int m_parentBodyIndex;

    /// int
    public int m_parentJointIndex;

    /// int
    public int m_childBodyIndex;

    /// int
    public int m_childJointIndex;

    /// double[7]
    [System.Runtime.InteropServices.MarshalAsAttribute(System.Runtime.InteropServices.UnmanagedType.ByValArray, SizeConst = 7, ArraySubType = System.Runtime.InteropServices.UnmanagedType.R8)]
    public double[] m_parentFrame;

    /// double[7]
    [System.Runtime.InteropServices.MarshalAsAttribute(System.Runtime.InteropServices.UnmanagedType.ByValArray, SizeConst = 7, ArraySubType = System.Runtime.InteropServices.UnmanagedType.R8)]
    public double[] m_childFrame;

    /// double[3]
    [System.Runtime.InteropServices.MarshalAsAttribute(System.Runtime.InteropServices.UnmanagedType.ByValArray, SizeConst = 3, ArraySubType = System.Runtime.InteropServices.UnmanagedType.R8)]
    public double[] m_jointAxis;

    /// int
    public int m_jointType;

    /// double
    public double m_maxAppliedForce;

    /// int
    public int m_userConstraintUniqueId;

    /// double
    public double m_gearRatio;

    /// int
    public int m_gearAuxLink;

    /// double
    public double m_relativePositionTarget;

    /// double
    public double m_erp;
}

[System.Runtime.InteropServices.StructLayoutAttribute(System.Runtime.InteropServices.LayoutKind.Sequential, CharSet = System.Runtime.InteropServices.CharSet.Ansi)]
public struct b3BodyInfo
{

    /// char[1024]
    [System.Runtime.InteropServices.MarshalAsAttribute(System.Runtime.InteropServices.UnmanagedType.ByValTStr, SizeConst = 1024)]
    public string m_baseName;

    /// char[1024]
    [System.Runtime.InteropServices.MarshalAsAttribute(System.Runtime.InteropServices.UnmanagedType.ByValTStr, SizeConst = 1024)]
    public string m_bodyName;
}

public enum DynamicsActivationState
{

    /// eActivationStateEnableSleeping -> 1
    eActivationStateEnableSleeping = 1,

    /// eActivationStateDisableSleeping -> 2
    eActivationStateDisableSleeping = 2,

    /// eActivationStateWakeUp -> 4
    eActivationStateWakeUp = 4,

    /// eActivationStateSleep -> 8
    eActivationStateSleep = 8,

    /// eActivationStateEnableWakeup -> 16
    eActivationStateEnableWakeup = 16,

    /// eActivationStateDisableWakeup -> 32
    eActivationStateDisableWakeup = 32,
}

public enum b3BodyType
{

    /// BT_RIGID_BODY -> 1
    BT_RIGID_BODY = 1,

    /// BT_MULTI_BODY -> 2
    BT_MULTI_BODY = 2,

    /// BT_SOFT_BODY -> 3
    BT_SOFT_BODY = 3,
}

[System.Runtime.InteropServices.StructLayoutAttribute(System.Runtime.InteropServices.LayoutKind.Sequential)]
public struct b3DynamicsInfo
{

    /// double
    public double m_mass;

    /// double[3]
    [System.Runtime.InteropServices.MarshalAsAttribute(System.Runtime.InteropServices.UnmanagedType.ByValArray, SizeConst = 3, ArraySubType = System.Runtime.InteropServices.UnmanagedType.R8)]
    public double[] m_localInertialDiagonal;

    /// double[7]
    [System.Runtime.InteropServices.MarshalAsAttribute(System.Runtime.InteropServices.UnmanagedType.ByValArray, SizeConst = 7, ArraySubType = System.Runtime.InteropServices.UnmanagedType.R8)]
    public double[] m_localInertialFrame;

    /// double
    public double m_lateralFrictionCoeff;

    /// double
    public double m_rollingFrictionCoeff;

    /// double
    public double m_spinningFrictionCoeff;

    /// double
    public double m_restitution;

    /// double
    public double m_contactStiffness;

    /// double
    public double m_contactDamping;

    /// int
    public int m_activationState;

    /// int
    public int m_bodyType;

    /// double
    public double m_angularDamping;

    /// double
    public double m_linearDamping;

    /// double
    public double m_ccdSweptSphereRadius;

    /// double
    public double m_contactProcessingThreshold;

    /// int
    public int m_frictionAnchor;

    /// double
    public double m_collisionMargin;
}

public enum SensorType
{

    /// eSensorForceTorqueType -> 1
    eSensorForceTorqueType = 1,
}

[System.Runtime.InteropServices.StructLayoutAttribute(System.Runtime.InteropServices.LayoutKind.Sequential)]
public struct b3JointSensorState
{

    /// double
    public double m_jointPosition;

    /// double
    public double m_jointVelocity;

    /// double[6]
    [System.Runtime.InteropServices.MarshalAsAttribute(System.Runtime.InteropServices.UnmanagedType.ByValArray, SizeConst = 6, ArraySubType = System.Runtime.InteropServices.UnmanagedType.R8)]
    public double[] m_jointForceTorque;

    /// double
    public double m_jointMotorTorque;
}

[System.Runtime.InteropServices.StructLayoutAttribute(System.Runtime.InteropServices.LayoutKind.Sequential)]
public struct b3JointSensorState2
{

    /// double[4]
    [System.Runtime.InteropServices.MarshalAsAttribute(System.Runtime.InteropServices.UnmanagedType.ByValArray, SizeConst = 4, ArraySubType = System.Runtime.InteropServices.UnmanagedType.R8)]
    public double[] m_jointPosition;

    /// double[3]
    [System.Runtime.InteropServices.MarshalAsAttribute(System.Runtime.InteropServices.UnmanagedType.ByValArray, SizeConst = 3, ArraySubType = System.Runtime.InteropServices.UnmanagedType.R8)]
    public double[] m_jointVelocity;

    /// double[6]
    [System.Runtime.InteropServices.MarshalAsAttribute(System.Runtime.InteropServices.UnmanagedType.ByValArray, SizeConst = 6, ArraySubType = System.Runtime.InteropServices.UnmanagedType.R8)]
    public double[] m_jointReactionForceTorque;

    /// double[3]
    [System.Runtime.InteropServices.MarshalAsAttribute(System.Runtime.InteropServices.UnmanagedType.ByValArray, SizeConst = 3, ArraySubType = System.Runtime.InteropServices.UnmanagedType.R8)]
    public double[] m_jointMotorTorqueMultiDof;

    /// int
    public int m_qDofSize;

    /// int
    public int m_uDofSize;
}

[System.Runtime.InteropServices.StructLayoutAttribute(System.Runtime.InteropServices.LayoutKind.Sequential)]
public struct b3DebugLines
{

    /// int
    public int m_numDebugLines;

    /// float*
    public System.IntPtr m_linesFrom;

    /// float*
    public System.IntPtr m_linesTo;

    /// float*
    public System.IntPtr m_linesColor;
}

[System.Runtime.InteropServices.StructLayoutAttribute(System.Runtime.InteropServices.LayoutKind.Sequential)]
public struct b3OverlappingObject
{

    /// int
    public int m_objectUniqueId;

    /// int
    public int m_linkIndex;
}

[System.Runtime.InteropServices.StructLayoutAttribute(System.Runtime.InteropServices.LayoutKind.Sequential)]
public struct b3AABBOverlapData
{

    /// int
    public int m_numOverlappingObjects;

    /// b3OverlappingObject*
    public System.IntPtr m_overlappingObjects;
}

[System.Runtime.InteropServices.StructLayoutAttribute(System.Runtime.InteropServices.LayoutKind.Sequential)]
public struct b3CameraImageData
{

    /// int
    public int m_pixelWidth;

    /// int
    public int m_pixelHeight;

    /// char*
    [System.Runtime.InteropServices.MarshalAsAttribute(System.Runtime.InteropServices.UnmanagedType.LPStr)]
    public string m_rgbColorData;

    /// float*
    public System.IntPtr m_depthValues;

    /// int*
    public System.IntPtr m_segmentationMaskValues;
}

[System.Runtime.InteropServices.StructLayoutAttribute(System.Runtime.InteropServices.LayoutKind.Sequential)]
public struct b3MeshVertex
{

    /// double
    public double x;

    /// double
    public double y;

    /// double
    public double z;

    /// double
    public double w;
}

public enum eMeshDataEnum
{

    /// B3_MESH_DATA_COLLISIONSHAPEINDEX -> 1
    B3_MESH_DATA_COLLISIONSHAPEINDEX = 1,
}

[System.Runtime.InteropServices.StructLayoutAttribute(System.Runtime.InteropServices.LayoutKind.Sequential)]
public struct b3MeshData
{

    /// int
    public int m_numVertices;

    /// b3MeshVertex*
    public System.IntPtr m_vertices;
}

[System.Runtime.InteropServices.StructLayoutAttribute(System.Runtime.InteropServices.LayoutKind.Sequential)]
public struct b3OpenGLVisualizerCameraInfo
{

    /// int
    public int m_width;

    /// int
    public int m_height;

    /// float[16]
    [System.Runtime.InteropServices.MarshalAsAttribute(System.Runtime.InteropServices.UnmanagedType.ByValArray, SizeConst = 16, ArraySubType = System.Runtime.InteropServices.UnmanagedType.R4)]
    public float[] m_viewMatrix;

    /// float[16]
    [System.Runtime.InteropServices.MarshalAsAttribute(System.Runtime.InteropServices.UnmanagedType.ByValArray, SizeConst = 16, ArraySubType = System.Runtime.InteropServices.UnmanagedType.R4)]
    public float[] m_projectionMatrix;

    /// float[3]
    [System.Runtime.InteropServices.MarshalAsAttribute(System.Runtime.InteropServices.UnmanagedType.ByValArray, SizeConst = 3, ArraySubType = System.Runtime.InteropServices.UnmanagedType.R4)]
    public float[] m_camUp;

    /// float[3]
    [System.Runtime.InteropServices.MarshalAsAttribute(System.Runtime.InteropServices.UnmanagedType.ByValArray, SizeConst = 3, ArraySubType = System.Runtime.InteropServices.UnmanagedType.R4)]
    public float[] m_camForward;

    /// float[3]
    [System.Runtime.InteropServices.MarshalAsAttribute(System.Runtime.InteropServices.UnmanagedType.ByValArray, SizeConst = 3, ArraySubType = System.Runtime.InteropServices.UnmanagedType.R4)]
    public float[] m_horizontal;

    /// float[3]
    [System.Runtime.InteropServices.MarshalAsAttribute(System.Runtime.InteropServices.UnmanagedType.ByValArray, SizeConst = 3, ArraySubType = System.Runtime.InteropServices.UnmanagedType.R4)]
    public float[] m_vertical;

    /// float
    public float m_yaw;

    /// float
    public float m_pitch;

    /// float
    public float m_dist;

    /// float[3]
    [System.Runtime.InteropServices.MarshalAsAttribute(System.Runtime.InteropServices.UnmanagedType.ByValArray, SizeConst = 3, ArraySubType = System.Runtime.InteropServices.UnmanagedType.R4)]
    public float[] m_target;
}

[System.Runtime.InteropServices.StructLayoutAttribute(System.Runtime.InteropServices.LayoutKind.Sequential)]
public struct b3UserConstraintState
{

    /// double[6]
    [System.Runtime.InteropServices.MarshalAsAttribute(System.Runtime.InteropServices.UnmanagedType.ByValArray, SizeConst = 6, ArraySubType = System.Runtime.InteropServices.UnmanagedType.R8)]
    public double[] m_appliedConstraintForces;

    /// int
    public int m_numDofs;
}

public enum b3VREventType
{

    /// VR_CONTROLLER_MOVE_EVENT -> 1
    VR_CONTROLLER_MOVE_EVENT = 1,

    /// VR_CONTROLLER_BUTTON_EVENT -> 2
    VR_CONTROLLER_BUTTON_EVENT = 2,

    /// VR_HMD_MOVE_EVENT -> 4
    VR_HMD_MOVE_EVENT = 4,

    /// VR_GENERIC_TRACKER_MOVE_EVENT -> 8
    VR_GENERIC_TRACKER_MOVE_EVENT = 8,
}

public enum b3VRButtonInfo
{

    /// eButtonIsDown -> 1
    eButtonIsDown = 1,

    /// eButtonTriggered -> 2
    eButtonTriggered = 2,

    /// eButtonReleased -> 4
    eButtonReleased = 4,
}

public enum eVRDeviceTypeEnums
{

    /// VR_DEVICE_CONTROLLER -> 1
    VR_DEVICE_CONTROLLER = 1,

    /// VR_DEVICE_HMD -> 2
    VR_DEVICE_HMD = 2,

    /// VR_DEVICE_GENERIC_TRACKER -> 4
    VR_DEVICE_GENERIC_TRACKER = 4,
}

public enum EVRCameraFlags
{

    /// VR_CAMERA_TRACK_OBJECT_ORIENTATION -> 1
    VR_CAMERA_TRACK_OBJECT_ORIENTATION = 1,
}

[System.Runtime.InteropServices.StructLayoutAttribute(System.Runtime.InteropServices.LayoutKind.Sequential)]
public struct b3VRControllerEvent
{

    /// int
    public int m_controllerId;

    /// int
    public int m_deviceType;

    /// int
    public int m_numMoveEvents;

    /// int
    public int m_numButtonEvents;

    /// float[4]
    [System.Runtime.InteropServices.MarshalAsAttribute(System.Runtime.InteropServices.UnmanagedType.ByValArray, SizeConst = 4, ArraySubType = System.Runtime.InteropServices.UnmanagedType.R4)]
    public float[] m_pos;

    /// float[4]
    [System.Runtime.InteropServices.MarshalAsAttribute(System.Runtime.InteropServices.UnmanagedType.ByValArray, SizeConst = 4, ArraySubType = System.Runtime.InteropServices.UnmanagedType.R4)]
    public float[] m_orn;

    /// float
    public float m_analogAxis;

    /// float[]
    public float[] m_auxAnalogAxis;

    /// int[64]
    [System.Runtime.InteropServices.MarshalAsAttribute(System.Runtime.InteropServices.UnmanagedType.ByValArray, SizeConst = 64, ArraySubType = System.Runtime.InteropServices.UnmanagedType.I4)]
    public int[] m_buttons;
}

[System.Runtime.InteropServices.StructLayoutAttribute(System.Runtime.InteropServices.LayoutKind.Sequential)]
public struct b3VREventsData
{

    /// int
    public int m_numControllerEvents;

    /// b3VRControllerEvent*
    public System.IntPtr m_controllerEvents;
}

[System.Runtime.InteropServices.StructLayoutAttribute(System.Runtime.InteropServices.LayoutKind.Sequential)]
public struct b3KeyboardEvent
{

    /// int
    public int m_keyCode;

    /// int
    public int m_keyState;
}

[System.Runtime.InteropServices.StructLayoutAttribute(System.Runtime.InteropServices.LayoutKind.Sequential)]
public struct b3KeyboardEventsData
{

    /// int
    public int m_numKeyboardEvents;

    /// b3KeyboardEvent*
    public System.IntPtr m_keyboardEvents;
}

public enum eMouseEventTypeEnums
{

    /// MOUSE_MOVE_EVENT -> 1
    MOUSE_MOVE_EVENT = 1,

    /// MOUSE_BUTTON_EVENT -> 2
    MOUSE_BUTTON_EVENT = 2,
}

[System.Runtime.InteropServices.StructLayoutAttribute(System.Runtime.InteropServices.LayoutKind.Sequential)]
public struct b3MouseEvent
{

    /// int
    public int m_eventType;

    /// float
    public float m_mousePosX;

    /// float
    public float m_mousePosY;

    /// int
    public int m_buttonIndex;

    /// int
    public int m_buttonState;
}

[System.Runtime.InteropServices.StructLayoutAttribute(System.Runtime.InteropServices.LayoutKind.Sequential)]
public struct b3MouseEventsData
{

    /// int
    public int m_numMouseEvents;

    /// b3MouseEvent*
    public System.IntPtr m_mouseEvents;
}

public enum b3NotificationType
{

    /// SIMULATION_RESET -> 0
    SIMULATION_RESET = 0,

    /// BODY_ADDED -> 1
    BODY_ADDED = 1,

    /// BODY_REMOVED -> 2
    BODY_REMOVED = 2,

    /// USER_DATA_ADDED -> 3
    USER_DATA_ADDED = 3,

    /// USER_DATA_REMOVED -> 4
    USER_DATA_REMOVED = 4,

    /// LINK_DYNAMICS_CHANGED -> 5
    LINK_DYNAMICS_CHANGED = 5,

    /// VISUAL_SHAPE_CHANGED -> 6
    VISUAL_SHAPE_CHANGED = 6,

    /// TRANSFORM_CHANGED -> 7
    TRANSFORM_CHANGED = 7,

    /// SIMULATION_STEPPED -> 8
    SIMULATION_STEPPED = 8,

    /// SOFTBODY_CHANGED -> 9
    SOFTBODY_CHANGED = 9,
}

public enum b3ResetSimulationFlags
{

    /// RESET_USE_DEFORMABLE_WORLD -> 1
    RESET_USE_DEFORMABLE_WORLD = 1,

    /// RESET_USE_DISCRETE_DYNAMICS_WORLD -> 2
    RESET_USE_DISCRETE_DYNAMICS_WORLD = 2,

    /// RESET_USE_SIMPLE_BROADPHASE -> 4
    RESET_USE_SIMPLE_BROADPHASE = 4,
}

[System.Runtime.InteropServices.StructLayoutAttribute(System.Runtime.InteropServices.LayoutKind.Sequential)]
public struct b3BodyNotificationArgs
{

    /// int
    public int m_bodyUniqueId;
}

[System.Runtime.InteropServices.StructLayoutAttribute(System.Runtime.InteropServices.LayoutKind.Sequential, CharSet = System.Runtime.InteropServices.CharSet.Ansi)]
public struct b3UserDataNotificationArgs
{

    /// int
    public int m_bodyUniqueId;

    /// int
    public int m_linkIndex;

    /// int
    public int m_visualShapeIndex;

    /// int
    public int m_userDataId;

    /// char[256]
    [System.Runtime.InteropServices.MarshalAsAttribute(System.Runtime.InteropServices.UnmanagedType.ByValTStr, SizeConst = 256)]
    public string m_key;
}

[System.Runtime.InteropServices.StructLayoutAttribute(System.Runtime.InteropServices.LayoutKind.Sequential)]
public struct b3LinkNotificationArgs
{

    /// int
    public int m_bodyUniqueId;

    /// int
    public int m_linkIndex;
}

[System.Runtime.InteropServices.StructLayoutAttribute(System.Runtime.InteropServices.LayoutKind.Sequential)]
public struct b3VisualShapeNotificationArgs
{

    /// int
    public int m_bodyUniqueId;

    /// int
    public int m_linkIndex;

    /// int
    public int m_visualShapeIndex;
}

[System.Runtime.InteropServices.StructLayoutAttribute(System.Runtime.InteropServices.LayoutKind.Sequential)]
public struct b3TransformChangeNotificationArgs
{

    /// int
    public int m_bodyUniqueId;

    /// int
    public int m_linkIndex;

    /// double[3]
    [System.Runtime.InteropServices.MarshalAsAttribute(System.Runtime.InteropServices.UnmanagedType.ByValArray, SizeConst = 3, ArraySubType = System.Runtime.InteropServices.UnmanagedType.R8)]
    public double[] m_worldPosition;

    /// double[4]
    [System.Runtime.InteropServices.MarshalAsAttribute(System.Runtime.InteropServices.UnmanagedType.ByValArray, SizeConst = 4, ArraySubType = System.Runtime.InteropServices.UnmanagedType.R8)]
    public double[] m_worldRotation;

    /// double[3]
    [System.Runtime.InteropServices.MarshalAsAttribute(System.Runtime.InteropServices.UnmanagedType.ByValArray, SizeConst = 3, ArraySubType = System.Runtime.InteropServices.UnmanagedType.R8)]
    public double[] m_localScaling;
}

[System.Runtime.InteropServices.StructLayoutAttribute(System.Runtime.InteropServices.LayoutKind.Sequential)]
public struct b3SoftBodyChangeNotificationArgs
{

    /// int
    public int m_bodyUniqueId;

    /// int
    public int m_linkIndex;
}

[System.Runtime.InteropServices.StructLayoutAttribute(System.Runtime.InteropServices.LayoutKind.Explicit)]
public struct Anonymous_69ab7ca8_cd54_4141_9586_3363bda263af
{

    /// b3BodyNotificationArgs
    [System.Runtime.InteropServices.FieldOffsetAttribute(0)]
    public b3BodyNotificationArgs m_bodyArgs;

    /// b3UserDataNotificationArgs
    [System.Runtime.InteropServices.FieldOffsetAttribute(0)]
    public b3UserDataNotificationArgs m_userDataArgs;

    /// b3LinkNotificationArgs
    [System.Runtime.InteropServices.FieldOffsetAttribute(0)]
    public b3LinkNotificationArgs m_linkArgs;

    /// b3VisualShapeNotificationArgs
    [System.Runtime.InteropServices.FieldOffsetAttribute(0)]
    public b3VisualShapeNotificationArgs m_visualShapeArgs;

    /// b3TransformChangeNotificationArgs
    [System.Runtime.InteropServices.FieldOffsetAttribute(0)]
    public b3TransformChangeNotificationArgs m_transformChangeArgs;

    /// b3SoftBodyChangeNotificationArgs
    [System.Runtime.InteropServices.FieldOffsetAttribute(0)]
    public b3SoftBodyChangeNotificationArgs m_softBodyChangeArgs;
}

[System.Runtime.InteropServices.StructLayoutAttribute(System.Runtime.InteropServices.LayoutKind.Sequential)]
public struct b3Notification
{

    /// int
    public int m_notificationType;

    /// Anonymous_69ab7ca8_cd54_4141_9586_3363bda263af
    public Anonymous_69ab7ca8_cd54_4141_9586_3363bda263af Union1;
}

[System.Runtime.InteropServices.StructLayoutAttribute(System.Runtime.InteropServices.LayoutKind.Sequential)]
public struct b3ContactPointData
{

    /// int
    public int m_contactFlags;

    /// int
    public int m_bodyUniqueIdA;

    /// int
    public int m_bodyUniqueIdB;

    /// int
    public int m_linkIndexA;

    /// int
    public int m_linkIndexB;

    /// double[3]
    [System.Runtime.InteropServices.MarshalAsAttribute(System.Runtime.InteropServices.UnmanagedType.ByValArray, SizeConst = 3, ArraySubType = System.Runtime.InteropServices.UnmanagedType.R8)]
    public double[] m_positionOnAInWS;

    /// double[3]
    [System.Runtime.InteropServices.MarshalAsAttribute(System.Runtime.InteropServices.UnmanagedType.ByValArray, SizeConst = 3, ArraySubType = System.Runtime.InteropServices.UnmanagedType.R8)]
    public double[] m_positionOnBInWS;

    /// double[3]
    [System.Runtime.InteropServices.MarshalAsAttribute(System.Runtime.InteropServices.UnmanagedType.ByValArray, SizeConst = 3, ArraySubType = System.Runtime.InteropServices.UnmanagedType.R8)]
    public double[] m_contactNormalOnBInWS;

    /// double
    public double m_contactDistance;

    /// double
    public double m_normalForce;

    /// double
    public double m_linearFrictionForce1;

    /// double
    public double m_linearFrictionForce2;

    /// double[3]
    [System.Runtime.InteropServices.MarshalAsAttribute(System.Runtime.InteropServices.UnmanagedType.ByValArray, SizeConst = 3, ArraySubType = System.Runtime.InteropServices.UnmanagedType.R8)]
    public double[] m_linearFrictionDirection1;

    /// double[3]
    [System.Runtime.InteropServices.MarshalAsAttribute(System.Runtime.InteropServices.UnmanagedType.ByValArray, SizeConst = 3, ArraySubType = System.Runtime.InteropServices.UnmanagedType.R8)]
    public double[] m_linearFrictionDirection2;
}

public enum Anonymous_8995a57c_ad0d_4ef8_852e_e41a4050ceb5
{

    /// CONTACT_QUERY_MODE_REPORT_EXISTING_CONTACT_POINTS -> 0
    CONTACT_QUERY_MODE_REPORT_EXISTING_CONTACT_POINTS = 0,

    /// CONTACT_QUERY_MODE_COMPUTE_CLOSEST_POINTS -> 1
    CONTACT_QUERY_MODE_COMPUTE_CLOSEST_POINTS = 1,
}

public enum b3StateLoggingType
{

    /// STATE_LOGGING_MINITAUR -> 0
    STATE_LOGGING_MINITAUR = 0,

    /// STATE_LOGGING_GENERIC_ROBOT -> 1
    STATE_LOGGING_GENERIC_ROBOT = 1,

    /// STATE_LOGGING_VR_CONTROLLERS -> 2
    STATE_LOGGING_VR_CONTROLLERS = 2,

    /// STATE_LOGGING_VIDEO_MP4 -> 3
    STATE_LOGGING_VIDEO_MP4 = 3,

    /// STATE_LOGGING_COMMANDS -> 4
    STATE_LOGGING_COMMANDS = 4,

    /// STATE_LOGGING_CONTACT_POINTS -> 5
    STATE_LOGGING_CONTACT_POINTS = 5,

    /// STATE_LOGGING_PROFILE_TIMINGS -> 6
    STATE_LOGGING_PROFILE_TIMINGS = 6,

    /// STATE_LOGGING_ALL_COMMANDS -> 7
    STATE_LOGGING_ALL_COMMANDS = 7,

    /// STATE_REPLAY_ALL_COMMANDS -> 8
    STATE_REPLAY_ALL_COMMANDS = 8,

    /// STATE_LOGGING_CUSTOM_TIMER -> 9
    STATE_LOGGING_CUSTOM_TIMER = 9,
}

[System.Runtime.InteropServices.StructLayoutAttribute(System.Runtime.InteropServices.LayoutKind.Sequential)]
public struct b3ContactInformation
{

    /// int
    public int m_numContactPoints;

    /// b3ContactPointData*
    public System.IntPtr m_contactPointData;
}

[System.Runtime.InteropServices.StructLayoutAttribute(System.Runtime.InteropServices.LayoutKind.Sequential)]
public struct b3RayData
{

    /// double[3]
    [System.Runtime.InteropServices.MarshalAsAttribute(System.Runtime.InteropServices.UnmanagedType.ByValArray, SizeConst = 3, ArraySubType = System.Runtime.InteropServices.UnmanagedType.R8)]
    public double[] m_rayFromPosition;

    /// double[3]
    [System.Runtime.InteropServices.MarshalAsAttribute(System.Runtime.InteropServices.UnmanagedType.ByValArray, SizeConst = 3, ArraySubType = System.Runtime.InteropServices.UnmanagedType.R8)]
    public double[] m_rayToPosition;
}

[System.Runtime.InteropServices.StructLayoutAttribute(System.Runtime.InteropServices.LayoutKind.Sequential)]
public struct b3RayHitInfo
{

    /// double
    public double m_hitFraction;

    /// int
    public int m_hitObjectUniqueId;

    /// int
    public int m_hitObjectLinkIndex;

    /// double[3]
    [System.Runtime.InteropServices.MarshalAsAttribute(System.Runtime.InteropServices.UnmanagedType.ByValArray, SizeConst = 3, ArraySubType = System.Runtime.InteropServices.UnmanagedType.R8)]
    public double[] m_hitPositionWorld;

    /// double[3]
    [System.Runtime.InteropServices.MarshalAsAttribute(System.Runtime.InteropServices.UnmanagedType.ByValArray, SizeConst = 3, ArraySubType = System.Runtime.InteropServices.UnmanagedType.R8)]
    public double[] m_hitNormalWorld;
}

[System.Runtime.InteropServices.StructLayoutAttribute(System.Runtime.InteropServices.LayoutKind.Sequential)]
public struct b3RaycastInformation
{

    /// int
    public int m_numRayHits;

    /// b3RayHitInfo*
    public System.IntPtr m_rayHits;
}

[System.Runtime.InteropServices.StructLayoutAttribute(System.Runtime.InteropServices.LayoutKind.Explicit)]
public struct RAY_DATA_UNION
{

    /// b3RayData
    [System.Runtime.InteropServices.FieldOffsetAttribute(0)]
    public b3RayData a;

    /// b3RayHitInfo
    [System.Runtime.InteropServices.FieldOffsetAttribute(0)]
    public b3RayHitInfo b;
}

public enum b3VisualShapeDataFlags
{

    /// eVISUAL_SHAPE_DATA_TEXTURE_UNIQUE_IDS -> 1
    eVISUAL_SHAPE_DATA_TEXTURE_UNIQUE_IDS = 1,
}

[System.Runtime.InteropServices.StructLayoutAttribute(System.Runtime.InteropServices.LayoutKind.Sequential, CharSet = System.Runtime.InteropServices.CharSet.Ansi)]
public struct b3VisualShapeData
{

    /// int
    public int m_objectUniqueId;

    /// int
    public int m_linkIndex;

    /// int
    public int m_visualGeometryType;

    /// double[3]
    [System.Runtime.InteropServices.MarshalAsAttribute(System.Runtime.InteropServices.UnmanagedType.ByValArray, SizeConst = 3, ArraySubType = System.Runtime.InteropServices.UnmanagedType.R8)]
    public double[] m_dimensions;

    /// char[1024]
    [System.Runtime.InteropServices.MarshalAsAttribute(System.Runtime.InteropServices.UnmanagedType.ByValTStr, SizeConst = 1024)]
    public string m_meshAssetFileName;

    /// double[7]
    [System.Runtime.InteropServices.MarshalAsAttribute(System.Runtime.InteropServices.UnmanagedType.ByValArray, SizeConst = 7, ArraySubType = System.Runtime.InteropServices.UnmanagedType.R8)]
    public double[] m_localVisualFrame;

    /// double[4]
    [System.Runtime.InteropServices.MarshalAsAttribute(System.Runtime.InteropServices.UnmanagedType.ByValArray, SizeConst = 4, ArraySubType = System.Runtime.InteropServices.UnmanagedType.R8)]
    public double[] m_rgbaColor;

    /// int
    public int m_tinyRendererTextureId;

    /// int
    public int m_textureUniqueId;

    /// int
    public int m_openglTextureId;
}

[System.Runtime.InteropServices.StructLayoutAttribute(System.Runtime.InteropServices.LayoutKind.Sequential)]
public struct b3VisualShapeInformation
{

    /// int
    public int m_numVisualShapes;

    /// b3VisualShapeData*
    public System.IntPtr m_visualShapeData;
}

[System.Runtime.InteropServices.StructLayoutAttribute(System.Runtime.InteropServices.LayoutKind.Sequential, CharSet = System.Runtime.InteropServices.CharSet.Ansi)]
public struct b3CollisionShapeData
{

    /// int
    public int m_objectUniqueId;

    /// int
    public int m_linkIndex;

    /// int
    public int m_collisionGeometryType;

    /// double[3]
    [System.Runtime.InteropServices.MarshalAsAttribute(System.Runtime.InteropServices.UnmanagedType.ByValArray, SizeConst = 3, ArraySubType = System.Runtime.InteropServices.UnmanagedType.R8)]
    public double[] m_dimensions;

    /// double[7]
    [System.Runtime.InteropServices.MarshalAsAttribute(System.Runtime.InteropServices.UnmanagedType.ByValArray, SizeConst = 7, ArraySubType = System.Runtime.InteropServices.UnmanagedType.R8)]
    public double[] m_localCollisionFrame;

    /// char[1024]
    [System.Runtime.InteropServices.MarshalAsAttribute(System.Runtime.InteropServices.UnmanagedType.ByValTStr, SizeConst = 1024)]
    public string m_meshAssetFileName;
}

[System.Runtime.InteropServices.StructLayoutAttribute(System.Runtime.InteropServices.LayoutKind.Sequential)]
public struct b3CollisionShapeInformation
{

    /// int
    public int m_numCollisionShapes;

    /// b3CollisionShapeData*
    public System.IntPtr m_collisionShapeData;
}

public enum eLinkStateFlags
{

    /// ACTUAL_STATE_COMPUTE_LINKVELOCITY -> 1
    ACTUAL_STATE_COMPUTE_LINKVELOCITY = 1,

    /// ACTUAL_STATE_COMPUTE_FORWARD_KINEMATICS -> 2
    ACTUAL_STATE_COMPUTE_FORWARD_KINEMATICS = 2,
}

[System.Runtime.InteropServices.StructLayoutAttribute(System.Runtime.InteropServices.LayoutKind.Sequential)]
public struct b3LinkState
{

    /// double[3]
    [System.Runtime.InteropServices.MarshalAsAttribute(System.Runtime.InteropServices.UnmanagedType.ByValArray, SizeConst = 3, ArraySubType = System.Runtime.InteropServices.UnmanagedType.R8)]
    public double[] m_worldPosition;

    /// double[4]
    [System.Runtime.InteropServices.MarshalAsAttribute(System.Runtime.InteropServices.UnmanagedType.ByValArray, SizeConst = 4, ArraySubType = System.Runtime.InteropServices.UnmanagedType.R8)]
    public double[] m_worldOrientation;

    /// double[3]
    [System.Runtime.InteropServices.MarshalAsAttribute(System.Runtime.InteropServices.UnmanagedType.ByValArray, SizeConst = 3, ArraySubType = System.Runtime.InteropServices.UnmanagedType.R8)]
    public double[] m_localInertialPosition;

    /// double[4]
    [System.Runtime.InteropServices.MarshalAsAttribute(System.Runtime.InteropServices.UnmanagedType.ByValArray, SizeConst = 4, ArraySubType = System.Runtime.InteropServices.UnmanagedType.R8)]
    public double[] m_localInertialOrientation;

    /// double[3]
    [System.Runtime.InteropServices.MarshalAsAttribute(System.Runtime.InteropServices.UnmanagedType.ByValArray, SizeConst = 3, ArraySubType = System.Runtime.InteropServices.UnmanagedType.R8)]
    public double[] m_worldLinkFramePosition;

    /// double[4]
    [System.Runtime.InteropServices.MarshalAsAttribute(System.Runtime.InteropServices.UnmanagedType.ByValArray, SizeConst = 4, ArraySubType = System.Runtime.InteropServices.UnmanagedType.R8)]
    public double[] m_worldLinkFrameOrientation;

    /// double[3]
    [System.Runtime.InteropServices.MarshalAsAttribute(System.Runtime.InteropServices.UnmanagedType.ByValArray, SizeConst = 3, ArraySubType = System.Runtime.InteropServices.UnmanagedType.R8)]
    public double[] m_worldLinearVelocity;

    /// double[3]
    [System.Runtime.InteropServices.MarshalAsAttribute(System.Runtime.InteropServices.UnmanagedType.ByValArray, SizeConst = 3, ArraySubType = System.Runtime.InteropServices.UnmanagedType.R8)]
    public double[] m_worldAngularVelocity;

    /// double[3]
    [System.Runtime.InteropServices.MarshalAsAttribute(System.Runtime.InteropServices.UnmanagedType.ByValArray, SizeConst = 3, ArraySubType = System.Runtime.InteropServices.UnmanagedType.R8)]
    public double[] m_worldAABBMin;

    /// double[3]
    [System.Runtime.InteropServices.MarshalAsAttribute(System.Runtime.InteropServices.UnmanagedType.ByValArray, SizeConst = 3, ArraySubType = System.Runtime.InteropServices.UnmanagedType.R8)]
    public double[] m_worldAABBMax;
}

public enum EnumControlMode
{

    /// CONTROL_MODE_VELOCITY -> 0
    CONTROL_MODE_VELOCITY = 0,

    CONTROL_MODE_TORQUE,

    CONTROL_MODE_POSITION_VELOCITY_PD,

    CONTROL_MODE_PD,

    CONTROL_MODE_STABLE_PD,
}

public enum EnumExternalForceFlags
{

    /// EF_LINK_FRAME -> 1
    EF_LINK_FRAME = 1,

    /// EF_WORLD_FRAME -> 2
    EF_WORLD_FRAME = 2,
}

public enum EnumRenderer
{

    /// ER_TINY_RENDERER -> (1<<16)
    ER_TINY_RENDERER = (1) << (16),

    /// ER_BULLET_HARDWARE_OPENGL -> (1<<17)
    ER_BULLET_HARDWARE_OPENGL = (1) << (17),
}

public enum EnumRendererAuxFlags
{

    /// ER_SEGMENTATION_MASK_OBJECT_AND_LINKINDEX -> 1
    ER_SEGMENTATION_MASK_OBJECT_AND_LINKINDEX = 1,

    /// ER_USE_PROJECTIVE_TEXTURE -> 2
    ER_USE_PROJECTIVE_TEXTURE = 2,

    /// ER_NO_SEGMENTATION_MASK -> 4
    ER_NO_SEGMENTATION_MASK = 4,
}

public enum EnumCalculateInverseKinematicsFlags
{

    /// IK_DLS -> 0
    IK_DLS = 0,

    /// IK_SDLS -> 1
    IK_SDLS = 1,

    /// IK_HAS_TARGET_POSITION -> 16
    IK_HAS_TARGET_POSITION = 16,

    /// IK_HAS_TARGET_ORIENTATION -> 32
    IK_HAS_TARGET_ORIENTATION = 32,

    /// IK_HAS_NULL_SPACE_VELOCITY -> 64
    IK_HAS_NULL_SPACE_VELOCITY = 64,

    /// IK_HAS_JOINT_DAMPING -> 128
    IK_HAS_JOINT_DAMPING = 128,

    /// IK_HAS_CURRENT_JOINT_POSITIONS -> 256
    IK_HAS_CURRENT_JOINT_POSITIONS = 256,

    /// IK_HAS_MAX_ITERATIONS -> 512
    IK_HAS_MAX_ITERATIONS = 512,

    /// IK_HAS_RESIDUAL_THRESHOLD -> 1024
    IK_HAS_RESIDUAL_THRESHOLD = 1024,
}

public enum b3ConfigureDebugVisualizerEnum
{

    /// COV_ENABLE_GUI -> 1
    COV_ENABLE_GUI = 1,

    COV_ENABLE_SHADOWS,

    COV_ENABLE_WIREFRAME,

    COV_ENABLE_VR_TELEPORTING,

    COV_ENABLE_VR_PICKING,

    COV_ENABLE_VR_RENDER_CONTROLLERS,

    COV_ENABLE_RENDERING,

    COV_ENABLE_SYNC_RENDERING_INTERNAL,

    COV_ENABLE_KEYBOARD_SHORTCUTS,

    COV_ENABLE_MOUSE_PICKING,

    COV_ENABLE_Y_AXIS_UP,

    COV_ENABLE_TINY_RENDERER,

    COV_ENABLE_RGB_BUFFER_PREVIEW,

    COV_ENABLE_DEPTH_BUFFER_PREVIEW,

    COV_ENABLE_SEGMENTATION_MARK_PREVIEW,

    COV_ENABLE_PLANAR_REFLECTION,

    COV_ENABLE_SINGLE_STEP_RENDERING,
}

public enum b3AddUserDebugItemEnum
{

    /// DEB_DEBUG_TEXT_ALWAYS_FACE_CAMERA -> 1
    DEB_DEBUG_TEXT_ALWAYS_FACE_CAMERA = 1,

    /// DEB_DEBUG_TEXT_USE_TRUE_TYPE_FONTS -> 2
    DEB_DEBUG_TEXT_USE_TRUE_TYPE_FONTS = 2,

    /// DEB_DEBUG_TEXT_HAS_TRACKING_OBJECT -> 4
    DEB_DEBUG_TEXT_HAS_TRACKING_OBJECT = 4,
}

public enum eCONNECT_METHOD
{

    /// eCONNECT_GUI -> 1
    eCONNECT_GUI = 1,

    /// eCONNECT_DIRECT -> 2
    eCONNECT_DIRECT = 2,

    /// eCONNECT_SHARED_MEMORY -> 3
    eCONNECT_SHARED_MEMORY = 3,

    /// eCONNECT_UDP -> 4
    eCONNECT_UDP = 4,

    /// eCONNECT_TCP -> 5
    eCONNECT_TCP = 5,

    /// eCONNECT_EXISTING_EXAMPLE_BROWSER -> 6
    eCONNECT_EXISTING_EXAMPLE_BROWSER = 6,

    /// eCONNECT_GUI_SERVER -> 7
    eCONNECT_GUI_SERVER = 7,

    /// eCONNECT_GUI_MAIN_THREAD -> 8
    eCONNECT_GUI_MAIN_THREAD = 8,

    /// eCONNECT_SHARED_MEMORY_SERVER -> 9
    eCONNECT_SHARED_MEMORY_SERVER = 9,

    /// eCONNECT_DART -> 10
    eCONNECT_DART = 10,

    /// eCONNECT_MUJOCO -> 11
    eCONNECT_MUJOCO = 11,

    /// eCONNECT_GRPC -> 12
    eCONNECT_GRPC = 12,

    /// eCONNECT_PHYSX -> 13
    eCONNECT_PHYSX = 13,

    /// eCONNECT_SHARED_MEMORY_GUI -> 14
    eCONNECT_SHARED_MEMORY_GUI = 14,

    /// eCONNECT_GRAPHICS_SERVER -> 15
    eCONNECT_GRAPHICS_SERVER = 15,

    /// eCONNECT_GRAPHICS_SERVER_TCP -> 16
    eCONNECT_GRAPHICS_SERVER_TCP = 16,

    /// eCONNECT_GRAPHICS_SERVER_MAIN_THREAD -> 17
    eCONNECT_GRAPHICS_SERVER_MAIN_THREAD = 17,
}

public enum eURDF_Flags
{

    /// URDF_USE_INERTIA_FROM_FILE -> 2
    URDF_USE_INERTIA_FROM_FILE = 2,

    /// URDF_USE_SELF_COLLISION -> 8
    URDF_USE_SELF_COLLISION = 8,

    /// URDF_USE_SELF_COLLISION_EXCLUDE_PARENT -> 16
    URDF_USE_SELF_COLLISION_EXCLUDE_PARENT = 16,

    /// URDF_USE_SELF_COLLISION_EXCLUDE_ALL_PARENTS -> 32
    URDF_USE_SELF_COLLISION_EXCLUDE_ALL_PARENTS = 32,

    /// URDF_RESERVED -> 64
    URDF_RESERVED = 64,

    /// URDF_USE_IMPLICIT_CYLINDER -> 128
    URDF_USE_IMPLICIT_CYLINDER = 128,

    /// URDF_GLOBAL_VELOCITIES_MB -> 256
    URDF_GLOBAL_VELOCITIES_MB = 256,

    /// MJCF_COLORS_FROM_FILE -> 512
    MJCF_COLORS_FROM_FILE = 512,

    /// URDF_ENABLE_CACHED_GRAPHICS_SHAPES -> 1024
    URDF_ENABLE_CACHED_GRAPHICS_SHAPES = 1024,

    /// URDF_ENABLE_SLEEPING -> 2048
    URDF_ENABLE_SLEEPING = 2048,

    /// URDF_INITIALIZE_SAT_FEATURES -> 4096
    URDF_INITIALIZE_SAT_FEATURES = 4096,

    /// URDF_USE_SELF_COLLISION_INCLUDE_PARENT -> 8192
    URDF_USE_SELF_COLLISION_INCLUDE_PARENT = 8192,

    /// URDF_PARSE_SENSORS -> 16384
    URDF_PARSE_SENSORS = 16384,

    /// URDF_USE_MATERIAL_COLORS_FROM_MTL -> 32768
    URDF_USE_MATERIAL_COLORS_FROM_MTL = 32768,

    /// URDF_USE_MATERIAL_TRANSPARANCY_FROM_MTL -> 65536
    URDF_USE_MATERIAL_TRANSPARANCY_FROM_MTL = 65536,

    /// URDF_MAINTAIN_LINK_ORDER -> 131072
    URDF_MAINTAIN_LINK_ORDER = 131072,

    /// URDF_ENABLE_WAKEUP -> 1<<18
    URDF_ENABLE_WAKEUP = (1) << (18),

    /// URDF_MERGE_FIXED_LINKS -> 1<<19
    URDF_MERGE_FIXED_LINKS = (1) << (19),

    /// URDF_IGNORE_VISUAL_SHAPES -> 1<<20
    URDF_IGNORE_VISUAL_SHAPES = (1) << (20),

    /// URDF_IGNORE_COLLISION_SHAPES -> 1<<21
    URDF_IGNORE_COLLISION_SHAPES = (1) << (21),

    /// URDF_PRINT_URDF_INFO -> 1<<22
    URDF_PRINT_URDF_INFO = (1) << (22),
}

public enum eUrdfGeomTypes
{

    /// GEOM_SPHERE -> 2
    GEOM_SPHERE = 2,

    GEOM_BOX,

    GEOM_CYLINDER,

    GEOM_MESH,

    GEOM_PLANE,

    GEOM_CAPSULE,

    GEOM_SDF,

    GEOM_HEIGHTFIELD,

    GEOM_UNKNOWN,
}

public enum eUrdfCollisionFlags
{

    /// GEOM_FORCE_CONCAVE_TRIMESH -> 1
    GEOM_FORCE_CONCAVE_TRIMESH = 1,

    /// GEOM_CONCAVE_INTERNAL_EDGE -> 2
    GEOM_CONCAVE_INTERNAL_EDGE = 2,

    /// GEOM_INITIALIZE_SAT_FEATURES -> URDF_INITIALIZE_SAT_FEATURES
    GEOM_INITIALIZE_SAT_FEATURES = eURDF_Flags.URDF_INITIALIZE_SAT_FEATURES,
}

public enum eUrdfVisualFlags
{

    /// GEOM_VISUAL_HAS_RGBA_COLOR -> 1
    GEOM_VISUAL_HAS_RGBA_COLOR = 1,

    /// GEOM_VISUAL_HAS_SPECULAR_COLOR -> 2
    GEOM_VISUAL_HAS_SPECULAR_COLOR = 2,
}

public enum eStateLoggingFlags
{

    /// STATE_LOG_JOINT_MOTOR_TORQUES -> 1
    STATE_LOG_JOINT_MOTOR_TORQUES = 1,

    /// STATE_LOG_JOINT_USER_TORQUES -> 2
    STATE_LOG_JOINT_USER_TORQUES = 2,

    /// STATE_LOG_JOINT_TORQUES -> STATE_LOG_JOINT_MOTOR_TORQUES+STATE_LOG_JOINT_USER_TORQUES
    STATE_LOG_JOINT_TORQUES = (eStateLoggingFlags.STATE_LOG_JOINT_MOTOR_TORQUES + eStateLoggingFlags.STATE_LOG_JOINT_USER_TORQUES),
}

public enum eJointFeedbackModes
{

    /// JOINT_FEEDBACK_IN_WORLD_SPACE -> 1
    JOINT_FEEDBACK_IN_WORLD_SPACE = 1,

    /// JOINT_FEEDBACK_IN_JOINT_FRAME -> 2
    JOINT_FEEDBACK_IN_JOINT_FRAME = 2,
}

[System.Runtime.InteropServices.StructLayoutAttribute(System.Runtime.InteropServices.LayoutKind.Sequential, CharSet = System.Runtime.InteropServices.CharSet.Ansi)]
public struct b3PluginArguments
{

    /// char[1024]
    [System.Runtime.InteropServices.MarshalAsAttribute(System.Runtime.InteropServices.UnmanagedType.ByValTStr, SizeConst = 1024)]
    public string m_text;

    /// int
    public int m_numInts;

    /// int[128]
    [System.Runtime.InteropServices.MarshalAsAttribute(System.Runtime.InteropServices.UnmanagedType.ByValArray, SizeConst = 128, ArraySubType = System.Runtime.InteropServices.UnmanagedType.I4)]
    public int[] m_ints;

    /// int
    public int m_numFloats;

    /// double[128]
    [System.Runtime.InteropServices.MarshalAsAttribute(System.Runtime.InteropServices.UnmanagedType.ByValArray, SizeConst = 128, ArraySubType = System.Runtime.InteropServices.UnmanagedType.R8)]
    public double[] m_floats;
}

[System.Runtime.InteropServices.StructLayoutAttribute(System.Runtime.InteropServices.LayoutKind.Sequential)]
public struct b3PhysicsSimulationParameters
{

    /// double
    public double m_deltaTime;

    /// double
    public double m_simulationTimestamp;

    /// double[3]
    [System.Runtime.InteropServices.MarshalAsAttribute(System.Runtime.InteropServices.UnmanagedType.ByValArray, SizeConst = 3, ArraySubType = System.Runtime.InteropServices.UnmanagedType.R8)]
    public double[] m_gravityAcceleration;

    /// int
    public int m_numSimulationSubSteps;

    /// int
    public int m_numSolverIterations;

    /// double
    public double m_warmStartingFactor;

    /// double
    public double m_articulatedWarmStartingFactor;

    /// int
    public int m_useRealTimeSimulation;

    /// int
    public int m_useSplitImpulse;

    /// double
    public double m_splitImpulsePenetrationThreshold;

    /// double
    public double m_contactBreakingThreshold;

    /// int
    public int m_internalSimFlags;

    /// double
    public double m_defaultContactERP;

    /// int
    public int m_collisionFilterMode;

    /// int
    public int m_enableFileCaching;

    /// double
    public double m_restitutionVelocityThreshold;

    /// double
    public double m_defaultNonContactERP;

    /// double
    public double m_frictionERP;

    /// double
    public double m_defaultGlobalCFM;

    /// double
    public double m_frictionCFM;

    /// int
    public int m_enableConeFriction;

    /// int
    public int m_deterministicOverlappingPairs;

    /// double
    public double m_allowedCcdPenetration;

    /// int
    public int m_jointFeedbackMode;

    /// double
    public double m_solverResidualThreshold;

    /// double
    public double m_contactSlop;

    /// int
    public int m_enableSAT;

    /// int
    public int m_constraintSolverType;

    /// int
    public int m_minimumSolverIslandSize;

    /// int
    public int m_reportSolverAnalytics;

    /// double
    public double m_sparseSdfVoxelSize;

    /// int
    public int m_numNonContactInnerIterations;
}

public enum eConstraintSolverTypes
{

    /// eConstraintSolverLCP_SI -> 1
    eConstraintSolverLCP_SI = 1,

    eConstraintSolverLCP_PGS,

    eConstraintSolverLCP_DANTZIG,

    eConstraintSolverLCP_LEMKE,

    eConstraintSolverLCP_NNCG,

    eConstraintSolverLCP_BLOCK_PGS,
}

[System.Runtime.InteropServices.StructLayoutAttribute(System.Runtime.InteropServices.LayoutKind.Sequential)]
public struct b3ForwardDynamicsAnalyticsIslandData
{

    /// int
    public int m_islandId;

    /// int
    public int m_numBodies;

    /// int
    public int m_numContactManifolds;

    /// int
    public int m_numIterationsUsed;

    /// double
    public double m_remainingLeastSquaresResidual;
}

[System.Runtime.InteropServices.StructLayoutAttribute(System.Runtime.InteropServices.LayoutKind.Sequential)]
public struct b3ForwardDynamicsAnalyticsArgs
{

    /// int
    public int m_numSteps;

    /// int
    public int m_numIslands;

    /// int
    public int m_numSolverCalls;

    /// b3ForwardDynamicsAnalyticsIslandData[64]
    [System.Runtime.InteropServices.MarshalAsAttribute(System.Runtime.InteropServices.UnmanagedType.ByValArray, SizeConst = 64, ArraySubType = System.Runtime.InteropServices.UnmanagedType.Struct)]
    public b3ForwardDynamicsAnalyticsIslandData[] m_islandData;
}

public enum eFileIOActions
{

    /// eAddFileIOAction -> 1024
    eAddFileIOAction = 1024,

    eRemoveFileIOAction,
}

public enum eFileIOTypes
{

    /// ePosixFileIO -> 1
    ePosixFileIO = 1,

    eZipFileIO,

    eCNSFileIO,

    eInMemoryFileIO,
}

[System.Runtime.InteropServices.StructLayoutAttribute(System.Runtime.InteropServices.LayoutKind.Sequential)]
public struct b3PhysicsClientHandle__
{

    /// int
    public int unused;
}

[System.Runtime.InteropServices.StructLayoutAttribute(System.Runtime.InteropServices.LayoutKind.Sequential)]
public struct b3SharedMemoryCommandHandle__
{

    /// int
    public int unused;
}

[System.Runtime.InteropServices.StructLayoutAttribute(System.Runtime.InteropServices.LayoutKind.Sequential)]
public struct b3SharedMemoryStatusHandle__
{

    /// int
    public int unused;
}

public partial class NativeMethods
{
    const string dllName = "pybullet_vs2010_x64_release.dll";
    /// Return Type: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    ///key: int
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3ConnectSharedMemory")]
    public static extern System.IntPtr b3ConnectSharedMemory(int key);


    /// Return Type: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    ///key: int
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3ConnectSharedMemory2")]
    public static extern System.IntPtr b3ConnectSharedMemory2(int key);


    /// Return Type: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3ConnectPhysicsDirect")]
    public static extern System.IntPtr b3ConnectPhysicsDirect();


    /// Return Type: void
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3DisconnectSharedMemory")]
    public static extern void b3DisconnectSharedMemory(IntPtr physClient);


    /// Return Type: int
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3CanSubmitCommand")]
    public static extern int b3CanSubmitCommand(IntPtr physClient);


    /// Return Type: b3SharedMemoryStatusHandle->b3SharedMemoryStatusHandle__*
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3SubmitClientCommandAndWaitStatus")]
    public static extern System.IntPtr b3SubmitClientCommandAndWaitStatus(IntPtr physClient, IntPtr commandHandle);


    /// Return Type: int
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3SubmitClientCommand")]
    public static extern int b3SubmitClientCommand(IntPtr physClient, IntPtr commandHandle);


    /// Return Type: b3SharedMemoryStatusHandle->b3SharedMemoryStatusHandle__*
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3ProcessServerStatus")]
    public static extern System.IntPtr b3ProcessServerStatus(IntPtr physClient);


    /// Return Type: int
    ///statusHandle: b3SharedMemoryStatusHandle->b3SharedMemoryStatusHandle__*
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3GetStatusType")]
    public static extern int b3GetStatusType(IntPtr statusHandle);


    /// Return Type: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3CreateCustomCommand")]
    public static extern System.IntPtr b3CreateCustomCommand(IntPtr physClient);


    /// Return Type: void
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///pluginPath: char*
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3CustomCommandLoadPlugin")]
    public static extern void b3CustomCommandLoadPlugin(IntPtr commandHandle, [System.Runtime.InteropServices.InAttribute()] [System.Runtime.InteropServices.MarshalAsAttribute(System.Runtime.InteropServices.UnmanagedType.LPStr)] string pluginPath);


    /// Return Type: void
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///postFix: char*
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3CustomCommandLoadPluginSetPostFix")]
    public static extern void b3CustomCommandLoadPluginSetPostFix(IntPtr commandHandle, [System.Runtime.InteropServices.InAttribute()] [System.Runtime.InteropServices.MarshalAsAttribute(System.Runtime.InteropServices.UnmanagedType.LPStr)] string postFix);


    /// Return Type: int
    ///statusHandle: b3SharedMemoryStatusHandle->b3SharedMemoryStatusHandle__*
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3GetStatusPluginUniqueId")]
    public static extern int b3GetStatusPluginUniqueId(IntPtr statusHandle);


    /// Return Type: int
    ///statusHandle: b3SharedMemoryStatusHandle->b3SharedMemoryStatusHandle__*
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3GetStatusPluginCommandResult")]
    public static extern int b3GetStatusPluginCommandResult(IntPtr statusHandle);


    /// Return Type: void
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///pluginUniqueId: int
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3CustomCommandUnloadPlugin")]
    public static extern void b3CustomCommandUnloadPlugin(IntPtr commandHandle, int pluginUniqueId);


    /// Return Type: void
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///pluginUniqueId: int
    ///textArguments: char*
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3CustomCommandExecutePluginCommand")]
    public static extern void b3CustomCommandExecutePluginCommand(IntPtr commandHandle, int pluginUniqueId, [System.Runtime.InteropServices.InAttribute()] [System.Runtime.InteropServices.MarshalAsAttribute(System.Runtime.InteropServices.UnmanagedType.LPStr)] string textArguments);


    /// Return Type: void
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///intVal: int
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3CustomCommandExecuteAddIntArgument")]
    public static extern void b3CustomCommandExecuteAddIntArgument(IntPtr commandHandle, int intVal);


    /// Return Type: void
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///floatVal: float
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3CustomCommandExecuteAddFloatArgument")]
    public static extern void b3CustomCommandExecuteAddFloatArgument(IntPtr commandHandle, float floatVal);


    /// Return Type: int
    ///statusHandle: b3SharedMemoryStatusHandle->b3SharedMemoryStatusHandle__*
    ///bodyIndicesOut: int*
    ///bodyIndicesCapacity: int
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3GetStatusBodyIndices")]
    public static extern int b3GetStatusBodyIndices(IntPtr statusHandle, ref int bodyIndicesOut, int bodyIndicesCapacity);


    /// Return Type: int
    ///statusHandle: b3SharedMemoryStatusHandle->b3SharedMemoryStatusHandle__*
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3GetStatusBodyIndex")]
    public static extern int b3GetStatusBodyIndex(IntPtr statusHandle);


    /// Return Type: int
    ///statusHandle: b3SharedMemoryStatusHandle->b3SharedMemoryStatusHandle__*
    ///bodyUniqueId: int*
    ///numDegreeOfFreedomQ: int*
    ///numDegreeOfFreedomU: int*
    ///rootLocalInertialFrame: double**
    ///actualStateQ: double**
    ///actualStateQdot: double**
    ///jointReactionForces: double**
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3GetStatusActualState")]
    public static extern int b3GetStatusActualState(IntPtr statusHandle, ref int bodyUniqueId, ref int numDegreeOfFreedomQ, ref int numDegreeOfFreedomU, ref System.IntPtr rootLocalInertialFrame, ref System.IntPtr actualStateQ, ref System.IntPtr actualStateQdot, ref System.IntPtr jointReactionForces);


    /// Return Type: int
    ///statusHandle: b3SharedMemoryStatusHandle->b3SharedMemoryStatusHandle__*
    ///bodyUniqueId: int*
    ///numLinks: int*
    ///numDegreeOfFreedomQ: int*
    ///numDegreeOfFreedomU: int*
    ///rootLocalInertialFrame: double**
    ///actualStateQ: double**
    ///actualStateQdot: double**
    ///jointReactionForces: double**
    ///linkLocalInertialFrames: double**
    ///jointMotorForces: double**
    ///linkStates: double**
    ///linkWorldVelocities: double**
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3GetStatusActualState2")]
    public static extern int b3GetStatusActualState2(IntPtr statusHandle, ref int bodyUniqueId, ref int numLinks, ref int numDegreeOfFreedomQ, ref int numDegreeOfFreedomU, ref System.IntPtr rootLocalInertialFrame, ref System.IntPtr actualStateQ, ref System.IntPtr actualStateQdot, ref System.IntPtr jointReactionForces, ref System.IntPtr linkLocalInertialFrames, ref System.IntPtr jointMotorForces, ref System.IntPtr linkStates, ref System.IntPtr linkWorldVelocities);


    /// Return Type: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    ///bodyUniqueId: int
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3RequestCollisionInfoCommandInit")]
    public static extern System.IntPtr b3RequestCollisionInfoCommandInit(IntPtr physClient, int bodyUniqueId);


    /// Return Type: int
    ///statusHandle: b3SharedMemoryStatusHandle->b3SharedMemoryStatusHandle__*
    ///linkIndex: int
    ///aabbMin: double*
    ///aabbMax: double*
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3GetStatusAABB")]
    public static extern int b3GetStatusAABB(IntPtr statusHandle, int linkIndex, ref double aabbMin, ref double aabbMax);


    /// Return Type: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3InitSyncBodyInfoCommand")]
    public static extern System.IntPtr b3InitSyncBodyInfoCommand(IntPtr physClient);


    /// Return Type: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    ///bodyUniqueId: int
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3InitRemoveBodyCommand")]
    public static extern System.IntPtr b3InitRemoveBodyCommand(IntPtr physClient, int bodyUniqueId);


    /// Return Type: int
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3GetNumBodies")]
    public static extern int b3GetNumBodies(IntPtr physClient);


    /// Return Type: int
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    ///serialIndex: int
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3GetBodyUniqueId")]
    public static extern int b3GetBodyUniqueId(IntPtr physClient, int serialIndex);


    /// Return Type: int
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    ///bodyUniqueId: int
    ///info: b3BodyInfo*
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3GetBodyInfo")]
    public static extern int b3GetBodyInfo(IntPtr physClient, int bodyUniqueId, ref b3BodyInfo info);


    /// Return Type: int
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    ///bodyUniqueId: int
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3GetNumJoints")]
    public static extern int b3GetNumJoints(IntPtr physClient, int bodyUniqueId);


    /// Return Type: int
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    ///bodyUniqueId: int
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3GetNumDofs")]
    public static extern int b3GetNumDofs(IntPtr physClient, int bodyUniqueId);


    /// Return Type: int
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    ///bodyUniqueId: int
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3ComputeDofCount")]
    public static extern int b3ComputeDofCount(IntPtr physClient, int bodyUniqueId);


    /// Return Type: int
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    ///bodyUniqueId: int
    ///jointIndex: int
    ///info: b3JointInfo*
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3GetJointInfo")]
    public static extern int b3GetJointInfo(IntPtr physClient, int bodyUniqueId, int jointIndex, ref b3JointInfo info);


    /// Return Type: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3InitSyncUserDataCommand")]
    public static extern System.IntPtr b3InitSyncUserDataCommand(IntPtr physClient);


    /// Return Type: void
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///bodyUniqueId: int
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3AddBodyToSyncUserDataRequest")]
    public static extern void b3AddBodyToSyncUserDataRequest(IntPtr commandHandle, int bodyUniqueId);


    /// Return Type: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    ///bodyUniqueId: int
    ///linkIndex: int
    ///visualShapeIndex: int
    ///key: char*
    ///valueType: UserDataValueType
    ///valueLength: int
    ///valueData: void*
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3InitAddUserDataCommand")]
    public static extern System.IntPtr b3InitAddUserDataCommand(IntPtr physClient, int bodyUniqueId, int linkIndex, int visualShapeIndex, [System.Runtime.InteropServices.InAttribute()] [System.Runtime.InteropServices.MarshalAsAttribute(System.Runtime.InteropServices.UnmanagedType.LPStr)] string key, UserDataValueType valueType, int valueLength, System.IntPtr valueData);


    /// Return Type: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    ///userDataId: int
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3InitRemoveUserDataCommand")]
    public static extern System.IntPtr b3InitRemoveUserDataCommand(IntPtr physClient, int userDataId);


    /// Return Type: int
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    ///userDataId: int
    ///valueOut: b3UserDataValue*
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3GetUserData")]
    public static extern int b3GetUserData(IntPtr physClient, int userDataId, ref b3UserDataValue valueOut);


    /// Return Type: int
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    ///bodyUniqueId: int
    ///linkIndex: int
    ///visualShapeIndex: int
    ///key: char*
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3GetUserDataId")]
    public static extern int b3GetUserDataId(IntPtr physClient, int bodyUniqueId, int linkIndex, int visualShapeIndex, [System.Runtime.InteropServices.InAttribute()] [System.Runtime.InteropServices.MarshalAsAttribute(System.Runtime.InteropServices.UnmanagedType.LPStr)] string key);


    /// Return Type: int
    ///statusHandle: b3SharedMemoryStatusHandle->b3SharedMemoryStatusHandle__*
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3GetUserDataIdFromStatus")]
    public static extern int b3GetUserDataIdFromStatus(IntPtr statusHandle);


    /// Return Type: int
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    ///bodyUniqueId: int
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3GetNumUserData")]
    public static extern int b3GetNumUserData(IntPtr physClient, int bodyUniqueId);


    /// Return Type: void
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    ///bodyUniqueId: int
    ///userDataIndex: int
    ///keyOut: char**
    ///userDataIdOut: int*
    ///linkIndexOut: int*
    ///visualShapeIndexOut: int*
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3GetUserDataInfo")]
    public static extern void b3GetUserDataInfo(IntPtr physClient, int bodyUniqueId, int userDataIndex, ref System.IntPtr keyOut, ref int userDataIdOut, ref int linkIndexOut, ref int visualShapeIndexOut);


    /// Return Type: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    ///bodyUniqueId: int
    ///linkIndex: int
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3GetDynamicsInfoCommandInit")]
    public static extern System.IntPtr b3GetDynamicsInfoCommandInit(IntPtr physClient, int bodyUniqueId, int linkIndex);


    /// Return Type: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///bodyUniqueId: int
    ///linkIndex: int
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3GetDynamicsInfoCommandInit2")]
    public static extern System.IntPtr b3GetDynamicsInfoCommandInit2(IntPtr commandHandle, int bodyUniqueId, int linkIndex);


    /// Return Type: int
    ///statusHandle: b3SharedMemoryStatusHandle->b3SharedMemoryStatusHandle__*
    ///info: b3DynamicsInfo*
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3GetDynamicsInfo")]
    public static extern int b3GetDynamicsInfo(IntPtr statusHandle, ref b3DynamicsInfo info);


    /// Return Type: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3InitChangeDynamicsInfo")]
    public static extern System.IntPtr b3InitChangeDynamicsInfo(IntPtr physClient);


    /// Return Type: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3InitChangeDynamicsInfo2")]
    public static extern System.IntPtr b3InitChangeDynamicsInfo2(IntPtr commandHandle);


    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///bodyUniqueId: int
    ///linkIndex: int
    ///mass: double
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3ChangeDynamicsInfoSetMass")]
    public static extern int b3ChangeDynamicsInfoSetMass(IntPtr commandHandle, int bodyUniqueId, int linkIndex, double mass);


    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///bodyUniqueId: int
    ///linkIndex: int
    ///localInertiaDiagonal: double*
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3ChangeDynamicsInfoSetLocalInertiaDiagonal")]
    public static extern int b3ChangeDynamicsInfoSetLocalInertiaDiagonal(IntPtr commandHandle, int bodyUniqueId, int linkIndex, ref double localInertiaDiagonal);


    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///bodyUniqueId: int
    ///linkIndex: int
    ///anisotropicFriction: double*
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3ChangeDynamicsInfoSetAnisotropicFriction")]
    public static extern int b3ChangeDynamicsInfoSetAnisotropicFriction(IntPtr commandHandle, int bodyUniqueId, int linkIndex, ref double anisotropicFriction);


    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///bodyUniqueId: int
    ///linkIndex: int
    ///lateralFriction: double
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3ChangeDynamicsInfoSetLateralFriction")]
    public static extern int b3ChangeDynamicsInfoSetLateralFriction(IntPtr commandHandle, int bodyUniqueId, int linkIndex, double lateralFriction);


    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///bodyUniqueId: int
    ///linkIndex: int
    ///friction: double
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3ChangeDynamicsInfoSetSpinningFriction")]
    public static extern int b3ChangeDynamicsInfoSetSpinningFriction(IntPtr commandHandle, int bodyUniqueId, int linkIndex, double friction);


    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///bodyUniqueId: int
    ///linkIndex: int
    ///friction: double
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3ChangeDynamicsInfoSetRollingFriction")]
    public static extern int b3ChangeDynamicsInfoSetRollingFriction(IntPtr commandHandle, int bodyUniqueId, int linkIndex, double friction);


    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///bodyUniqueId: int
    ///linkIndex: int
    ///restitution: double
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3ChangeDynamicsInfoSetRestitution")]
    public static extern int b3ChangeDynamicsInfoSetRestitution(IntPtr commandHandle, int bodyUniqueId, int linkIndex, double restitution);


    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///bodyUniqueId: int
    ///linearDamping: double
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3ChangeDynamicsInfoSetLinearDamping")]
    public static extern int b3ChangeDynamicsInfoSetLinearDamping(IntPtr commandHandle, int bodyUniqueId, double linearDamping);


    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///bodyUniqueId: int
    ///angularDamping: double
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3ChangeDynamicsInfoSetAngularDamping")]
    public static extern int b3ChangeDynamicsInfoSetAngularDamping(IntPtr commandHandle, int bodyUniqueId, double angularDamping);


    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///bodyUniqueId: int
    ///linkIndex: int
    ///jointDamping: double
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3ChangeDynamicsInfoSetJointDamping")]
    public static extern int b3ChangeDynamicsInfoSetJointDamping(IntPtr commandHandle, int bodyUniqueId, int linkIndex, double jointDamping);


    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///bodyUniqueId: int
    ///linkIndex: int
    ///contactStiffness: double
    ///contactDamping: double
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3ChangeDynamicsInfoSetContactStiffnessAndDamping")]
    public static extern int b3ChangeDynamicsInfoSetContactStiffnessAndDamping(IntPtr commandHandle, int bodyUniqueId, int linkIndex, double contactStiffness, double contactDamping);


    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///bodyUniqueId: int
    ///linkIndex: int
    ///frictionAnchor: int
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3ChangeDynamicsInfoSetFrictionAnchor")]
    public static extern int b3ChangeDynamicsInfoSetFrictionAnchor(IntPtr commandHandle, int bodyUniqueId, int linkIndex, int frictionAnchor);


    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///bodyUniqueId: int
    ///linkIndex: int
    ///ccdSweptSphereRadius: double
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3ChangeDynamicsInfoSetCcdSweptSphereRadius")]
    public static extern int b3ChangeDynamicsInfoSetCcdSweptSphereRadius(IntPtr commandHandle, int bodyUniqueId, int linkIndex, double ccdSweptSphereRadius);


    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///bodyUniqueId: int
    ///linkIndex: int
    ///contactProcessingThreshold: double
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3ChangeDynamicsInfoSetContactProcessingThreshold")]
    public static extern int b3ChangeDynamicsInfoSetContactProcessingThreshold(IntPtr commandHandle, int bodyUniqueId, int linkIndex, double contactProcessingThreshold);


    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///bodyUniqueId: int
    ///activationState: int
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3ChangeDynamicsInfoSetActivationState")]
    public static extern int b3ChangeDynamicsInfoSetActivationState(IntPtr commandHandle, int bodyUniqueId, int activationState);


    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///bodyUniqueId: int
    ///maxJointVelocity: double
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3ChangeDynamicsInfoSetMaxJointVelocity")]
    public static extern int b3ChangeDynamicsInfoSetMaxJointVelocity(IntPtr commandHandle, int bodyUniqueId, double maxJointVelocity);


    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///bodyUniqueId: int
    ///collisionMargin: double
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3ChangeDynamicsInfoSetCollisionMargin")]
    public static extern int b3ChangeDynamicsInfoSetCollisionMargin(IntPtr commandHandle, int bodyUniqueId, double collisionMargin);


    /// Return Type: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    ///parentBodyUniqueId: int
    ///parentJointIndex: int
    ///childBodyUniqueId: int
    ///childJointIndex: int
    ///info: b3JointInfo*
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3InitCreateUserConstraintCommand")]
    public static extern System.IntPtr b3InitCreateUserConstraintCommand(IntPtr physClient, int parentBodyUniqueId, int parentJointIndex, int childBodyUniqueId, int childJointIndex, ref b3JointInfo info);


    /// Return Type: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///parentBodyUniqueId: int
    ///parentJointIndex: int
    ///childBodyUniqueId: int
    ///childJointIndex: int
    ///info: b3JointInfo*
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3InitCreateUserConstraintCommand2")]
    public static extern System.IntPtr b3InitCreateUserConstraintCommand2(IntPtr commandHandle, int parentBodyUniqueId, int parentJointIndex, int childBodyUniqueId, int childJointIndex, ref b3JointInfo info);


    /// Return Type: int
    ///statusHandle: b3SharedMemoryStatusHandle->b3SharedMemoryStatusHandle__*
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3GetStatusUserConstraintUniqueId")]
    public static extern int b3GetStatusUserConstraintUniqueId(IntPtr statusHandle);


    /// Return Type: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    ///userConstraintUniqueId: int
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3InitChangeUserConstraintCommand")]
    public static extern System.IntPtr b3InitChangeUserConstraintCommand(IntPtr physClient, int userConstraintUniqueId);


    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///jointChildPivot: double*
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3InitChangeUserConstraintSetPivotInB")]
    public static extern int b3InitChangeUserConstraintSetPivotInB(IntPtr commandHandle, ref double jointChildPivot);


    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///jointChildFrameOrn: double*
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3InitChangeUserConstraintSetFrameInB")]
    public static extern int b3InitChangeUserConstraintSetFrameInB(IntPtr commandHandle, ref double jointChildFrameOrn);


    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///maxAppliedForce: double
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3InitChangeUserConstraintSetMaxForce")]
    public static extern int b3InitChangeUserConstraintSetMaxForce(IntPtr commandHandle, double maxAppliedForce);


    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///gearRatio: double
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3InitChangeUserConstraintSetGearRatio")]
    public static extern int b3InitChangeUserConstraintSetGearRatio(IntPtr commandHandle, double gearRatio);


    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///gearAuxLink: int
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3InitChangeUserConstraintSetGearAuxLink")]
    public static extern int b3InitChangeUserConstraintSetGearAuxLink(IntPtr commandHandle, int gearAuxLink);


    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///relativePositionTarget: double
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3InitChangeUserConstraintSetRelativePositionTarget")]
    public static extern int b3InitChangeUserConstraintSetRelativePositionTarget(IntPtr commandHandle, double relativePositionTarget);


    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///erp: double
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3InitChangeUserConstraintSetERP")]
    public static extern int b3InitChangeUserConstraintSetERP(IntPtr commandHandle, double erp);


    /// Return Type: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    ///userConstraintUniqueId: int
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3InitRemoveUserConstraintCommand")]
    public static extern System.IntPtr b3InitRemoveUserConstraintCommand(IntPtr physClient, int userConstraintUniqueId);


    /// Return Type: int
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3GetNumUserConstraints")]
    public static extern int b3GetNumUserConstraints(IntPtr physClient);


    /// Return Type: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    ///constraintUniqueId: int
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3InitGetUserConstraintStateCommand")]
    public static extern System.IntPtr b3InitGetUserConstraintStateCommand(IntPtr physClient, int constraintUniqueId);


    /// Return Type: int
    ///statusHandle: b3SharedMemoryStatusHandle->b3SharedMemoryStatusHandle__*
    ///constraintState: b3UserConstraintState*
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3GetStatusUserConstraintState")]
    public static extern int b3GetStatusUserConstraintState(IntPtr statusHandle, ref b3UserConstraintState constraintState);


    /// Return Type: int
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    ///constraintUniqueId: int
    ///info: b3UserConstraint*
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3GetUserConstraintInfo")]
    public static extern int b3GetUserConstraintInfo(IntPtr physClient, int constraintUniqueId, ref b3UserConstraint info);


    /// Return Type: int
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    ///serialIndex: int
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3GetUserConstraintId")]
    public static extern int b3GetUserConstraintId(IntPtr physClient, int serialIndex);


    /// Return Type: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    ///debugMode: int
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3InitRequestDebugLinesCommand")]
    public static extern System.IntPtr b3InitRequestDebugLinesCommand(IntPtr physClient, int debugMode);


    /// Return Type: void
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    ///lines: b3DebugLines*
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3GetDebugLines")]
    public static extern void b3GetDebugLines(IntPtr physClient, ref b3DebugLines lines);


    /// Return Type: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3InitConfigureOpenGLVisualizer")]
    public static extern System.IntPtr b3InitConfigureOpenGLVisualizer(IntPtr physClient);


    /// Return Type: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3InitConfigureOpenGLVisualizer2")]
    public static extern System.IntPtr b3InitConfigureOpenGLVisualizer2(IntPtr commandHandle);


    /// Return Type: void
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///flag: int
    ///enabled: int
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3ConfigureOpenGLVisualizerSetVisualizationFlags")]
    public static extern void b3ConfigureOpenGLVisualizerSetVisualizationFlags(IntPtr commandHandle, int flag, int enabled);


    /// Return Type: void
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///shadowMapResolution: int
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3ConfigureOpenGLVisualizerSetShadowMapResolution")]
    public static extern void b3ConfigureOpenGLVisualizerSetShadowMapResolution(IntPtr commandHandle, int shadowMapResolution);


    /// Return Type: void
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///shadowMapWorldSize: int
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3ConfigureOpenGLVisualizerSetShadowMapWorldSize")]
    public static extern void b3ConfigureOpenGLVisualizerSetShadowMapWorldSize(IntPtr commandHandle, int shadowMapWorldSize);


    /// Return Type: void
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///remoteSyncTransformInterval: double
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3ConfigureOpenGLVisualizerSetRemoteSyncTransformInterval")]
    public static extern void b3ConfigureOpenGLVisualizerSetRemoteSyncTransformInterval(IntPtr commandHandle, double remoteSyncTransformInterval);


    /// Return Type: void
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///cameraDistance: float
    ///cameraPitch: float
    ///cameraYaw: float
    ///cameraTargetPosition: float*
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3ConfigureOpenGLVisualizerSetViewMatrix")]
    public static extern void b3ConfigureOpenGLVisualizerSetViewMatrix(IntPtr commandHandle, float cameraDistance, float cameraPitch, float cameraYaw, ref float cameraTargetPosition);


    /// Return Type: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3InitRequestOpenGLVisualizerCameraCommand")]
    public static extern System.IntPtr b3InitRequestOpenGLVisualizerCameraCommand(IntPtr physClient);


    /// Return Type: int
    ///statusHandle: b3SharedMemoryStatusHandle->b3SharedMemoryStatusHandle__*
    ///camera: b3OpenGLVisualizerCameraInfo*
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3GetStatusOpenGLVisualizerCamera")]
    public static extern int b3GetStatusOpenGLVisualizerCamera(IntPtr statusHandle, ref b3OpenGLVisualizerCameraInfo camera);


    /// Return Type: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    ///fromXYZ: double*
    ///toXYZ: double*
    ///colorRGB: double*
    ///lineWidth: double
    ///lifeTime: double
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3InitUserDebugDrawAddLine3D")]
    public static extern System.IntPtr b3InitUserDebugDrawAddLine3D(IntPtr physClient, ref double fromXYZ, ref double toXYZ, ref double colorRGB, double lineWidth, double lifeTime);


    /// Return Type: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    ///txt: char*
    ///positionXYZ: double*
    ///colorRGB: double*
    ///textSize: double
    ///lifeTime: double
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3InitUserDebugDrawAddText3D")]
    public static extern System.IntPtr b3InitUserDebugDrawAddText3D(IntPtr physClient, [System.Runtime.InteropServices.InAttribute()] [System.Runtime.InteropServices.MarshalAsAttribute(System.Runtime.InteropServices.UnmanagedType.LPStr)] string txt, ref double positionXYZ, ref double colorRGB, double textSize, double lifeTime);


    /// Return Type: void
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///optionFlags: int
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3UserDebugTextSetOptionFlags")]
    public static extern void b3UserDebugTextSetOptionFlags(IntPtr commandHandle, int optionFlags);


    /// Return Type: void
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///orientation: double*
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3UserDebugTextSetOrientation")]
    public static extern void b3UserDebugTextSetOrientation(IntPtr commandHandle, ref double orientation);


    /// Return Type: void
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///replaceItem: int
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3UserDebugItemSetReplaceItemUniqueId")]
    public static extern void b3UserDebugItemSetReplaceItemUniqueId(IntPtr commandHandle, int replaceItem);


    /// Return Type: void
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///objectUniqueId: int
    ///linkIndex: int
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3UserDebugItemSetParentObject")]
    public static extern void b3UserDebugItemSetParentObject(IntPtr commandHandle, int objectUniqueId, int linkIndex);


    /// Return Type: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    ///txt: char*
    ///rangeMin: double
    ///rangeMax: double
    ///startValue: double
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3InitUserDebugAddParameter")]
    public static extern System.IntPtr b3InitUserDebugAddParameter(IntPtr physClient, [System.Runtime.InteropServices.InAttribute()] [System.Runtime.InteropServices.MarshalAsAttribute(System.Runtime.InteropServices.UnmanagedType.LPStr)] string txt, double rangeMin, double rangeMax, double startValue);


    /// Return Type: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    ///debugItemUniqueId: int
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3InitUserDebugReadParameter")]
    public static extern System.IntPtr b3InitUserDebugReadParameter(IntPtr physClient, int debugItemUniqueId);


    /// Return Type: int
    ///statusHandle: b3SharedMemoryStatusHandle->b3SharedMemoryStatusHandle__*
    ///paramValue: double*
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3GetStatusDebugParameterValue")]
    public static extern int b3GetStatusDebugParameterValue(IntPtr statusHandle, ref double paramValue);


    /// Return Type: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    ///debugItemUniqueId: int
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3InitUserDebugDrawRemove")]
    public static extern System.IntPtr b3InitUserDebugDrawRemove(IntPtr physClient, int debugItemUniqueId);


    /// Return Type: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3InitUserDebugDrawRemoveAll")]
    public static extern System.IntPtr b3InitUserDebugDrawRemoveAll(IntPtr physClient);


    /// Return Type: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3InitUserRemoveAllParameters")]
    public static extern System.IntPtr b3InitUserRemoveAllParameters(IntPtr physClient);


    /// Return Type: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3InitDebugDrawingCommand")]
    public static extern System.IntPtr b3InitDebugDrawingCommand(IntPtr physClient);


    /// Return Type: void
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///objectUniqueId: int
    ///linkIndex: int
    ///objectColorRGB: double*
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3SetDebugObjectColor")]
    public static extern void b3SetDebugObjectColor(IntPtr commandHandle, int objectUniqueId, int linkIndex, ref double objectColorRGB);


    /// Return Type: void
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///objectUniqueId: int
    ///linkIndex: int
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3RemoveDebugObjectColor")]
    public static extern void b3RemoveDebugObjectColor(IntPtr commandHandle, int objectUniqueId, int linkIndex);


    /// Return Type: int
    ///statusHandle: b3SharedMemoryStatusHandle->b3SharedMemoryStatusHandle__*
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3GetDebugItemUniqueId")]
    public static extern int b3GetDebugItemUniqueId(IntPtr statusHandle);


    /// Return Type: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3InitRequestCameraImage")]
    public static extern System.IntPtr b3InitRequestCameraImage(IntPtr physClient);


    /// Return Type: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3InitRequestCameraImage2")]
    public static extern System.IntPtr b3InitRequestCameraImage2(IntPtr commandHandle);


    /// Return Type: void
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///viewMatrix: float*
    ///projectionMatrix: float*
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3RequestCameraImageSetCameraMatrices")]
    public static extern void b3RequestCameraImageSetCameraMatrices(IntPtr commandHandle, ref float viewMatrix, ref float projectionMatrix);


    /// Return Type: void
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///width: int
    ///height: int
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3RequestCameraImageSetPixelResolution")]
    public static extern void b3RequestCameraImageSetPixelResolution(IntPtr commandHandle, int width, int height);


    /// Return Type: void
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///lightDirection: float*
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3RequestCameraImageSetLightDirection")]
    public static extern void b3RequestCameraImageSetLightDirection(IntPtr commandHandle, ref float lightDirection);


    /// Return Type: void
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///lightColor: float*
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3RequestCameraImageSetLightColor")]
    public static extern void b3RequestCameraImageSetLightColor(IntPtr commandHandle, ref float lightColor);


    /// Return Type: void
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///lightDistance: float
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3RequestCameraImageSetLightDistance")]
    public static extern void b3RequestCameraImageSetLightDistance(IntPtr commandHandle, float lightDistance);


    /// Return Type: void
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///lightAmbientCoeff: float
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3RequestCameraImageSetLightAmbientCoeff")]
    public static extern void b3RequestCameraImageSetLightAmbientCoeff(IntPtr commandHandle, float lightAmbientCoeff);


    /// Return Type: void
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///lightDiffuseCoeff: float
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3RequestCameraImageSetLightDiffuseCoeff")]
    public static extern void b3RequestCameraImageSetLightDiffuseCoeff(IntPtr commandHandle, float lightDiffuseCoeff);


    /// Return Type: void
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///lightSpecularCoeff: float
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3RequestCameraImageSetLightSpecularCoeff")]
    public static extern void b3RequestCameraImageSetLightSpecularCoeff(IntPtr commandHandle, float lightSpecularCoeff);


    /// Return Type: void
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///hasShadow: int
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3RequestCameraImageSetShadow")]
    public static extern void b3RequestCameraImageSetShadow(IntPtr commandHandle, int hasShadow);


    /// Return Type: void
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///renderer: int
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3RequestCameraImageSelectRenderer")]
    public static extern void b3RequestCameraImageSelectRenderer(IntPtr commandHandle, int renderer);


    /// Return Type: void
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///flags: int
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3RequestCameraImageSetFlags")]
    public static extern void b3RequestCameraImageSetFlags(IntPtr commandHandle, int flags);


    /// Return Type: void
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    ///imageData: b3CameraImageData*
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3GetCameraImageData")]
    public static extern void b3GetCameraImageData(IntPtr physClient, ref b3CameraImageData imageData);


    /// Return Type: void
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///viewMatrix: float*
    ///projectionMatrix: float*
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3RequestCameraImageSetProjectiveTextureMatrices")]
    public static extern void b3RequestCameraImageSetProjectiveTextureMatrices(IntPtr commandHandle, ref float viewMatrix, ref float projectionMatrix);


    /// Return Type: void
    ///cameraPosition: float*
    ///cameraTargetPosition: float*
    ///cameraUp: float*
    ///viewMatrix: float*
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3ComputeViewMatrixFromPositions")]
    public static extern void b3ComputeViewMatrixFromPositions(ref float cameraPosition, ref float cameraTargetPosition, ref float cameraUp, ref float viewMatrix);


    /// Return Type: void
    ///cameraTargetPosition: float*
    ///distance: float
    ///yaw: float
    ///pitch: float
    ///roll: float
    ///upAxis: int
    ///viewMatrix: float*
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3ComputeViewMatrixFromYawPitchRoll")]
    public static extern void b3ComputeViewMatrixFromYawPitchRoll(ref float cameraTargetPosition, float distance, float yaw, float pitch, float roll, int upAxis, ref float viewMatrix);


    /// Return Type: void
    ///viewMatrix: float*
    ///cameraPosition: float*
    ///cameraTargetPosition: float*
    ///cameraUp: float*
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3ComputePositionFromViewMatrix")]
    public static extern void b3ComputePositionFromViewMatrix(ref float viewMatrix, ref float cameraPosition, ref float cameraTargetPosition, ref float cameraUp);


    /// Return Type: void
    ///left: float
    ///right: float
    ///bottom: float
    ///top: float
    ///nearVal: float
    ///farVal: float
    ///projectionMatrix: float*
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3ComputeProjectionMatrix")]
    public static extern void b3ComputeProjectionMatrix(float left, float right, float bottom, float top, float nearVal, float farVal, ref float projectionMatrix);


    /// Return Type: void
    ///fov: float
    ///aspect: float
    ///nearVal: float
    ///farVal: float
    ///projectionMatrix: float*
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3ComputeProjectionMatrixFOV")]
    public static extern void b3ComputeProjectionMatrixFOV(float fov, float aspect, float nearVal, float farVal, ref float projectionMatrix);


    /// Return Type: void
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///cameraPosition: float*
    ///cameraTargetPosition: float*
    ///cameraUp: float*
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3RequestCameraImageSetViewMatrix")]
    public static extern void b3RequestCameraImageSetViewMatrix(IntPtr commandHandle, ref float cameraPosition, ref float cameraTargetPosition, ref float cameraUp);


    /// Return Type: void
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///cameraTargetPosition: float*
    ///distance: float
    ///yaw: float
    ///pitch: float
    ///roll: float
    ///upAxis: int
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3RequestCameraImageSetViewMatrix2")]
    public static extern void b3RequestCameraImageSetViewMatrix2(IntPtr commandHandle, ref float cameraTargetPosition, float distance, float yaw, float pitch, float roll, int upAxis);


    /// Return Type: void
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///left: float
    ///right: float
    ///bottom: float
    ///top: float
    ///nearVal: float
    ///farVal: float
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3RequestCameraImageSetProjectionMatrix")]
    public static extern void b3RequestCameraImageSetProjectionMatrix(IntPtr commandHandle, float left, float right, float bottom, float top, float nearVal, float farVal);


    /// Return Type: void
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///fov: float
    ///aspect: float
    ///nearVal: float
    ///farVal: float
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3RequestCameraImageSetFOVProjectionMatrix")]
    public static extern void b3RequestCameraImageSetFOVProjectionMatrix(IntPtr commandHandle, float fov, float aspect, float nearVal, float farVal);


    /// Return Type: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3InitRequestContactPointInformation")]
    public static extern System.IntPtr b3InitRequestContactPointInformation(IntPtr physClient);


    /// Return Type: void
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///bodyUniqueIdA: int
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3SetContactFilterBodyA")]
    public static extern void b3SetContactFilterBodyA(IntPtr commandHandle, int bodyUniqueIdA);


    /// Return Type: void
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///bodyUniqueIdB: int
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3SetContactFilterBodyB")]
    public static extern void b3SetContactFilterBodyB(IntPtr commandHandle, int bodyUniqueIdB);


    /// Return Type: void
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///linkIndexA: int
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3SetContactFilterLinkA")]
    public static extern void b3SetContactFilterLinkA(IntPtr commandHandle, int linkIndexA);


    /// Return Type: void
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///linkIndexB: int
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3SetContactFilterLinkB")]
    public static extern void b3SetContactFilterLinkB(IntPtr commandHandle, int linkIndexB);


    /// Return Type: void
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    ///contactPointData: b3ContactInformation*
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3GetContactPointInformation")]
    public static extern void b3GetContactPointInformation(IntPtr physClient, ref b3ContactInformation contactPointData);


    /// Return Type: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3InitClosestDistanceQuery")]
    public static extern System.IntPtr b3InitClosestDistanceQuery(IntPtr physClient);


    /// Return Type: void
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///bodyUniqueIdA: int
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3SetClosestDistanceFilterBodyA")]
    public static extern void b3SetClosestDistanceFilterBodyA(IntPtr commandHandle, int bodyUniqueIdA);


    /// Return Type: void
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///linkIndexA: int
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3SetClosestDistanceFilterLinkA")]
    public static extern void b3SetClosestDistanceFilterLinkA(IntPtr commandHandle, int linkIndexA);


    /// Return Type: void
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///bodyUniqueIdB: int
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3SetClosestDistanceFilterBodyB")]
    public static extern void b3SetClosestDistanceFilterBodyB(IntPtr commandHandle, int bodyUniqueIdB);


    /// Return Type: void
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///linkIndexB: int
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3SetClosestDistanceFilterLinkB")]
    public static extern void b3SetClosestDistanceFilterLinkB(IntPtr commandHandle, int linkIndexB);


    /// Return Type: void
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///distance: double
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3SetClosestDistanceThreshold")]
    public static extern void b3SetClosestDistanceThreshold(IntPtr commandHandle, double distance);


    /// Return Type: void
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///collisionShapeA: int
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3SetClosestDistanceFilterCollisionShapeA")]
    public static extern void b3SetClosestDistanceFilterCollisionShapeA(IntPtr commandHandle, int collisionShapeA);


    /// Return Type: void
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///collisionShapeB: int
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3SetClosestDistanceFilterCollisionShapeB")]
    public static extern void b3SetClosestDistanceFilterCollisionShapeB(IntPtr commandHandle, int collisionShapeB);


    /// Return Type: void
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///collisionShapePositionA: double*
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3SetClosestDistanceFilterCollisionShapePositionA")]
    public static extern void b3SetClosestDistanceFilterCollisionShapePositionA(IntPtr commandHandle, ref double collisionShapePositionA);


    /// Return Type: void
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///collisionShapePositionB: double*
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3SetClosestDistanceFilterCollisionShapePositionB")]
    public static extern void b3SetClosestDistanceFilterCollisionShapePositionB(IntPtr commandHandle, ref double collisionShapePositionB);


    /// Return Type: void
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///collisionShapeOrientationA: double*
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3SetClosestDistanceFilterCollisionShapeOrientationA")]
    public static extern void b3SetClosestDistanceFilterCollisionShapeOrientationA(IntPtr commandHandle, ref double collisionShapeOrientationA);


    /// Return Type: void
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///collisionShapeOrientationB: double*
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3SetClosestDistanceFilterCollisionShapeOrientationB")]
    public static extern void b3SetClosestDistanceFilterCollisionShapeOrientationB(IntPtr commandHandle, ref double collisionShapeOrientationB);


    /// Return Type: void
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    ///contactPointInfo: b3ContactInformation*
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3GetClosestPointInformation")]
    public static extern void b3GetClosestPointInformation(IntPtr physClient, ref b3ContactInformation contactPointInfo);


    /// Return Type: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    ///aabbMin: double*
    ///aabbMax: double*
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3InitAABBOverlapQuery")]
    public static extern System.IntPtr b3InitAABBOverlapQuery(IntPtr physClient, ref double aabbMin, ref double aabbMax);


    /// Return Type: void
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    ///data: b3AABBOverlapData*
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3GetAABBOverlapResults")]
    public static extern void b3GetAABBOverlapResults(IntPtr physClient, ref b3AABBOverlapData data);


    /// Return Type: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    ///bodyUniqueIdA: int
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3InitRequestVisualShapeInformation")]
    public static extern System.IntPtr b3InitRequestVisualShapeInformation(IntPtr physClient, int bodyUniqueIdA);


    /// Return Type: void
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    ///visualShapeInfo: b3VisualShapeInformation*
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3GetVisualShapeInformation")]
    public static extern void b3GetVisualShapeInformation(IntPtr physClient, ref b3VisualShapeInformation visualShapeInfo);


    /// Return Type: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    ///bodyUniqueId: int
    ///linkIndex: int
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3InitRequestCollisionShapeInformation")]
    public static extern System.IntPtr b3InitRequestCollisionShapeInformation(IntPtr physClient, int bodyUniqueId, int linkIndex);


    /// Return Type: void
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    ///collisionShapeInfo: b3CollisionShapeInformation*
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3GetCollisionShapeInformation")]
    public static extern void b3GetCollisionShapeInformation(IntPtr physClient, ref b3CollisionShapeInformation collisionShapeInfo);


    /// Return Type: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    ///filename: char*
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3InitLoadTexture")]
    public static extern System.IntPtr b3InitLoadTexture(IntPtr physClient, [System.Runtime.InteropServices.InAttribute()] [System.Runtime.InteropServices.MarshalAsAttribute(System.Runtime.InteropServices.UnmanagedType.LPStr)] string filename);


    /// Return Type: int
    ///statusHandle: b3SharedMemoryStatusHandle->b3SharedMemoryStatusHandle__*
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3GetStatusTextureUniqueId")]
    public static extern int b3GetStatusTextureUniqueId(IntPtr statusHandle);


    /// Return Type: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    ///textureUniqueId: int
    ///width: int
    ///height: int
    ///rgbPixels: char*
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3CreateChangeTextureCommandInit")]
    public static extern System.IntPtr b3CreateChangeTextureCommandInit(IntPtr physClient, int textureUniqueId, int width, int height, [System.Runtime.InteropServices.InAttribute()] [System.Runtime.InteropServices.MarshalAsAttribute(System.Runtime.InteropServices.UnmanagedType.LPStr)] string rgbPixels);


    /// Return Type: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    ///bodyUniqueId: int
    ///jointIndex: int
    ///shapeIndex: int
    ///textureUniqueId: int
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3InitUpdateVisualShape")]
    public static extern System.IntPtr b3InitUpdateVisualShape(IntPtr physClient, int bodyUniqueId, int jointIndex, int shapeIndex, int textureUniqueId);


    /// Return Type: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    ///bodyUniqueId: int
    ///jointIndex: int
    ///shapeIndex: int
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3InitUpdateVisualShape2")]
    public static extern System.IntPtr b3InitUpdateVisualShape2(IntPtr physClient, int bodyUniqueId, int jointIndex, int shapeIndex);


    /// Return Type: void
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///textureUniqueId: int
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3UpdateVisualShapeTexture")]
    public static extern void b3UpdateVisualShapeTexture(IntPtr commandHandle, int textureUniqueId);


    /// Return Type: void
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///rgbaColor: double*
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3UpdateVisualShapeRGBAColor")]
    public static extern void b3UpdateVisualShapeRGBAColor(IntPtr commandHandle, ref double rgbaColor);


    /// Return Type: void
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///specularColor: double*
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3UpdateVisualShapeSpecularColor")]
    public static extern void b3UpdateVisualShapeSpecularColor(IntPtr commandHandle, ref double specularColor);


    /// Return Type: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3InitPhysicsParamCommand")]
    public static extern System.IntPtr b3InitPhysicsParamCommand(IntPtr physClient);


    /// Return Type: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3InitPhysicsParamCommand2")]
    public static extern System.IntPtr b3InitPhysicsParamCommand2(IntPtr commandHandle);


    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///gravx: double
    ///gravy: double
    ///gravz: double
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3PhysicsParamSetGravity")]
    public static extern int b3PhysicsParamSetGravity(IntPtr commandHandle, double gravx, double gravy, double gravz);


    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///timeStep: double
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3PhysicsParamSetTimeStep")]
    public static extern int b3PhysicsParamSetTimeStep(IntPtr commandHandle, double timeStep);


    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///defaultContactERP: double
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3PhysicsParamSetDefaultContactERP")]
    public static extern int b3PhysicsParamSetDefaultContactERP(IntPtr commandHandle, double defaultContactERP);


    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///defaultNonContactERP: double
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3PhysicsParamSetDefaultNonContactERP")]
    public static extern int b3PhysicsParamSetDefaultNonContactERP(IntPtr commandHandle, double defaultNonContactERP);


    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///frictionERP: double
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3PhysicsParamSetDefaultFrictionERP")]
    public static extern int b3PhysicsParamSetDefaultFrictionERP(IntPtr commandHandle, double frictionERP);


    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///defaultGlobalCFM: double
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3PhysicsParamSetDefaultGlobalCFM")]
    public static extern int b3PhysicsParamSetDefaultGlobalCFM(IntPtr commandHandle, double defaultGlobalCFM);


    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///frictionCFM: double
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3PhysicsParamSetDefaultFrictionCFM")]
    public static extern int b3PhysicsParamSetDefaultFrictionCFM(IntPtr commandHandle, double frictionCFM);


    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///numSubSteps: int
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3PhysicsParamSetNumSubSteps")]
    public static extern int b3PhysicsParamSetNumSubSteps(IntPtr commandHandle, int numSubSteps);


    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///enableRealTimeSimulation: int
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3PhysicsParamSetRealTimeSimulation")]
    public static extern int b3PhysicsParamSetRealTimeSimulation(IntPtr commandHandle, int enableRealTimeSimulation);


    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///numSolverIterations: int
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3PhysicsParamSetNumSolverIterations")]
    public static extern int b3PhysicsParamSetNumSolverIterations(IntPtr commandHandle, int numSolverIterations);


    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///numMotorIterations: int
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3PhysicsParamSetNumNonContactInnerIterations")]
    public static extern int b3PhysicsParamSetNumNonContactInnerIterations(IntPtr commandHandle, int numMotorIterations);


    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///warmStartingFactor: double
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3PhysicsParamSetWarmStartingFactor")]
    public static extern int b3PhysicsParamSetWarmStartingFactor(IntPtr commandHandle, double warmStartingFactor);


    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///warmStartingFactor: double
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3PhysicsParamSetArticulatedWarmStartingFactor")]
    public static extern int b3PhysicsParamSetArticulatedWarmStartingFactor(IntPtr commandHandle, double warmStartingFactor);


    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///filterMode: int
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3PhysicsParamSetCollisionFilterMode")]
    public static extern int b3PhysicsParamSetCollisionFilterMode(IntPtr commandHandle, int filterMode);


    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///useSplitImpulse: int
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3PhysicsParamSetUseSplitImpulse")]
    public static extern int b3PhysicsParamSetUseSplitImpulse(IntPtr commandHandle, int useSplitImpulse);


    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///splitImpulsePenetrationThreshold: double
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3PhysicsParamSetSplitImpulsePenetrationThreshold")]
    public static extern int b3PhysicsParamSetSplitImpulsePenetrationThreshold(IntPtr commandHandle, double splitImpulsePenetrationThreshold);


    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///contactBreakingThreshold: double
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3PhysicsParamSetContactBreakingThreshold")]
    public static extern int b3PhysicsParamSetContactBreakingThreshold(IntPtr commandHandle, double contactBreakingThreshold);


    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///maxNumCmdPer1ms: int
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3PhysicsParamSetMaxNumCommandsPer1ms")]
    public static extern int b3PhysicsParamSetMaxNumCommandsPer1ms(IntPtr commandHandle, int maxNumCmdPer1ms);


    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///enableFileCaching: int
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3PhysicsParamSetEnableFileCaching")]
    public static extern int b3PhysicsParamSetEnableFileCaching(IntPtr commandHandle, int enableFileCaching);


    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///restitutionVelocityThreshold: double
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3PhysicsParamSetRestitutionVelocityThreshold")]
    public static extern int b3PhysicsParamSetRestitutionVelocityThreshold(IntPtr commandHandle, double restitutionVelocityThreshold);


    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///enableConeFriction: int
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3PhysicsParamSetEnableConeFriction")]
    public static extern int b3PhysicsParamSetEnableConeFriction(IntPtr commandHandle, int enableConeFriction);


    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///deterministicOverlappingPairs: int
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3PhysicsParameterSetDeterministicOverlappingPairs")]
    public static extern int b3PhysicsParameterSetDeterministicOverlappingPairs(IntPtr commandHandle, int deterministicOverlappingPairs);


    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///allowedCcdPenetration: double
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3PhysicsParameterSetAllowedCcdPenetration")]
    public static extern int b3PhysicsParameterSetAllowedCcdPenetration(IntPtr commandHandle, double allowedCcdPenetration);


    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///jointFeedbackMode: int
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3PhysicsParameterSetJointFeedbackMode")]
    public static extern int b3PhysicsParameterSetJointFeedbackMode(IntPtr commandHandle, int jointFeedbackMode);


    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///solverResidualThreshold: double
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3PhysicsParamSetSolverResidualThreshold")]
    public static extern int b3PhysicsParamSetSolverResidualThreshold(IntPtr commandHandle, double solverResidualThreshold);


    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///contactSlop: double
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3PhysicsParamSetContactSlop")]
    public static extern int b3PhysicsParamSetContactSlop(IntPtr commandHandle, double contactSlop);


    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///enableSAT: int
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3PhysicsParameterSetEnableSAT")]
    public static extern int b3PhysicsParameterSetEnableSAT(IntPtr commandHandle, int enableSAT);


    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///constraintSolverType: int
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3PhysicsParameterSetConstraintSolverType")]
    public static extern int b3PhysicsParameterSetConstraintSolverType(IntPtr commandHandle, int constraintSolverType);


    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///minimumSolverIslandSize: int
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3PhysicsParameterSetMinimumSolverIslandSize")]
    public static extern int b3PhysicsParameterSetMinimumSolverIslandSize(IntPtr commandHandle, int minimumSolverIslandSize);


    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///reportSolverAnalytics: int
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3PhysicsParamSetSolverAnalytics")]
    public static extern int b3PhysicsParamSetSolverAnalytics(IntPtr commandHandle, int reportSolverAnalytics);


    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///sparseSdfVoxelSize: double
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3PhysicsParameterSetSparseSdfVoxelSize")]
    public static extern int b3PhysicsParameterSetSparseSdfVoxelSize(IntPtr commandHandle, double sparseSdfVoxelSize);


    /// Return Type: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3InitRequestPhysicsParamCommand")]
    public static extern System.IntPtr b3InitRequestPhysicsParamCommand(IntPtr physClient);


    /// Return Type: int
    ///statusHandle: b3SharedMemoryStatusHandle->b3SharedMemoryStatusHandle__*
    ///params: b3PhysicsSimulationParameters*
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3GetStatusPhysicsSimulationParameters")]
    public static extern int b3GetStatusPhysicsSimulationParameters(IntPtr statusHandle, ref b3PhysicsSimulationParameters @params);


    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///flags: int
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3PhysicsParamSetInternalSimFlags")]
    public static extern int b3PhysicsParamSetInternalSimFlags(IntPtr commandHandle, int flags);


    /// Return Type: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3InitStepSimulationCommand")]
    public static extern System.IntPtr b3InitStepSimulationCommand(IntPtr physClient);


    /// Return Type: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3InitStepSimulationCommand2")]
    public static extern System.IntPtr b3InitStepSimulationCommand2(IntPtr commandHandle);


    /// Return Type: int
    ///statusHandle: b3SharedMemoryStatusHandle->b3SharedMemoryStatusHandle__*
    ///analyticsData: b3ForwardDynamicsAnalyticsArgs*
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3GetStatusForwardDynamicsAnalyticsData")]
    public static extern int b3GetStatusForwardDynamicsAnalyticsData(IntPtr statusHandle, ref b3ForwardDynamicsAnalyticsArgs analyticsData);


    /// Return Type: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3InitResetSimulationCommand")]
    public static extern System.IntPtr b3InitResetSimulationCommand(IntPtr physClient);


    /// Return Type: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3InitResetSimulationCommand2")]
    public static extern System.IntPtr b3InitResetSimulationCommand2(IntPtr commandHandle);


    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///flags: int
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3InitResetSimulationSetFlags")]
    public static extern int b3InitResetSimulationSetFlags(IntPtr commandHandle, int flags);


    /// Return Type: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    ///urdfFileName: char*
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3LoadUrdfCommandInit")]
    public static extern System.IntPtr b3LoadUrdfCommandInit(IntPtr physClient, [System.Runtime.InteropServices.InAttribute()] [System.Runtime.InteropServices.MarshalAsAttribute(System.Runtime.InteropServices.UnmanagedType.LPStr)] string urdfFileName);


    /// Return Type: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///urdfFileName: char*
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3LoadUrdfCommandInit2")]
    public static extern System.IntPtr b3LoadUrdfCommandInit2(IntPtr commandHandle, [System.Runtime.InteropServices.InAttribute()] [System.Runtime.InteropServices.MarshalAsAttribute(System.Runtime.InteropServices.UnmanagedType.LPStr)] string urdfFileName);


    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///startPosX: double
    ///startPosY: double
    ///startPosZ: double
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3LoadUrdfCommandSetStartPosition")]
    public static extern int b3LoadUrdfCommandSetStartPosition(IntPtr commandHandle, double startPosX, double startPosY, double startPosZ);


    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///startOrnX: double
    ///startOrnY: double
    ///startOrnZ: double
    ///startOrnW: double
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3LoadUrdfCommandSetStartOrientation")]
    public static extern int b3LoadUrdfCommandSetStartOrientation(IntPtr commandHandle, double startOrnX, double startOrnY, double startOrnZ, double startOrnW);


    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///useMultiBody: int
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3LoadUrdfCommandSetUseMultiBody")]
    public static extern int b3LoadUrdfCommandSetUseMultiBody(IntPtr commandHandle, int useMultiBody);


    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///useFixedBase: int
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3LoadUrdfCommandSetUseFixedBase")]
    public static extern int b3LoadUrdfCommandSetUseFixedBase(IntPtr commandHandle, int useFixedBase);


    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///flags: int
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3LoadUrdfCommandSetFlags")]
    public static extern int b3LoadUrdfCommandSetFlags(IntPtr commandHandle, int flags);


    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///globalScaling: double
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3LoadUrdfCommandSetGlobalScaling")]
    public static extern int b3LoadUrdfCommandSetGlobalScaling(IntPtr commandHandle, double globalScaling);


    /// Return Type: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3SaveStateCommandInit")]
    public static extern System.IntPtr b3SaveStateCommandInit(IntPtr physClient);


    /// Return Type: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    ///stateId: int
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3InitRemoveStateCommand")]
    public static extern System.IntPtr b3InitRemoveStateCommand(IntPtr physClient, int stateId);


    /// Return Type: int
    ///statusHandle: b3SharedMemoryStatusHandle->b3SharedMemoryStatusHandle__*
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3GetStatusGetStateId")]
    public static extern int b3GetStatusGetStateId(IntPtr statusHandle);


    /// Return Type: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3LoadStateCommandInit")]
    public static extern System.IntPtr b3LoadStateCommandInit(IntPtr physClient);


    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///stateId: int
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3LoadStateSetStateId")]
    public static extern int b3LoadStateSetStateId(IntPtr commandHandle, int stateId);


    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///fileName: char*
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3LoadStateSetFileName")]
    public static extern int b3LoadStateSetFileName(IntPtr commandHandle, [System.Runtime.InteropServices.InAttribute()] [System.Runtime.InteropServices.MarshalAsAttribute(System.Runtime.InteropServices.UnmanagedType.LPStr)] string fileName);


    /// Return Type: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    ///fileName: char*
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3LoadBulletCommandInit")]
    public static extern System.IntPtr b3LoadBulletCommandInit(IntPtr physClient, [System.Runtime.InteropServices.InAttribute()] [System.Runtime.InteropServices.MarshalAsAttribute(System.Runtime.InteropServices.UnmanagedType.LPStr)] string fileName);


    /// Return Type: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    ///fileName: char*
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3SaveBulletCommandInit")]
    public static extern System.IntPtr b3SaveBulletCommandInit(IntPtr physClient, [System.Runtime.InteropServices.InAttribute()] [System.Runtime.InteropServices.MarshalAsAttribute(System.Runtime.InteropServices.UnmanagedType.LPStr)] string fileName);


    /// Return Type: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    ///fileName: char*
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3LoadMJCFCommandInit")]
    public static extern System.IntPtr b3LoadMJCFCommandInit(IntPtr physClient, [System.Runtime.InteropServices.InAttribute()] [System.Runtime.InteropServices.MarshalAsAttribute(System.Runtime.InteropServices.UnmanagedType.LPStr)] string fileName);


    /// Return Type: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///fileName: char*
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3LoadMJCFCommandInit2")]
    public static extern System.IntPtr b3LoadMJCFCommandInit2(IntPtr commandHandle, [System.Runtime.InteropServices.InAttribute()] [System.Runtime.InteropServices.MarshalAsAttribute(System.Runtime.InteropServices.UnmanagedType.LPStr)] string fileName);


    /// Return Type: void
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///flags: int
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3LoadMJCFCommandSetFlags")]
    public static extern void b3LoadMJCFCommandSetFlags(IntPtr commandHandle, int flags);


    /// Return Type: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    ///bodyUniqueId: int
    ///jointPositionsQ: double*
    ///jointVelocitiesQdot: double*
    ///jointAccelerations: double*
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3CalculateInverseDynamicsCommandInit")]
    public static extern System.IntPtr b3CalculateInverseDynamicsCommandInit(IntPtr physClient, int bodyUniqueId, ref double jointPositionsQ, ref double jointVelocitiesQdot, ref double jointAccelerations);


    /// Return Type: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    ///bodyUniqueId: int
    ///jointPositionsQ: double*
    ///dofCountQ: int
    ///jointVelocitiesQdot: double*
    ///jointAccelerations: double*
    ///dofCountQdot: int
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3CalculateInverseDynamicsCommandInit2")]
    public static extern System.IntPtr b3CalculateInverseDynamicsCommandInit2(IntPtr physClient, int bodyUniqueId, ref double jointPositionsQ, int dofCountQ, ref double jointVelocitiesQdot, ref double jointAccelerations, int dofCountQdot);


    /// Return Type: void
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///flags: int
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3CalculateInverseDynamicsSetFlags")]
    public static extern void b3CalculateInverseDynamicsSetFlags(IntPtr commandHandle, int flags);


    /// Return Type: int
    ///statusHandle: b3SharedMemoryStatusHandle->b3SharedMemoryStatusHandle__*
    ///bodyUniqueId: int*
    ///dofCount: int*
    ///jointForces: double*
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3GetStatusInverseDynamicsJointForces")]
    public static extern int b3GetStatusInverseDynamicsJointForces(IntPtr statusHandle, ref int bodyUniqueId, ref int dofCount, ref double jointForces);


    /// Return Type: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    ///bodyUniqueId: int
    ///linkIndex: int
    ///localPosition: double*
    ///jointPositionsQ: double*
    ///jointVelocitiesQdot: double*
    ///jointAccelerations: double*
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3CalculateJacobianCommandInit")]
    public static extern System.IntPtr b3CalculateJacobianCommandInit(IntPtr physClient, int bodyUniqueId, int linkIndex, ref double localPosition, ref double jointPositionsQ, ref double jointVelocitiesQdot, ref double jointAccelerations);


    /// Return Type: int
    ///statusHandle: b3SharedMemoryStatusHandle->b3SharedMemoryStatusHandle__*
    ///dofCount: int*
    ///linearJacobian: double*
    ///angularJacobian: double*
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3GetStatusJacobian")]
    public static extern int b3GetStatusJacobian(IntPtr statusHandle, ref int dofCount, ref double linearJacobian, ref double angularJacobian);


    /// Return Type: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    ///bodyUniqueId: int
    ///jointPositionsQ: double*
    ///dofCountQ: int
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3CalculateMassMatrixCommandInit")]
    public static extern System.IntPtr b3CalculateMassMatrixCommandInit(IntPtr physClient, int bodyUniqueId, ref double jointPositionsQ, int dofCountQ);


    /// Return Type: void
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///flags: int
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3CalculateMassMatrixSetFlags")]
    public static extern void b3CalculateMassMatrixSetFlags(IntPtr commandHandle, int flags);


    /// Return Type: int
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    ///statusHandle: b3SharedMemoryStatusHandle->b3SharedMemoryStatusHandle__*
    ///dofCount: int*
    ///massMatrix: double*
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3GetStatusMassMatrix")]
    public static extern int b3GetStatusMassMatrix(IntPtr physClient, IntPtr statusHandle, ref int dofCount, ref double massMatrix);


    /// Return Type: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    ///bodyUniqueId: int
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3CalculateInverseKinematicsCommandInit")]
    public static extern System.IntPtr b3CalculateInverseKinematicsCommandInit(IntPtr physClient, int bodyUniqueId);


    /// Return Type: void
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///endEffectorLinkIndex: int
    ///targetPosition: double*
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3CalculateInverseKinematicsAddTargetPurePosition")]
    public static extern void b3CalculateInverseKinematicsAddTargetPurePosition(IntPtr commandHandle, int endEffectorLinkIndex, ref double targetPosition);


    /// Return Type: void
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///numEndEffectorLinkIndices: int
    ///endEffectorIndices: int*
    ///targetPositions: double*
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3CalculateInverseKinematicsAddTargetsPurePosition")]
    public static extern void b3CalculateInverseKinematicsAddTargetsPurePosition(IntPtr commandHandle, int numEndEffectorLinkIndices, ref int endEffectorIndices, ref double targetPositions);


    /// Return Type: void
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///endEffectorLinkIndex: int
    ///targetPosition: double*
    ///targetOrientation: double*
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3CalculateInverseKinematicsAddTargetPositionWithOrientation")]
    public static extern void b3CalculateInverseKinematicsAddTargetPositionWithOrientation(IntPtr commandHandle, int endEffectorLinkIndex, ref double targetPosition, ref double targetOrientation);


    /// Return Type: void
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///numDof: int
    ///endEffectorLinkIndex: int
    ///targetPosition: double*
    ///lowerLimit: double*
    ///upperLimit: double*
    ///jointRange: double*
    ///restPose: double*
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3CalculateInverseKinematicsPosWithNullSpaceVel")]
    public static extern void b3CalculateInverseKinematicsPosWithNullSpaceVel(IntPtr commandHandle, int numDof, int endEffectorLinkIndex, ref double targetPosition, ref double lowerLimit, ref double upperLimit, ref double jointRange, ref double restPose);


    /// Return Type: void
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///numDof: int
    ///endEffectorLinkIndex: int
    ///targetPosition: double*
    ///targetOrientation: double*
    ///lowerLimit: double*
    ///upperLimit: double*
    ///jointRange: double*
    ///restPose: double*
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3CalculateInverseKinematicsPosOrnWithNullSpaceVel")]
    public static extern void b3CalculateInverseKinematicsPosOrnWithNullSpaceVel(IntPtr commandHandle, int numDof, int endEffectorLinkIndex, ref double targetPosition, ref double targetOrientation, ref double lowerLimit, ref double upperLimit, ref double jointRange, ref double restPose);


    /// Return Type: void
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///numDof: int
    ///jointDampingCoeff: double*
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3CalculateInverseKinematicsSetJointDamping")]
    public static extern void b3CalculateInverseKinematicsSetJointDamping(IntPtr commandHandle, int numDof, ref double jointDampingCoeff);


    /// Return Type: void
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///solver: int
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3CalculateInverseKinematicsSelectSolver")]
    public static extern void b3CalculateInverseKinematicsSelectSolver(IntPtr commandHandle, int solver);


    /// Return Type: int
    ///statusHandle: b3SharedMemoryStatusHandle->b3SharedMemoryStatusHandle__*
    ///bodyUniqueId: int*
    ///dofCount: int*
    ///jointPositions: double*
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3GetStatusInverseKinematicsJointPositions")]
    public static extern int b3GetStatusInverseKinematicsJointPositions(IntPtr statusHandle, ref int bodyUniqueId, ref int dofCount, ref double jointPositions);


    /// Return Type: void
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///numDof: int
    ///currentJointPositions: double*
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3CalculateInverseKinematicsSetCurrentPositions")]
    public static extern void b3CalculateInverseKinematicsSetCurrentPositions(IntPtr commandHandle, int numDof, ref double currentJointPositions);


    /// Return Type: void
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///maxNumIterations: int
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3CalculateInverseKinematicsSetMaxNumIterations")]
    public static extern void b3CalculateInverseKinematicsSetMaxNumIterations(IntPtr commandHandle, int maxNumIterations);


    /// Return Type: void
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///residualThreshold: double
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3CalculateInverseKinematicsSetResidualThreshold")]
    public static extern void b3CalculateInverseKinematicsSetResidualThreshold(IntPtr commandHandle, double residualThreshold);


    /// Return Type: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3CollisionFilterCommandInit")]
    public static extern System.IntPtr b3CollisionFilterCommandInit(IntPtr physClient);


    /// Return Type: void
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///bodyUniqueIdA: int
    ///bodyUniqueIdB: int
    ///linkIndexA: int
    ///linkIndexB: int
    ///enableCollision: int
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3SetCollisionFilterPair")]
    public static extern void b3SetCollisionFilterPair(IntPtr commandHandle, int bodyUniqueIdA, int bodyUniqueIdB, int linkIndexA, int linkIndexB, int enableCollision);


    /// Return Type: void
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///bodyUniqueIdA: int
    ///linkIndexA: int
    ///collisionFilterGroup: int
    ///collisionFilterMask: int
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3SetCollisionFilterGroupMask")]
    public static extern void b3SetCollisionFilterGroupMask(IntPtr commandHandle, int bodyUniqueIdA, int linkIndexA, int collisionFilterGroup, int collisionFilterMask);


    /// Return Type: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    ///sdfFileName: char*
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3LoadSdfCommandInit")]
    public static extern System.IntPtr b3LoadSdfCommandInit(IntPtr physClient, [System.Runtime.InteropServices.InAttribute()] [System.Runtime.InteropServices.MarshalAsAttribute(System.Runtime.InteropServices.UnmanagedType.LPStr)] string sdfFileName);


    /// Return Type: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///sdfFileName: char*
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3LoadSdfCommandInit2")]
    public static extern System.IntPtr b3LoadSdfCommandInit2(IntPtr commandHandle, [System.Runtime.InteropServices.InAttribute()] [System.Runtime.InteropServices.MarshalAsAttribute(System.Runtime.InteropServices.UnmanagedType.LPStr)] string sdfFileName);


    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///useMultiBody: int
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3LoadSdfCommandSetUseMultiBody")]
    public static extern int b3LoadSdfCommandSetUseMultiBody(IntPtr commandHandle, int useMultiBody);


    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///globalScaling: double
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3LoadSdfCommandSetUseGlobalScaling")]
    public static extern int b3LoadSdfCommandSetUseGlobalScaling(IntPtr commandHandle, double globalScaling);


    /// Return Type: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    ///sdfFileName: char*
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3SaveWorldCommandInit")]
    public static extern System.IntPtr b3SaveWorldCommandInit(IntPtr physClient, [System.Runtime.InteropServices.InAttribute()] [System.Runtime.InteropServices.MarshalAsAttribute(System.Runtime.InteropServices.UnmanagedType.LPStr)] string sdfFileName);


    /// Return Type: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    ///controlMode: int
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3JointControlCommandInit")]
    public static extern System.IntPtr b3JointControlCommandInit(IntPtr physClient, int controlMode);


    /// Return Type: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    ///bodyUniqueId: int
    ///controlMode: int
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3JointControlCommandInit2")]
    public static extern System.IntPtr b3JointControlCommandInit2(IntPtr physClient, int bodyUniqueId, int controlMode);


    /// Return Type: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///bodyUniqueId: int
    ///controlMode: int
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3JointControlCommandInit2Internal")]
    public static extern System.IntPtr b3JointControlCommandInit2Internal(IntPtr commandHandle, int bodyUniqueId, int controlMode);


    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///qIndex: int
    ///value: double
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3JointControlSetDesiredPosition")]
    public static extern int b3JointControlSetDesiredPosition(IntPtr commandHandle, int qIndex, double value);


    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///qIndex: int
    ///position: double*
    ///dofCount: int
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3JointControlSetDesiredPositionMultiDof")]
    public static extern int b3JointControlSetDesiredPositionMultiDof(IntPtr commandHandle, int qIndex, ref double position, int dofCount);


    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///dofIndex: int
    ///value: double
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3JointControlSetKp")]
    public static extern int b3JointControlSetKp(IntPtr commandHandle, int dofIndex, double value);


    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///dofIndex: int
    ///value: double
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3JointControlSetKd")]
    public static extern int b3JointControlSetKd(IntPtr commandHandle, int dofIndex, double value);


    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///dofIndex: int
    ///maximumVelocity: double
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3JointControlSetMaximumVelocity")]
    public static extern int b3JointControlSetMaximumVelocity(IntPtr commandHandle, int dofIndex, double maximumVelocity);


    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///dofIndex: int
    ///value: double
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3JointControlSetDesiredVelocity")]
    public static extern int b3JointControlSetDesiredVelocity(IntPtr commandHandle, int dofIndex, double value);


    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///dofIndex: int
    ///velocity: double*
    ///dofCount: int
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3JointControlSetDesiredVelocityMultiDof")]
    public static extern int b3JointControlSetDesiredVelocityMultiDof(IntPtr commandHandle, int dofIndex, ref double velocity, int dofCount);


    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///dofIndex: int
    ///velocity: double*
    ///dofCount: int
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3JointControlSetDesiredVelocityMultiDof2")]
    public static extern int b3JointControlSetDesiredVelocityMultiDof2(IntPtr commandHandle, int dofIndex, ref double velocity, int dofCount);


    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///dofIndex: int
    ///value: double
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3JointControlSetMaximumForce")]
    public static extern int b3JointControlSetMaximumForce(IntPtr commandHandle, int dofIndex, double value);


    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///dofIndex: int
    ///forces: double*
    ///dofCount: int
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3JointControlSetDesiredForceTorqueMultiDof")]
    public static extern int b3JointControlSetDesiredForceTorqueMultiDof(IntPtr commandHandle, int dofIndex, ref double forces, int dofCount);


    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///dofIndex: int
    ///value: double
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3JointControlSetDesiredForceTorque")]
    public static extern int b3JointControlSetDesiredForceTorque(IntPtr commandHandle, int dofIndex, double value);


    /// Return Type: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3CreateCollisionShapeCommandInit")]
    public static extern System.IntPtr b3CreateCollisionShapeCommandInit(IntPtr physClient);


    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///radius: double
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3CreateCollisionShapeAddSphere")]
    public static extern int b3CreateCollisionShapeAddSphere(IntPtr commandHandle, double radius);


    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///halfExtents: double*
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3CreateCollisionShapeAddBox")]
    public static extern int b3CreateCollisionShapeAddBox(IntPtr commandHandle, ref double halfExtents);


    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///radius: double
    ///height: double
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3CreateCollisionShapeAddCapsule")]
    public static extern int b3CreateCollisionShapeAddCapsule(IntPtr commandHandle, double radius, double height);


    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///radius: double
    ///height: double
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3CreateCollisionShapeAddCylinder")]
    public static extern int b3CreateCollisionShapeAddCylinder(IntPtr commandHandle, double radius, double height);


    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///fileName: char*
    ///meshScale: double*
    ///textureScaling: double
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3CreateCollisionShapeAddHeightfield")]
    public static extern int b3CreateCollisionShapeAddHeightfield(IntPtr commandHandle, [System.Runtime.InteropServices.InAttribute()] [System.Runtime.InteropServices.MarshalAsAttribute(System.Runtime.InteropServices.UnmanagedType.LPStr)] string fileName, ref double meshScale, double textureScaling);


    /// Return Type: int
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///meshScale: double*
    ///textureScaling: double
    ///heightfieldData: float*
    ///numHeightfieldRows: int
    ///numHeightfieldColumns: int
    ///replaceHeightfieldIndex: int
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3CreateCollisionShapeAddHeightfield2")]
    public static extern int b3CreateCollisionShapeAddHeightfield2(IntPtr physClient, IntPtr commandHandle, ref double meshScale, double textureScaling, ref float heightfieldData, int numHeightfieldRows, int numHeightfieldColumns, int replaceHeightfieldIndex);


    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///planeNormal: double*
    ///planeConstant: double
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3CreateCollisionShapeAddPlane")]
    public static extern int b3CreateCollisionShapeAddPlane(IntPtr commandHandle, ref double planeNormal, double planeConstant);


    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///fileName: char*
    ///meshScale: double*
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3CreateCollisionShapeAddMesh")]
    public static extern int b3CreateCollisionShapeAddMesh(IntPtr commandHandle, [System.Runtime.InteropServices.InAttribute()] [System.Runtime.InteropServices.MarshalAsAttribute(System.Runtime.InteropServices.UnmanagedType.LPStr)] string fileName, ref double meshScale);


    /// Return Type: int
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///meshScale: double*
    ///vertices: double*
    ///numVertices: int
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3CreateCollisionShapeAddConvexMesh")]
    public static extern int b3CreateCollisionShapeAddConvexMesh(IntPtr physClient, IntPtr commandHandle, ref double meshScale, ref double vertices, int numVertices);


    /// Return Type: int
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///meshScale: double*
    ///vertices: double*
    ///numVertices: int
    ///indices: int*
    ///numIndices: int
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3CreateCollisionShapeAddConcaveMesh")]
    public static extern int b3CreateCollisionShapeAddConcaveMesh(IntPtr physClient, IntPtr commandHandle, ref double meshScale, ref double vertices, int numVertices, ref int indices, int numIndices);


    /// Return Type: void
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///shapeIndex: int
    ///flags: int
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3CreateCollisionSetFlag")]
    public static extern void b3CreateCollisionSetFlag(IntPtr commandHandle, int shapeIndex, int flags);


    /// Return Type: void
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///shapeIndex: int
    ///childPosition: double*
    ///childOrientation: double*
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3CreateCollisionShapeSetChildTransform")]
    public static extern void b3CreateCollisionShapeSetChildTransform(IntPtr commandHandle, int shapeIndex, ref double childPosition, ref double childOrientation);


    /// Return Type: int
    ///statusHandle: b3SharedMemoryStatusHandle->b3SharedMemoryStatusHandle__*
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3GetStatusCollisionShapeUniqueId")]
    public static extern int b3GetStatusCollisionShapeUniqueId(IntPtr statusHandle);


    /// Return Type: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    ///collisionShapeId: int
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3InitRemoveCollisionShapeCommand")]
    public static extern System.IntPtr b3InitRemoveCollisionShapeCommand(IntPtr physClient, int collisionShapeId);


    /// Return Type: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    ///bodyUniqueId: int
    ///linkIndex: int
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3GetMeshDataCommandInit")]
    public static extern System.IntPtr b3GetMeshDataCommandInit(IntPtr physClient, int bodyUniqueId, int linkIndex);


    /// Return Type: void
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///shapeIndex: int
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3GetMeshDataSetCollisionShapeIndex")]
    public static extern void b3GetMeshDataSetCollisionShapeIndex(IntPtr commandHandle, int shapeIndex);


    /// Return Type: void
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    ///meshData: b3MeshData*
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3GetMeshData")]
    public static extern void b3GetMeshData(IntPtr physClient, ref b3MeshData meshData);


    /// Return Type: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3CreateVisualShapeCommandInit")]
    public static extern System.IntPtr b3CreateVisualShapeCommandInit(IntPtr physClient);


    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///radius: double
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3CreateVisualShapeAddSphere")]
    public static extern int b3CreateVisualShapeAddSphere(IntPtr commandHandle, double radius);


    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///halfExtents: double*
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3CreateVisualShapeAddBox")]
    public static extern int b3CreateVisualShapeAddBox(IntPtr commandHandle, ref double halfExtents);


    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///radius: double
    ///height: double
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3CreateVisualShapeAddCapsule")]
    public static extern int b3CreateVisualShapeAddCapsule(IntPtr commandHandle, double radius, double height);


    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///radius: double
    ///height: double
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3CreateVisualShapeAddCylinder")]
    public static extern int b3CreateVisualShapeAddCylinder(IntPtr commandHandle, double radius, double height);


    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///planeNormal: double*
    ///planeConstant: double
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3CreateVisualShapeAddPlane")]
    public static extern int b3CreateVisualShapeAddPlane(IntPtr commandHandle, ref double planeNormal, double planeConstant);


    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///fileName: char*
    ///meshScale: double*
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3CreateVisualShapeAddMesh")]
    public static extern int b3CreateVisualShapeAddMesh(IntPtr commandHandle, [System.Runtime.InteropServices.InAttribute()] [System.Runtime.InteropServices.MarshalAsAttribute(System.Runtime.InteropServices.UnmanagedType.LPStr)] string fileName, ref double meshScale);


    /// Return Type: int
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///meshScale: double*
    ///vertices: double*
    ///numVertices: int
    ///indices: int*
    ///numIndices: int
    ///normals: double*
    ///numNormals: int
    ///uvs: double*
    ///numUVs: int
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3CreateVisualShapeAddMesh2")]
    public static extern int b3CreateVisualShapeAddMesh2(IntPtr physClient, IntPtr commandHandle, ref double meshScale, ref double vertices, int numVertices, ref int indices, int numIndices, ref double normals, int numNormals, ref double uvs, int numUVs);


    /// Return Type: void
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///shapeIndex: int
    ///flags: int
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3CreateVisualSetFlag")]
    public static extern void b3CreateVisualSetFlag(IntPtr commandHandle, int shapeIndex, int flags);


    /// Return Type: void
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///shapeIndex: int
    ///childPosition: double*
    ///childOrientation: double*
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3CreateVisualShapeSetChildTransform")]
    public static extern void b3CreateVisualShapeSetChildTransform(IntPtr commandHandle, int shapeIndex, ref double childPosition, ref double childOrientation);


    /// Return Type: void
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///shapeIndex: int
    ///specularColor: double*
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3CreateVisualShapeSetSpecularColor")]
    public static extern void b3CreateVisualShapeSetSpecularColor(IntPtr commandHandle, int shapeIndex, ref double specularColor);


    /// Return Type: void
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///shapeIndex: int
    ///rgbaColor: double*
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3CreateVisualShapeSetRGBAColor")]
    public static extern void b3CreateVisualShapeSetRGBAColor(IntPtr commandHandle, int shapeIndex, ref double rgbaColor);


    /// Return Type: int
    ///statusHandle: b3SharedMemoryStatusHandle->b3SharedMemoryStatusHandle__*
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3GetStatusVisualShapeUniqueId")]
    public static extern int b3GetStatusVisualShapeUniqueId(IntPtr statusHandle);


    /// Return Type: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3CreateMultiBodyCommandInit")]
    public static extern System.IntPtr b3CreateMultiBodyCommandInit(IntPtr physClient);


    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///mass: double
    ///collisionShapeUnique: int
    ///visualShapeUniqueId: int
    ///basePosition: double*
    ///baseOrientation: double*
    ///baseInertialFramePosition: double*
    ///baseInertialFrameOrientation: double*
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3CreateMultiBodyBase")]
    public static extern int b3CreateMultiBodyBase(IntPtr commandHandle, double mass, int collisionShapeUnique, int visualShapeUniqueId, ref double basePosition, ref double baseOrientation, ref double baseInertialFramePosition, ref double baseInertialFrameOrientation);


    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///linkMass: double
    ///linkCollisionShapeIndex: double
    ///linkVisualShapeIndex: double
    ///linkPosition: double*
    ///linkOrientation: double*
    ///linkInertialFramePosition: double*
    ///linkInertialFrameOrientation: double*
    ///linkParentIndex: int
    ///linkJointType: int
    ///linkJointAxis: double*
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3CreateMultiBodyLink")]
    public static extern int b3CreateMultiBodyLink(IntPtr commandHandle, double linkMass, double linkCollisionShapeIndex, double linkVisualShapeIndex, ref double linkPosition, ref double linkOrientation, ref double linkInertialFramePosition, ref double linkInertialFrameOrientation, int linkParentIndex, int linkJointType, ref double linkJointAxis);


    /// Return Type: int
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///batchPositions: double*
    ///numBatchObjects: int
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3CreateMultiBodySetBatchPositions")]
    public static extern int b3CreateMultiBodySetBatchPositions(IntPtr physClient, IntPtr commandHandle, ref double batchPositions, int numBatchObjects);


    /// Return Type: void
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3CreateMultiBodyUseMaximalCoordinates")]
    public static extern void b3CreateMultiBodyUseMaximalCoordinates(IntPtr commandHandle);


    /// Return Type: void
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///flags: int
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3CreateMultiBodySetFlags")]
    public static extern void b3CreateMultiBodySetFlags(IntPtr commandHandle, int flags);


    /// Return Type: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3CreateBoxShapeCommandInit")]
    public static extern System.IntPtr b3CreateBoxShapeCommandInit(IntPtr physClient);


    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///startPosX: double
    ///startPosY: double
    ///startPosZ: double
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3CreateBoxCommandSetStartPosition")]
    public static extern int b3CreateBoxCommandSetStartPosition(IntPtr commandHandle, double startPosX, double startPosY, double startPosZ);


    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///startOrnX: double
    ///startOrnY: double
    ///startOrnZ: double
    ///startOrnW: double
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3CreateBoxCommandSetStartOrientation")]
    public static extern int b3CreateBoxCommandSetStartOrientation(IntPtr commandHandle, double startOrnX, double startOrnY, double startOrnZ, double startOrnW);


    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///halfExtentsX: double
    ///halfExtentsY: double
    ///halfExtentsZ: double
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3CreateBoxCommandSetHalfExtents")]
    public static extern int b3CreateBoxCommandSetHalfExtents(IntPtr commandHandle, double halfExtentsX, double halfExtentsY, double halfExtentsZ);


    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///mass: double
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3CreateBoxCommandSetMass")]
    public static extern int b3CreateBoxCommandSetMass(IntPtr commandHandle, double mass);


    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///collisionShapeType: int
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3CreateBoxCommandSetCollisionShapeType")]
    public static extern int b3CreateBoxCommandSetCollisionShapeType(IntPtr commandHandle, int collisionShapeType);


    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///red: double
    ///green: double
    ///blue: double
    ///alpha: double
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3CreateBoxCommandSetColorRGBA")]
    public static extern int b3CreateBoxCommandSetColorRGBA(IntPtr commandHandle, double red, double green, double blue, double alpha);


    /// Return Type: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    ///bodyUniqueId: int
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3CreatePoseCommandInit")]
    public static extern System.IntPtr b3CreatePoseCommandInit(IntPtr physClient, int bodyUniqueId);


    /// Return Type: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///bodyUniqueId: int
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3CreatePoseCommandInit2")]
    public static extern System.IntPtr b3CreatePoseCommandInit2(IntPtr commandHandle, int bodyUniqueId);


    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///startPosX: double
    ///startPosY: double
    ///startPosZ: double
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3CreatePoseCommandSetBasePosition")]
    public static extern int b3CreatePoseCommandSetBasePosition(IntPtr commandHandle, double startPosX, double startPosY, double startPosZ);


    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///startOrnX: double
    ///startOrnY: double
    ///startOrnZ: double
    ///startOrnW: double
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3CreatePoseCommandSetBaseOrientation")]
    public static extern int b3CreatePoseCommandSetBaseOrientation(IntPtr commandHandle, double startOrnX, double startOrnY, double startOrnZ, double startOrnW);


    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///linVel: double*
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3CreatePoseCommandSetBaseLinearVelocity")]
    public static extern int b3CreatePoseCommandSetBaseLinearVelocity(IntPtr commandHandle, ref double linVel);


    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///angVel: double*
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3CreatePoseCommandSetBaseAngularVelocity")]
    public static extern int b3CreatePoseCommandSetBaseAngularVelocity(IntPtr commandHandle, ref double angVel);


    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///numJointPositions: int
    ///jointPositions: double*
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3CreatePoseCommandSetJointPositions")]
    public static extern int b3CreatePoseCommandSetJointPositions(IntPtr commandHandle, int numJointPositions, ref double jointPositions);


    /// Return Type: int
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///jointIndex: int
    ///jointPosition: double
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3CreatePoseCommandSetJointPosition")]
    public static extern int b3CreatePoseCommandSetJointPosition(IntPtr physClient, IntPtr commandHandle, int jointIndex, double jointPosition);


    /// Return Type: int
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///jointIndex: int
    ///jointPosition: double*
    ///posSize: int
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3CreatePoseCommandSetJointPositionMultiDof")]
    public static extern int b3CreatePoseCommandSetJointPositionMultiDof(IntPtr physClient, IntPtr commandHandle, int jointIndex, ref double jointPosition, int posSize);


    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///numJointPositions: int
    ///q: double*
    ///hasQ: int*
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3CreatePoseCommandSetQ")]
    public static extern int b3CreatePoseCommandSetQ(IntPtr commandHandle, int numJointPositions, ref double q, ref int hasQ);


    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///numJointVelocities: int
    ///qDots: double*
    ///hasQdots: int*
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3CreatePoseCommandSetQdots")]
    public static extern int b3CreatePoseCommandSetQdots(IntPtr commandHandle, int numJointVelocities, ref double qDots, ref int hasQdots);


    /// Return Type: int
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///numJointVelocities: int
    ///jointVelocities: double*
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3CreatePoseCommandSetJointVelocities")]
    public static extern int b3CreatePoseCommandSetJointVelocities(IntPtr physClient, IntPtr commandHandle, int numJointVelocities, ref double jointVelocities);


    /// Return Type: int
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///jointIndex: int
    ///jointVelocity: double
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3CreatePoseCommandSetJointVelocity")]
    public static extern int b3CreatePoseCommandSetJointVelocity(IntPtr physClient, IntPtr commandHandle, int jointIndex, double jointVelocity);


    /// Return Type: int
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///jointIndex: int
    ///jointVelocity: double*
    ///velSize: int
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3CreatePoseCommandSetJointVelocityMultiDof")]
    public static extern int b3CreatePoseCommandSetJointVelocityMultiDof(IntPtr physClient, IntPtr commandHandle, int jointIndex, ref double jointVelocity, int velSize);


    /// Return Type: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    ///bodyUniqueId: int
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3CreateSensorCommandInit")]
    public static extern System.IntPtr b3CreateSensorCommandInit(IntPtr physClient, int bodyUniqueId);


    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///jointIndex: int
    ///enable: int
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3CreateSensorEnable6DofJointForceTorqueSensor")]
    public static extern int b3CreateSensorEnable6DofJointForceTorqueSensor(IntPtr commandHandle, int jointIndex, int enable);


    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///linkIndex: int
    ///enable: int
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3CreateSensorEnableIMUForLink")]
    public static extern int b3CreateSensorEnableIMUForLink(IntPtr commandHandle, int linkIndex, int enable);


    /// Return Type: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    ///bodyUniqueId: int
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3RequestActualStateCommandInit")]
    public static extern System.IntPtr b3RequestActualStateCommandInit(IntPtr physClient, int bodyUniqueId);


    /// Return Type: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///bodyUniqueId: int
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3RequestActualStateCommandInit2")]
    public static extern System.IntPtr b3RequestActualStateCommandInit2(IntPtr commandHandle, int bodyUniqueId);


    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///computeLinkVelocity: int
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3RequestActualStateCommandComputeLinkVelocity")]
    public static extern int b3RequestActualStateCommandComputeLinkVelocity(IntPtr commandHandle, int computeLinkVelocity);


    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///computeForwardKinematics: int
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3RequestActualStateCommandComputeForwardKinematics")]
    public static extern int b3RequestActualStateCommandComputeForwardKinematics(IntPtr commandHandle, int computeForwardKinematics);


    /// Return Type: int
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    ///statusHandle: b3SharedMemoryStatusHandle->b3SharedMemoryStatusHandle__*
    ///jointIndex: int
    ///state: b3JointSensorState*
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3GetJointState")]
    public static extern int b3GetJointState(IntPtr physClient, IntPtr statusHandle, int jointIndex, ref b3JointSensorState state);


    /// Return Type: int
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    ///statusHandle: b3SharedMemoryStatusHandle->b3SharedMemoryStatusHandle__*
    ///jointIndex: int
    ///state: b3JointSensorState2*
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3GetJointStateMultiDof")]
    public static extern int b3GetJointStateMultiDof(IntPtr physClient, IntPtr statusHandle, int jointIndex, ref b3JointSensorState2 state);


    /// Return Type: int
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    ///statusHandle: b3SharedMemoryStatusHandle->b3SharedMemoryStatusHandle__*
    ///linkIndex: int
    ///state: b3LinkState*
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3GetLinkState")]
    public static extern int b3GetLinkState(IntPtr physClient, IntPtr statusHandle, int linkIndex, ref b3LinkState state);


    /// Return Type: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    ///rayFromWorldX: double
    ///rayFromWorldY: double
    ///rayFromWorldZ: double
    ///rayToWorldX: double
    ///rayToWorldY: double
    ///rayToWorldZ: double
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3PickBody")]
    public static extern System.IntPtr b3PickBody(IntPtr physClient, double rayFromWorldX, double rayFromWorldY, double rayFromWorldZ, double rayToWorldX, double rayToWorldY, double rayToWorldZ);


    /// Return Type: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    ///rayFromWorldX: double
    ///rayFromWorldY: double
    ///rayFromWorldZ: double
    ///rayToWorldX: double
    ///rayToWorldY: double
    ///rayToWorldZ: double
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3MovePickedBody")]
    public static extern System.IntPtr b3MovePickedBody(IntPtr physClient, double rayFromWorldX, double rayFromWorldY, double rayFromWorldZ, double rayToWorldX, double rayToWorldY, double rayToWorldZ);


    /// Return Type: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3RemovePickingConstraint")]
    public static extern System.IntPtr b3RemovePickingConstraint(IntPtr physClient);


    /// Return Type: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    ///rayFromWorldX: double
    ///rayFromWorldY: double
    ///rayFromWorldZ: double
    ///rayToWorldX: double
    ///rayToWorldY: double
    ///rayToWorldZ: double
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3CreateRaycastCommandInit")]
    public static extern System.IntPtr b3CreateRaycastCommandInit(IntPtr physClient, double rayFromWorldX, double rayFromWorldY, double rayFromWorldZ, double rayToWorldX, double rayToWorldY, double rayToWorldZ);


    /// Return Type: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3CreateRaycastBatchCommandInit")]
    public static extern System.IntPtr b3CreateRaycastBatchCommandInit(IntPtr physClient);


    /// Return Type: void
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///numThreads: int
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3RaycastBatchSetNumThreads")]
    public static extern void b3RaycastBatchSetNumThreads(IntPtr commandHandle, int numThreads);


    /// Return Type: void
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///rayFromWorld: double*
    ///rayToWorld: double*
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3RaycastBatchAddRay")]
    public static extern void b3RaycastBatchAddRay(IntPtr commandHandle, ref double rayFromWorld, ref double rayToWorld);


    /// Return Type: void
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///rayFromWorld: double*
    ///rayToWorld: double*
    ///numRays: int
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3RaycastBatchAddRays")]
    public static extern void b3RaycastBatchAddRays(IntPtr physClient, IntPtr commandHandle, ref double rayFromWorld, ref double rayToWorld, int numRays);


    /// Return Type: void
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///parentObjectUniqueId: int
    ///parentLinkIndex: int
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3RaycastBatchSetParentObject")]
    public static extern void b3RaycastBatchSetParentObject(IntPtr commandHandle, int parentObjectUniqueId, int parentLinkIndex);


    /// Return Type: void
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    ///raycastInfo: b3RaycastInformation*
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3GetRaycastInformation")]
    public static extern void b3GetRaycastInformation(IntPtr physClient, ref b3RaycastInformation raycastInfo);


    /// Return Type: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3ApplyExternalForceCommandInit")]
    public static extern System.IntPtr b3ApplyExternalForceCommandInit(IntPtr physClient);


    /// Return Type: void
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///bodyUniqueId: int
    ///linkId: int
    ///force: double*
    ///position: double*
    ///flag: int
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3ApplyExternalForce")]
    public static extern void b3ApplyExternalForce(IntPtr commandHandle, int bodyUniqueId, int linkId, ref double force, ref double position, int flag);


    /// Return Type: void
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///bodyUniqueId: int
    ///linkId: int
    ///torque: double*
    ///flag: int
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3ApplyExternalTorque")]
    public static extern void b3ApplyExternalTorque(IntPtr commandHandle, int bodyUniqueId, int linkId, ref double torque, int flag);


    /// Return Type: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    ///fileName: char*
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3LoadSoftBodyCommandInit")]
    public static extern System.IntPtr b3LoadSoftBodyCommandInit(IntPtr physClient, [System.Runtime.InteropServices.InAttribute()] [System.Runtime.InteropServices.MarshalAsAttribute(System.Runtime.InteropServices.UnmanagedType.LPStr)] string fileName);


    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///scale: double
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3LoadSoftBodySetScale")]
    public static extern int b3LoadSoftBodySetScale(IntPtr commandHandle, double scale);


    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///mass: double
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3LoadSoftBodySetMass")]
    public static extern int b3LoadSoftBodySetMass(IntPtr commandHandle, double mass);


    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///collisionMargin: double
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3LoadSoftBodySetCollisionMargin")]
    public static extern int b3LoadSoftBodySetCollisionMargin(IntPtr commandHandle, double collisionMargin);


    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///startPosX: double
    ///startPosY: double
    ///startPosZ: double
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3LoadSoftBodySetStartPosition")]
    public static extern int b3LoadSoftBodySetStartPosition(IntPtr commandHandle, double startPosX, double startPosY, double startPosZ);


    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///startOrnX: double
    ///startOrnY: double
    ///startOrnZ: double
    ///startOrnW: double
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3LoadSoftBodySetStartOrientation")]
    public static extern int b3LoadSoftBodySetStartOrientation(IntPtr commandHandle, double startOrnX, double startOrnY, double startOrnZ, double startOrnW);


    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///filename: char*
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3LoadSoftBodyUpdateSimMesh")]
    public static extern int b3LoadSoftBodyUpdateSimMesh(IntPtr commandHandle, [System.Runtime.InteropServices.InAttribute()] [System.Runtime.InteropServices.MarshalAsAttribute(System.Runtime.InteropServices.UnmanagedType.LPStr)] string filename);


    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///corotatedMu: double
    ///corotatedLambda: double
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3LoadSoftBodyAddCorotatedForce")]
    public static extern int b3LoadSoftBodyAddCorotatedForce(IntPtr commandHandle, double corotatedMu, double corotatedLambda);


    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///NeoHookeanMu: double
    ///NeoHookeanLambda: double
    ///NeoHookeanDamping: double
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3LoadSoftBodyAddNeoHookeanForce")]
    public static extern int b3LoadSoftBodyAddNeoHookeanForce(IntPtr commandHandle, double NeoHookeanMu, double NeoHookeanLambda, double NeoHookeanDamping);


    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///springElasticStiffness: double
    ///springDampingStiffness: double
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3LoadSoftBodyAddMassSpringForce")]
    public static extern int b3LoadSoftBodyAddMassSpringForce(IntPtr commandHandle, double springElasticStiffness, double springDampingStiffness);


    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///gravityX: double
    ///gravityY: double
    ///gravityZ: double
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3LoadSoftBodyAddGravityForce")]
    public static extern int b3LoadSoftBodyAddGravityForce(IntPtr commandHandle, double gravityX, double gravityY, double gravityZ);


    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///collisionHardness: double
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3LoadSoftBodySetCollisionHardness")]
    public static extern int b3LoadSoftBodySetCollisionHardness(IntPtr commandHandle, double collisionHardness);


    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///useSelfCollision: int
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3LoadSoftBodySetSelfCollision")]
    public static extern int b3LoadSoftBodySetSelfCollision(IntPtr commandHandle, int useSelfCollision);


    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///useFaceContact: int
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3LoadSoftBodyUseFaceContact")]
    public static extern int b3LoadSoftBodyUseFaceContact(IntPtr commandHandle, int useFaceContact);


    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///frictionCoefficient: double
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3LoadSoftBodySetFrictionCoefficient")]
    public static extern int b3LoadSoftBodySetFrictionCoefficient(IntPtr commandHandle, double frictionCoefficient);


    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///useBendingSprings: int
    ///bendingStiffness: double
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3LoadSoftBodyUseBendingSprings")]
    public static extern int b3LoadSoftBodyUseBendingSprings(IntPtr commandHandle, int useBendingSprings, double bendingStiffness);


    /// Return Type: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3RequestVREventsCommandInit")]
    public static extern System.IntPtr b3RequestVREventsCommandInit(IntPtr physClient);


    /// Return Type: void
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///deviceTypeFilter: int
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3VREventsSetDeviceTypeFilter")]
    public static extern void b3VREventsSetDeviceTypeFilter(IntPtr commandHandle, int deviceTypeFilter);


    /// Return Type: void
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    ///vrEventsData: b3VREventsData*
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3GetVREventsData")]
    public static extern void b3GetVREventsData(IntPtr physClient, ref b3VREventsData vrEventsData);


    /// Return Type: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3SetVRCameraStateCommandInit")]
    public static extern System.IntPtr b3SetVRCameraStateCommandInit(IntPtr physClient);


    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///rootPos: double*
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3SetVRCameraRootPosition")]
    public static extern int b3SetVRCameraRootPosition(IntPtr commandHandle, ref double rootPos);


    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///rootOrn: double*
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3SetVRCameraRootOrientation")]
    public static extern int b3SetVRCameraRootOrientation(IntPtr commandHandle, ref double rootOrn);


    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///objectUniqueId: int
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3SetVRCameraTrackingObject")]
    public static extern int b3SetVRCameraTrackingObject(IntPtr commandHandle, int objectUniqueId);


    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///flag: int
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3SetVRCameraTrackingObjectFlag")]
    public static extern int b3SetVRCameraTrackingObjectFlag(IntPtr commandHandle, int flag);


    /// Return Type: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3RequestKeyboardEventsCommandInit")]
    public static extern System.IntPtr b3RequestKeyboardEventsCommandInit(IntPtr physClient);


    /// Return Type: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3RequestKeyboardEventsCommandInit2")]
    public static extern System.IntPtr b3RequestKeyboardEventsCommandInit2(IntPtr commandHandle);


    /// Return Type: void
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    ///keyboardEventsData: b3KeyboardEventsData*
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3GetKeyboardEventsData")]
    public static extern void b3GetKeyboardEventsData(IntPtr physClient, ref b3KeyboardEventsData keyboardEventsData);


    /// Return Type: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3RequestMouseEventsCommandInit")]
    public static extern System.IntPtr b3RequestMouseEventsCommandInit(IntPtr physClient);


    /// Return Type: void
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    ///mouseEventsData: b3MouseEventsData*
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3GetMouseEventsData")]
    public static extern void b3GetMouseEventsData(IntPtr physClient, ref b3MouseEventsData mouseEventsData);


    /// Return Type: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3StateLoggingCommandInit")]
    public static extern System.IntPtr b3StateLoggingCommandInit(IntPtr physClient);


    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///loggingType: int
    ///fileName: char*
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3StateLoggingStart")]
    public static extern int b3StateLoggingStart(IntPtr commandHandle, int loggingType, [System.Runtime.InteropServices.InAttribute()] [System.Runtime.InteropServices.MarshalAsAttribute(System.Runtime.InteropServices.UnmanagedType.LPStr)] string fileName);


    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///objectUniqueId: int
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3StateLoggingAddLoggingObjectUniqueId")]
    public static extern int b3StateLoggingAddLoggingObjectUniqueId(IntPtr commandHandle, int objectUniqueId);


    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///maxLogDof: int
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3StateLoggingSetMaxLogDof")]
    public static extern int b3StateLoggingSetMaxLogDof(IntPtr commandHandle, int maxLogDof);


    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///linkIndexA: int
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3StateLoggingSetLinkIndexA")]
    public static extern int b3StateLoggingSetLinkIndexA(IntPtr commandHandle, int linkIndexA);


    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///linkIndexB: int
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3StateLoggingSetLinkIndexB")]
    public static extern int b3StateLoggingSetLinkIndexB(IntPtr commandHandle, int linkIndexB);


    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///bodyAUniqueId: int
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3StateLoggingSetBodyAUniqueId")]
    public static extern int b3StateLoggingSetBodyAUniqueId(IntPtr commandHandle, int bodyAUniqueId);


    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///bodyBUniqueId: int
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3StateLoggingSetBodyBUniqueId")]
    public static extern int b3StateLoggingSetBodyBUniqueId(IntPtr commandHandle, int bodyBUniqueId);


    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///deviceTypeFilter: int
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3StateLoggingSetDeviceTypeFilter")]
    public static extern int b3StateLoggingSetDeviceTypeFilter(IntPtr commandHandle, int deviceTypeFilter);


    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///logFlags: int
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3StateLoggingSetLogFlags")]
    public static extern int b3StateLoggingSetLogFlags(IntPtr commandHandle, int logFlags);


    /// Return Type: int
    ///statusHandle: b3SharedMemoryStatusHandle->b3SharedMemoryStatusHandle__*
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3GetStatusLoggingUniqueId")]
    public static extern int b3GetStatusLoggingUniqueId(IntPtr statusHandle);


    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///loggingUid: int
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3StateLoggingStop")]
    public static extern int b3StateLoggingStop(IntPtr commandHandle, int loggingUid);


    /// Return Type: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    ///name: char*
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3ProfileTimingCommandInit")]
    public static extern System.IntPtr b3ProfileTimingCommandInit(IntPtr physClient, [System.Runtime.InteropServices.InAttribute()] [System.Runtime.InteropServices.MarshalAsAttribute(System.Runtime.InteropServices.UnmanagedType.LPStr)] string name);


    /// Return Type: void
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///duration: int
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3SetProfileTimingDuractionInMicroSeconds")]
    public static extern void b3SetProfileTimingDuractionInMicroSeconds(IntPtr commandHandle, int duration);


    /// Return Type: void
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///type: int
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3SetProfileTimingType")]
    public static extern void b3SetProfileTimingType(IntPtr commandHandle, int type);


    /// Return Type: void
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    ///timingName: char*
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3PushProfileTiming")]
    public static extern void b3PushProfileTiming(IntPtr physClient, [System.Runtime.InteropServices.InAttribute()] [System.Runtime.InteropServices.MarshalAsAttribute(System.Runtime.InteropServices.UnmanagedType.LPStr)] string timingName);


    /// Return Type: void
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3PopProfileTiming")]
    public static extern void b3PopProfileTiming(IntPtr physClient);


    /// Return Type: void
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    ///timeOutInSeconds: double
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3SetTimeOut")]
    public static extern void b3SetTimeOut(IntPtr physClient, double timeOutInSeconds);


    /// Return Type: double
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3GetTimeOut")]
    public static extern double b3GetTimeOut(IntPtr physClient);


    /// Return Type: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    ///path: char*
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3SetAdditionalSearchPath")]
    public static extern System.IntPtr b3SetAdditionalSearchPath(IntPtr physClient, [System.Runtime.InteropServices.InAttribute()] [System.Runtime.InteropServices.MarshalAsAttribute(System.Runtime.InteropServices.UnmanagedType.LPStr)] string path);


    /// Return Type: void
    ///posA: double*
    ///ornA: double*
    ///posB: double*
    ///ornB: double*
    ///outPos: double*
    ///outOrn: double*
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3MultiplyTransforms")]
    public static extern void b3MultiplyTransforms(ref double posA, ref double ornA, ref double posB, ref double ornB, ref double outPos, ref double outOrn);


    /// Return Type: void
    ///pos: double*
    ///orn: double*
    ///outPos: double*
    ///outOrn: double*
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3InvertTransform")]
    public static extern void b3InvertTransform(ref double pos, ref double orn, ref double outPos, ref double outOrn);


    /// Return Type: void
    ///startQuat: double*
    ///endQuat: double*
    ///interpolationFraction: double
    ///outOrn: double*
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3QuaternionSlerp")]
    public static extern void b3QuaternionSlerp(ref double startQuat, ref double endQuat, double interpolationFraction, ref double outOrn);


    /// Return Type: void
    ///axis: double*
    ///angle: double
    ///outQuat: double*
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3GetQuaternionFromAxisAngle")]
    public static extern void b3GetQuaternionFromAxisAngle(ref double axis, double angle, ref double outQuat);


    /// Return Type: void
    ///quat: double*
    ///axis: double*
    ///angle: double*
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3GetAxisAngleFromQuaternion")]
    public static extern void b3GetAxisAngleFromQuaternion(ref double quat, ref double axis, ref double angle);


    /// Return Type: void
    ///startQuat: double*
    ///endQuat: double*
    ///outOrn: double*
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3GetQuaternionDifference")]
    public static extern void b3GetQuaternionDifference(ref double startQuat, ref double endQuat, ref double outOrn);


    /// Return Type: void
    ///startQuat: double*
    ///endQuat: double*
    ///axisOut: double*
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3GetAxisDifferenceQuaternion")]
    public static extern void b3GetAxisDifferenceQuaternion(ref double startQuat, ref double endQuat, ref double axisOut);


    /// Return Type: void
    ///startQuat: double*
    ///endQuat: double*
    ///deltaTime: double
    ///angVelOut: double*
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3CalculateVelocityQuaternion")]
    public static extern void b3CalculateVelocityQuaternion(ref double startQuat, ref double endQuat, double deltaTime, ref double angVelOut);


    /// Return Type: void
    ///quat: double*
    ///vec: double*
    ///vecOut: double*
    [System.Runtime.InteropServices.DllImportAttribute(dllName, EntryPoint = "b3RotateVector")]
    public static extern void b3RotateVector(ref double quat, ref double vec, ref double vecOut);

}
