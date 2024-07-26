#include "MotionMatching.h"

#include "Core/StringCrc.h"
#include "ECWorld/SceneWorld.h"
#include "ECWorld/SkinMeshComponent.h"
#include "ECWorld/TransformComponent.h"
#include "Rendering/RenderContext.h"
#include "Rendering/Utility/VertexLayoutUtility.h"
#include "Rendering/Resources/ShaderResource.h"
#include "Window/Input.h"
#include <ECWorld/MotionMatchingComponent.h>
#include "Display/CameraController.h"

namespace engine
{

enum
{
    GAMEPAD_STICK_LEFT,
    GAMEPAD_STICK_RIGHT,
};

Vec3 gamepad_get_stick(int stick)
{
    Vec3 pad(0, 0, 0); float am = 1.0f;
    if (stick == GAMEPAD_STICK_LEFT)
    {
        if (Input::Get().IsKeyPressed(KeyCode::h)) pad.x += am;
        if (Input::Get().IsKeyPressed(KeyCode::f)) pad.x -= am;
        if (Input::Get().IsKeyPressed(KeyCode::t)) pad.z += am;
        if (Input::Get().IsKeyPressed(KeyCode::g)) pad.z -= am;
    }
    return pad;
}

// Taken from https://theorangeduck.com/page/spring-roll-call#controllers
void simulation_positions_update(
    Vec3& position,
    Vec3& velocity,
    Vec3& acceleration,
    const Vec3 desired_velocity,
    const float halflife,
    const float dt)
{
    float y = halflife_to_damping(halflife) / 2.0f;
    Vec3 j0 = velocity - desired_velocity;
    Vec3 j1 = acceleration + j0 * y;
    float eydt = fast_negexpf(y * dt);

    Vec3 position_prev = position;

    position = eydt * (((-j1) / (y * y)) + ((-j0 - j1 * dt) / y)) +
        (j1 / (y * y)) + j0 / y + desired_velocity * dt + position_prev;
    velocity = eydt * (j0 + j1 * dt) + desired_velocity;
    acceleration = eydt * (acceleration - j1 * y * dt);
}

void trajectory_positions_predict(
    slice1d<Vec3> positions,
    slice1d<Vec3> velocities,
    slice1d<Vec3> accelerations,
    const Vec3 position,
    const Vec3 velocity,
    const Vec3 acceleration,
    const slice1d<Vec3> desired_velocities,
    const float halflife,
    const float dt
  )
{
    positions(0) = position;
    velocities(0) = velocity;
    accelerations(0) = acceleration;

    for (int i = 1; i < positions.size; i++)
    {
        positions(i) = positions(i - 1);
        velocities(i) = velocities(i - 1);
        accelerations(i) = accelerations(i - 1);

        simulation_positions_update(
            positions(i),
            velocities(i),
            accelerations(i),
            desired_velocities(i),
            halflife,
            dt
           );
    }
}

Vec3 desired_velocity_update(
    const Vec3 gamepadstick_left,
    const float camera_azimuth,
    const quat simulation_rotation,
    const float fwrd_speed,
    const float side_speed,
    const float back_speed)
{
    // Find stick position in world space by rotating using camera azimuth
    Vec3 global_stick_direction = quat_mul_Vec3(
        quat_from_angle_axis(camera_azimuth, Vec3(0, 1, 0)), gamepadstick_left);

    // Find stick position local to current facing direction
    Vec3 local_stick_direction = quat_inv_mul_Vec3(
        simulation_rotation, global_stick_direction);

    // Scale stick by forward, sideways and backwards speeds
    Vec3 local_desired_velocity = local_stick_direction.z > 0.0 ?
        Vec3(side_speed, 0.0f, fwrd_speed) * local_stick_direction :
        Vec3(side_speed, 0.0f, back_speed) * local_stick_direction;

    // Re-orientate into the world space
    return quat_mul_Vec3(simulation_rotation, local_desired_velocity);
}

// Predict what the desired velocity will be in the 
// future. Here we need to use the future trajectory 
// rotation as well as predicted future camera 
// position to find an accurate desired velocity in 
// the world space
void trajectory_desired_velocities_predict(
    slice1d<Vec3> desired_velocities,
    const slice1d<quat> trajectory_rotations,
    const Vec3 desired_velocity,
    const float camera_azimuth,
    const Vec3 gamepadstick_left,
    const Vec3 gamepadstick_right,
    const bool desired_strafe,
    const float fwrd_speed,
    const float side_speed,
    const float back_speed,
    const float dt)
{
    desired_velocities(0) = desired_velocity;

    for (int i = 1; i < desired_velocities.size; i++)
    {
        desired_velocities(i) = desired_velocity_update(
            gamepadstick_left,
            0.0f,
            trajectory_rotations(i),
            fwrd_speed,
            side_speed,
            back_speed);
    }
}

void simulation_rotations_update(
    quat& rotation,
    Vec3& angular_velocity,
    const quat desired_rotation,
    const float halflife,
    const float dt)
{
    simple_spring_damper_exact(
        rotation,
        angular_velocity,
        desired_rotation,
        halflife, dt);
}

quat desired_rotation_update(
    const quat desired_rotation,
    const Vec3 gamepadstick_left,
    const Vec3 gamepadstick_right,
    const float camera_azimuth,
    const bool desired_strafe,
    const Vec3 desired_velocity)
{
    quat desired_rotation_curr = desired_rotation;

    // If strafe is active then desired direction is coming from right
    // stick as long as that stick is being used, otherwise we assume
    // forward facing
    if (desired_strafe)
    {
        Vec3 desired_direction = quat_mul_Vec3(quat_from_angle_axis(camera_azimuth, Vec3(0, 1, 0)), Vec3(0, 0, -1));

        if (length(gamepadstick_right) > 0.01f)
        {
            desired_direction = quat_mul_Vec3(quat_from_angle_axis(camera_azimuth, Vec3(0, 1, 0)), normalize(gamepadstick_right));
        }

        return quat_from_angle_axis(atan2f(desired_direction.x, desired_direction.z), Vec3(0, 1, 0));
    }

    // If strafe is not active the desired direction comes from the left 
    // stick as long as that stick is being used
    else if (length(gamepadstick_left) > 0.01f)
    {

        Vec3 desired_direction = normalize(desired_velocity);
        return quat_from_angle_axis(atan2f(desired_direction.x, desired_direction.z), Vec3(0, 1, 0));
    }

    // Otherwise desired direction remains the same
    else
    {
        return desired_rotation_curr;
    }
}

// Predict desired rotations given the estimated future 
// camera rotation and other parameters
void trajectory_desired_rotations_predict(
    slice1d<quat> desired_rotations,
    const slice1d<Vec3> desired_velocities,
    const quat desired_rotation,
    const float camera_azimuth,
    const Vec3 gamepadstick_left,
    const Vec3 gamepadstick_right,
    const bool desired_strafe,
    const float dt)
{
    desired_rotations(0) = desired_rotation;

    for (int i = 1; i < desired_rotations.size; i++)
    {
        desired_rotations(i) = desired_rotation_update(
            desired_rotations(i - 1),
            gamepadstick_left,
            gamepadstick_right,
            0.0f,
            desired_strafe,
            desired_velocities(i));
    }
}

void trajectory_rotations_predict(
    slice1d<quat> rotations,
    slice1d<Vec3> angular_velocities,
    const quat rotation,
    const Vec3 angular_velocity,
    const slice1d<quat> desired_rotations,
    const float halflife,
    const float dt)
{
    rotations.set(rotation);
    angular_velocities.set(angular_velocity);

    for (int i = 1; i < rotations.size; i++)
    {
        simulation_rotations_update(
            rotations(i),
            angular_velocities(i),
            desired_rotations(i),
            halflife,
            i * dt);
    }
}

void desired_gait_update(
    float& desired_gait,
    float& desired_gait_velocity,
    const float dt,
    const float gait_change_halflife = 0.1f)
{
    simple_spring_damper_exact(
        desired_gait,
        desired_gait_velocity,
        1.0f,
        gait_change_halflife,
        dt);
}



// Copy a part of a feature vector from the 
// matching database into the query feature vector
void query_copy_denormalized_feature(
    slice1d<float> query,
    int& offset,
    const int size,
    const slice1d<float> features,
    const slice1d<float> features_offset,
    const slice1d<float> features_scale)
{
    for (int i = 0; i < size; i++)
    {
        query(offset + i) = features(offset + i) * features_scale(offset + i) + features_offset(offset + i);
    }

    offset += size;
}

// Compute the query feature vector for the current 
// trajectory controlled by the gamepad.
void query_compute_trajectory_position_feature(
    slice1d<float> query,
    int& offset,
    const Vec3 root_position,
    const quat root_rotation,
    const slice1d<Vec3> trajectory_positions)
{
    Vec3 traj0 = quat_inv_mul_Vec3(root_rotation, trajectory_positions(1) - root_position);
    Vec3 traj1 = quat_inv_mul_Vec3(root_rotation, trajectory_positions(2) - root_position);
    Vec3 traj2 = quat_inv_mul_Vec3(root_rotation, trajectory_positions(3) - root_position);

    query(offset + 0) = traj0.x;
    query(offset + 1) = traj0.z;
    query(offset + 2) = traj1.x;
    query(offset + 3) = traj1.z;
    query(offset + 4) = traj2.x;
    query(offset + 5) = traj2.z;

    offset += 6;
}

// Same but for the trajectory direction
void query_compute_trajectory_direction_feature(
    slice1d<float> query,
    int& offset,
    const quat root_rotation,
    const slice1d<quat> trajectory_rotations)
{
    Vec3 traj0 = quat_inv_mul_Vec3(root_rotation, quat_mul_Vec3(trajectory_rotations(1), Vec3(0, 0, 1)));
    Vec3 traj1 = quat_inv_mul_Vec3(root_rotation, quat_mul_Vec3(trajectory_rotations(2), Vec3(0, 0, 1)));
    Vec3 traj2 = quat_inv_mul_Vec3(root_rotation, quat_mul_Vec3(trajectory_rotations(3), Vec3(0, 0, 1)));

    query(offset + 0) = traj0.x;
    query(offset + 1) = traj0.z;
    query(offset + 2) = traj1.x;
    query(offset + 3) = traj1.z;
    query(offset + 4) = traj2.x;
    query(offset + 5) = traj2.z;

    offset += 6;
}

// This function transitions the inertializer for 
// the full character. It takes as input the current 
// offsets, as well as the root transition locations,
// current root state, and the full pose information 
// for the pose being transitioned from (src) as well 
// as the pose being transitioned to (dst) in their
// own animation spaces.
void inertialize_pose_transition(
    slice1d<Vec3> bone_offset_positions,
    slice1d<Vec3> bone_offset_velocities,
    slice1d<quat> bone_offset_rotations,
    slice1d<Vec3> bone_offset_angular_velocities,
    Vec3& transition_src_position,
    quat& transition_src_rotation,
    Vec3& transition_dst_position,
    quat& transition_dst_rotation,
    const Vec3 root_position,
    const Vec3 root_velocity,
    const quat root_rotation,
    const Vec3 root_angular_velocity,
    const slice1d<Vec3> bone_src_positions,
    const slice1d<Vec3> bone_src_velocities,
    const slice1d<quat> bone_src_rotations,
    const slice1d<Vec3> bone_src_angular_velocities,
    const slice1d<Vec3> bone_dst_positions,
    const slice1d<Vec3> bone_dst_velocities,
    const slice1d<quat> bone_dst_rotations,
    const slice1d<Vec3> bone_dst_angular_velocities)
{
    // First we record the root position and rotation
    // in the animation data for the source and destination
    // animation
    transition_dst_position = root_position;
    transition_dst_rotation = root_rotation;
    transition_src_position = bone_dst_positions(0);
    transition_src_rotation = bone_dst_rotations(0);

    // We then find the velocities so we can transition the 
    // root inertiaizers
    Vec3 world_space_dst_velocity = quat_mul_Vec3(transition_dst_rotation,
        quat_inv_mul_Vec3(transition_src_rotation, bone_dst_velocities(0)));

    Vec3 world_space_dst_angular_velocity = quat_mul_Vec3(transition_dst_rotation,
        quat_inv_mul_Vec3(transition_src_rotation, bone_dst_angular_velocities(0)));

    // Transition inertializers recording the offsets for 
    // the root joint
    inertialize_transition(
        bone_offset_positions(0),
        bone_offset_velocities(0),
        root_position,
        root_velocity,
        root_position,
        world_space_dst_velocity);

    inertialize_transition(
        bone_offset_rotations(0),
        bone_offset_angular_velocities(0),
        root_rotation,
        root_angular_velocity,
        root_rotation,
        world_space_dst_angular_velocity);

    // Transition all the inertializers for each other bone
    for (int i = 1; i < bone_offset_positions.size; i++)
    {
        inertialize_transition(
            bone_offset_positions(i),
            bone_offset_velocities(i),
            bone_src_positions(i),
            bone_src_velocities(i),
            bone_dst_positions(i),
            bone_dst_velocities(i));

        inertialize_transition(
            bone_offset_rotations(i),
            bone_offset_angular_velocities(i),
            bone_src_rotations(i),
            bone_src_angular_velocities(i),
            bone_dst_rotations(i),
            bone_dst_angular_velocities(i));
    }
}

void deform_character_mesh(
    Mesh& mesh,
    const character& c,
    const slice1d<Vec3> bone_anim_positions,
    const slice1d<quat> bone_anim_rotations,
    const slice1d<int> bone_parents)
{

    linear_blend_skinning_positions(
        slice1d<Vec3>(mesh.vertexCount, (Vec3*)mesh.vertices),
        c.positions,
        c.bone_weights,
        c.bone_indices,
        c.bone_rest_positions,
        c.bone_rest_rotations,
        bone_anim_positions,
        bone_anim_rotations);
    linear_blend_skinning_normals(
        slice1d<Vec3>(mesh.vertexCount, (Vec3*)mesh.normals),
        c.normals,
        c.bone_weights,
        c.bone_indices,
        c.bone_rest_rotations,
        bone_anim_rotations);
}

void MotionMatching::Init()
{
    AddDependentShaderResource(GetRenderContext()->RegisterShaderProgram("MotionMatchingProgram", "vs_whiteModel", "fs_whiteModel"));

    bgfx::setViewName(GetViewID(), "MotionMatchingRenderer");
}

void MotionMatching::UpdateView(const float* pViewMatrix, const float* pProjectionMatrix)
{
	UpdateViewRenderTarget();
	bgfx::setViewTransform(GetViewID(), pViewMatrix, pProjectionMatrix);
}

void MotionMatching::Build()
{
 
    // Load Animation Data and build Matching Database
    character_load(m_character, "C:/Users/zw186/OneDrive/Desktop/catdog/CatDogEngine/Engine/Source/Runtime/MotionMatching/resources/character.bin");
    database_load(db,"C:/Users/zw186/OneDrive/Desktop/catdog/CatDogEngine/Engine/Source/Runtime/MotionMatching/resources/database.bin");

    float feature_weight_foot_position = 0.75f;
    float feature_weight_foot_velocity = 1.0f;
    float feature_weight_hip_velocity = 1.0f;
    float feature_weight_trajectory_positions = 1.0f;
    float feature_weight_trajectory_directions = 1.5f;

    database_build_matching_features(
        db,
        feature_weight_foot_position,
        feature_weight_foot_velocity,
        feature_weight_hip_velocity,
        feature_weight_trajectory_positions,
        feature_weight_trajectory_directions);

    database_save_matching_features(db, "C:/Users/zw186/OneDrive/Desktop/catdog/CatDogEngine/Engine/Source/Runtime/MotionMatching/resources/features.bin");

    // Pose & Inertializer Data

    frame_index = db.range_starts(0);
    inertialize_blending_halflife = 0.1f;

    curr_bone_positions = db.bone_positions(frame_index);
    curr_bone_velocities = db.bone_velocities(frame_index);
    curr_bone_rotations = db.bone_rotations(frame_index);
    curr_bone_angular_velocities = db.bone_angular_velocities(frame_index);
    curr_bone_contacts = db.contact_states(frame_index);

    trns_bone_positions = db.bone_positions(frame_index);
    trns_bone_velocities = db.bone_velocities(frame_index);
    trns_bone_rotations = db.bone_rotations(frame_index);
    trns_bone_angular_velocities = db.bone_angular_velocities(frame_index);
    trns_bone_contacts = db.contact_states(frame_index);

    bone_positions = db.bone_positions(frame_index);
    bone_velocities = db.bone_velocities(frame_index);
    bone_rotations = db.bone_rotations(frame_index);
    bone_angular_velocities = db.bone_angular_velocities(frame_index);
    
    array1d<Vec3> positions(db.nbones());
    bone_offset_positions = positions;

    array1d<Vec3> velocities(db.nbones());
    bone_offset_velocities = velocities;

    array1d<quat> rotations(db.nbones());
    bone_offset_rotations = rotations;

    array1d<Vec3> angular_velocities(db.nbones());
    bone_offset_angular_velocities = angular_velocities;

    array1d<Vec3> global_positions(db.nbones());
    global_bone_positions = global_positions;

    array1d<Vec3> global_velocities(db.nbones());
    global_bone_velocities = global_velocities;

    array1d<quat> global_rotations(db.nbones());
    global_bone_rotations = global_rotations;

    array1d<Vec3> global_angular_velocities(db.nbones());
    global_bone_angular_velocities = global_angular_velocities;

    array1d<bool> global_computed(db.nbones());
    global_bone_computed = global_computed;

    Vec3 transition_src_position;
    quat transition_src_rotation;
    Vec3 transition_dst_position;
    quat transition_dst_rotation;


    inertialize_pose_reset(
        bone_offset_positions,
        bone_offset_velocities,
        bone_offset_rotations,
        bone_offset_angular_velocities,
        transition_src_position,
        transition_src_rotation,
        transition_dst_position,
        transition_dst_rotation,
        bone_positions(0),
        bone_rotations(0));

    inertialize_pose_update(
        bone_positions,
        bone_velocities,
        bone_rotations,
        bone_angular_velocities,
        bone_offset_positions,
        bone_offset_velocities,
        bone_offset_rotations,
        bone_offset_angular_velocities,
        db.bone_positions(frame_index),
        db.bone_velocities(frame_index),
        db.bone_rotations(frame_index),
        db.bone_angular_velocities(frame_index),
        transition_src_position,
        transition_src_rotation,
        transition_dst_position,
        transition_dst_rotation,
        inertialize_blending_halflife,
        0.0f);
    array1d<Vec3> vector(4);
    array1d<quat> rotation(4);
    trajectory_desired_velocities = vector;
    trajectory_desired_rotations = rotation;
    trajectory_positions = vector;
    trajectory_velocities = vector;
    trajectory_accelerations = vector;
    trajectory_rotations = rotation;
    trajectory_angular_velocities = vector;

    array1d<int> int_tow(2);
    contact_bones = int_tow;
    contact_bones(0) = Bone_LeftToe;
    contact_bones(1) = Bone_RightToe;

    array1d<bool> arrayBool(contact_bones.size);
    array1d<Vec3> vec3(contact_bones.size);
    contact_states = arrayBool;
    contact_locks = arrayBool;
    contact_positions = vec3;
    contact_velocities = vec3;
    contact_points = vec3;
    contact_targets = vec3;
    contact_offset_positions = vec3;
    contact_offset_velocities = vec3;

    for (int i = 0; i < contact_bones.size; i++)
    {
        Vec3 bone_position;
        Vec3 bone_velocity;
        quat bone_rotation;
        Vec3 bone_angular_velocity;

        forward_kinematics_velocity(
            bone_position,
            bone_velocity,
            bone_rotation,
            bone_angular_velocity,
            bone_positions,
            bone_velocities,
            bone_rotations,
            bone_angular_velocities,
            db.bone_parents,
            contact_bones(i));

        contact_reset(
            contact_states(i),
            contact_locks(i),
            contact_positions(i),
            contact_velocities(i),
            contact_points(i),
            contact_targets(i),
            contact_offset_positions(i),
            contact_offset_velocities(i),
            bone_position,
            bone_velocity,
            false);
    }

    adjusted_bone_positions = bone_positions;
    adjusted_bone_rotations = bone_rotations;
    m_isBuild = true;
}

void MotionMatching::Render(float deltaTime)
{
    const cd::SceneDatabase* pSceneDatabase = m_pCurrentSceneWorld->GetSceneDatabase();
    for (Entity entity : m_pCurrentSceneWorld->GetMotionMatchingEntities())
    {
        MotionMatchingComponent* pMotionMatchingComponent = m_pCurrentSceneWorld->GetMotionMatchingComponent(entity);
        if (!pMotionMatchingComponent)
        {
            continue;
        }

        if (!m_isBuild)
        {
            Build();
        }

        m_mesh = cd::MoveTemp(pMotionMatchingComponent->GetMesh());
        static int duration = 121;
        Vec3 destinationDir = Vec3(0, 0, 0);
        Vec3 obstacles_positions;
        if (duration > 120)
        {
            std::srand(std::time(0));

            float offsetX = std::rand() % 80 - (80 / 2);
            float offsetZ = std::rand() % 80 - (80 / 2);
            obstacles_positions = Vec3(simulation_position.x + offsetX, simulation_position.y, simulation_position.z + offsetZ);
            duration = 0;
        }
        duration++;

        destinationDir = normalize(obstacles_positions - simulation_position);
        float Length = length(obstacles_positions - simulation_position);
        Vec3 gamepadstick_left = gamepad_get_stick(GAMEPAD_STICK_LEFT);
        if (Length < 12)
        {
          //  gamepadstick_left = destinationDir * 0.1;
        }
        else if (Length > 20)
        {
          //  gamepadstick_left = destinationDir * 1.5;
        }

      //  gamepadstick_left = destinationDir;
       
        // Get the desired gait (walk / run)
        desired_gait_update(
            desired_gait,
            desired_gait_velocity,
            deltaTime);


        // Get if strafe is desired
        float simulation_fwrd_speed = lerpf(simulation_run_fwrd_speed, simulation_walk_fwrd_speed, desired_gait);
        float simulation_side_speed = lerpf(simulation_run_side_speed, simulation_walk_side_speed, desired_gait);
        float simulation_back_speed = lerpf(simulation_run_back_speed, simulation_walk_back_speed, desired_gait);

        // Get the desired velocity
        Vec3 desired_velocity_curr = desired_velocity_update(
            gamepadstick_left,
            0.0f,
            simulation_rotation,
            simulation_fwrd_speed,
            simulation_side_speed,
            simulation_back_speed);

        // Get the desired rotation/direction
        quat desired_rotation_curr = desired_rotation_update(
            desired_rotation,
            gamepadstick_left,
            Vec3(0.0f, 0.0f, 0.0f),
            0.0f,
            false,
            desired_velocity_curr);


        desired_rotation_change_prev = desired_rotation_change_curr;
        desired_rotation_change_curr = quat_to_scaled_angle_axis(quat_abs(quat_mul_inv(desired_rotation_curr, desired_rotation))) / deltaTime;
        desired_rotation = desired_rotation_curr;

        desired_rotation_change_prev = desired_rotation_change_curr;
        desired_rotation_change_curr = quat_to_scaled_angle_axis(quat_abs(quat_mul_inv(desired_rotation_curr, desired_rotation))) / deltaTime;
        desired_rotation = desired_rotation_curr;

        bool force_search = false;

        if (force_search_timer <= 0.0f)
        {
            force_search = true;
            force_search_timer = search_time;
        }
        else if (force_search_timer > 0)
        {
            force_search_timer -= deltaTime;
        }

        // Predict Future Trajectory

        trajectory_desired_rotations_predict(
            trajectory_desired_rotations,
            trajectory_desired_velocities,
            desired_rotation,
            0.0f,
            gamepadstick_left,
            Vec3(0.0f, 0.0f, 0.0f),
            false,
            20.0f * deltaTime);

        trajectory_rotations_predict(
            trajectory_rotations,
            trajectory_angular_velocities,
            simulation_rotation,
            simulation_angular_velocity,
            trajectory_desired_rotations,
            simulation_rotation_halflife,
            20.0f * deltaTime);

        trajectory_desired_velocities_predict(
            trajectory_desired_velocities,
            trajectory_rotations,
            desired_velocity,
            0.0f,
            gamepadstick_left,
            Vec3(0.0f, 0.0f, 0.0f),
            false,
            simulation_fwrd_speed,
            simulation_side_speed,
            simulation_back_speed,
            20.0f * deltaTime);

        trajectory_positions_predict(
            trajectory_positions,
            trajectory_velocities,
            trajectory_accelerations,
            simulation_position,
            simulation_velocity,
            simulation_acceleration,
            trajectory_desired_velocities,
            simulation_velocity_halflife,
            20.0f * deltaTime
        );

        // Make query vector for search.
           // In theory this only needs to be done when a search is 
           // actually required however for visualization purposes it
           // can be nice to do it every frame
        array1d<float> query(db.nfeatures());

        // Compute the features of the query vector

        slice1d<float> query_features = db.features(frame_index);

        int offset = 0;
        query_copy_denormalized_feature(query, offset, 3, query_features, db.features_offset, db.features_scale); // Left Foot Position
        query_copy_denormalized_feature(query, offset, 3, query_features, db.features_offset, db.features_scale); // Right Foot Position
        query_copy_denormalized_feature(query, offset, 3, query_features, db.features_offset, db.features_scale); // Left Foot Velocity
        query_copy_denormalized_feature(query, offset, 3, query_features, db.features_offset, db.features_scale); // Right Foot Velocity
        query_copy_denormalized_feature(query, offset, 3, query_features, db.features_offset, db.features_scale); // Hip Velocity
        query_compute_trajectory_position_feature(query, offset, bone_positions(0), bone_rotations(0), trajectory_positions);
        query_compute_trajectory_direction_feature(query, offset, bone_rotations(0), trajectory_rotations);

        assert(offset == db.nfeatures());
      


        // Check if we reached the end of the current anim
        bool end_of_anim = database_trajectory_index_clamp(db, frame_index, 1) == frame_index;

        // Do we need to search?
        if (force_search || search_timer <= 0.0f || end_of_anim)
        {
            {
                // Search

                int best_index = end_of_anim ? -1 : frame_index;
                float best_cost = FLT_MAX;

                database_search(
                    best_index,
                    best_cost,
                    db,
                    query);

                // Transition if better frame found

                if (best_index != frame_index)
                {
                    trns_bone_positions = db.bone_positions(best_index);
                    trns_bone_velocities = db.bone_velocities(best_index);
                    trns_bone_rotations = db.bone_rotations(best_index);
                    trns_bone_angular_velocities = db.bone_angular_velocities(best_index);

                    inertialize_pose_transition(
                        bone_offset_positions,
                        bone_offset_velocities,
                        bone_offset_rotations,
                        bone_offset_angular_velocities,
                        transition_src_position,
                        transition_src_rotation,
                        transition_dst_position,
                        transition_dst_rotation,
                        bone_positions(0),
                        bone_velocities(0),
                        bone_rotations(0),
                        bone_angular_velocities(0),
                        curr_bone_positions,
                        curr_bone_velocities,
                        curr_bone_rotations,
                        curr_bone_angular_velocities,
                        trns_bone_positions,
                        trns_bone_velocities,
                        trns_bone_rotations,
                        trns_bone_angular_velocities);

                    frame_index = best_index;
                }
            }

            // Reset search timer
            search_timer = search_time;
        }

        // Tick down search timer
        search_timer -= deltaTime;

        // Tick frame
        frame_index++; // Assumes dt is fixed to 60fps

        // Look-up Next Pose
        curr_bone_positions = db.bone_positions(frame_index);
        curr_bone_velocities = db.bone_velocities(frame_index);
        curr_bone_rotations = db.bone_rotations(frame_index);
        curr_bone_angular_velocities = db.bone_angular_velocities(frame_index);
        curr_bone_contacts = db.contact_states(frame_index);

        // Update inertializer

        inertialize_pose_update(
            bone_positions,
            bone_velocities,
            bone_rotations,
            bone_angular_velocities,
            bone_offset_positions,
            bone_offset_velocities,
            bone_offset_rotations,
            bone_offset_angular_velocities,
            curr_bone_positions,
            curr_bone_velocities,
            curr_bone_rotations,
            curr_bone_angular_velocities,
            transition_src_position,
            transition_src_rotation,
            transition_dst_position,
            transition_dst_rotation,
            inertialize_blending_halflife,
            deltaTime);

        // Update Simulation
        cd::VertexFormat vertexFormat;
        vertexFormat.AddVertexAttributeLayout(cd::VertexAttributeType::Position, cd::AttributeValueType::Float, 3);

        const uint32_t vertexCount = m_mesh.vertexCount;
        m_vertexBuffer.resize(vertexCount * vertexFormat.GetStride());
        m_indexBuffer.resize(m_character.triangles.size * sizeof(uint16_t));

        Vec3 simulation_position_prev = simulation_position;

        simulation_positions_update(
            simulation_position,
            simulation_velocity,
            simulation_acceleration,
            desired_velocity,
            simulation_velocity_halflife,
            deltaTime);

        simulation_rotations_update(
            simulation_rotation,
            simulation_angular_velocity,
            desired_rotation,
            simulation_rotation_halflife,
            deltaTime);

        adjusted_bone_positions = bone_positions;
        adjusted_bone_rotations = bone_rotations;

        forward_kinematics_full(
            global_bone_positions,
            global_bone_rotations,
            adjusted_bone_positions,
            adjusted_bone_rotations,
            db.bone_parents);

        deform_character_mesh(
            m_mesh,
            m_character,
            global_bone_positions,
            global_bone_rotations,
            db.bone_parents);

        slice1d<Vec3> anim_positions = slice1d<Vec3>(m_mesh.vertexCount, (Vec3*)m_mesh.vertices);
        uint32_t currentDataSize = 0U;
        ;
        auto currentDataPtr = m_vertexBuffer.data();
        for (uint32_t vertexIndex = 0; vertexIndex < vertexCount; ++vertexIndex)
        {
            const cd::Point& position = cd::Point(anim_positions.data[vertexIndex].x, anim_positions.data[vertexIndex].y, anim_positions.data[vertexIndex].z);
            uint32_t posDataSize = cd::Point::Size * sizeof(cd::Point::ValueType);
            std::memcpy(&currentDataPtr[currentDataSize], position.begin(), posDataSize);
            currentDataSize += posDataSize;
        }

        uint32_t ibDataSize = 0U;
        auto ibDataPtr = m_indexBuffer.data();
        auto FillIndexBuffer = [&ibDataPtr, &ibDataSize](const void* pData, uint32_t dataSize)
            {
                std::memcpy(&ibDataPtr[ibDataSize], pData, dataSize);
                ibDataSize += dataSize;
            };
        cd::PolygonGroup polygonGroup;
        for (uint32_t i = 0U; i < m_character.triangles.size / 3; ++i)
        {
            cd::Polygon polygon;
            polygon.push_back(m_character.triangles.data[i * 3]);
            polygon.push_back(m_character.triangles.data[i * 3 + 1]);
            polygon.push_back(m_character.triangles.data[i * 3 + 2]);
            polygonGroup.push_back(polygon);
        }
        uint32_t currentIndexDataSize = 0U;
        size_t indexTypeSize = sizeof(uint16_t);
        for (const auto& polygon : polygonGroup)
        {
            for (auto vertexIndex : polygon)
            {
                FillIndexBuffer(&vertexIndex, indexTypeSize);
            }
        }
   
   
        bgfx::VertexLayout vertexLayout;
        VertexLayoutUtility::CreateVertexLayout(vertexLayout, vertexFormat.GetVertexAttributeLayouts());
        m_VBH = bgfx::createVertexBuffer(bgfx::makeRef(m_vertexBuffer.data(), static_cast<uint32_t>(m_vertexBuffer.size())), vertexLayout).idx;
        m_IBH = bgfx::createIndexBuffer(bgfx::makeRef(m_indexBuffer.data(), static_cast<uint32_t>(m_indexBuffer.size())), 0U).idx;

        bgfx::setVertexBuffer(0, bgfx::VertexBufferHandle{ m_VBH });
        bgfx::setIndexBuffer(bgfx::IndexBufferHandle{ m_IBH });
        constexpr uint64_t state = BGFX_STATE_WRITE_MASK | BGFX_STATE_MSAA | BGFX_STATE_DEPTH_TEST_LESS |
            BGFX_STATE_BLEND_FUNC(BGFX_STATE_BLEND_SRC_ALPHA, BGFX_STATE_BLEND_INV_SRC_ALPHA);
        bgfx::setState(state);

        constexpr StringCrc programHandleIndex{ "WhiteModelProgram" };
        GetRenderContext()->Submit(GetViewID(), programHandleIndex);
      
    }
}

}