#pragma once

#include "Renderer.h"
#include "MotionMatching/character.h"
#include <MotionMatching/spring.h>
#include <MotionMatching/database.h>
#include <vector>


namespace engine
{

class SceneWorld;

class MotionMatching final : public Renderer
{
public:
	using Renderer::Renderer;

	virtual void Init() override;
	virtual void UpdateView(const float* pViewMatrix, const float* pProjectionMatrix) override;
	virtual void Render(float deltaTime) override;
	void Build();
	void SetSceneWorld(SceneWorld* pSceneWorld) { m_pCurrentSceneWorld = pSceneWorld; }

private:
    std::vector<std::byte> m_vertexBuffer;
    std::vector<std::byte> m_indexBuffer;
    uint16_t m_VBH = UINT16_MAX;
    uint16_t m_IBH = UINT16_MAX;
    character m_character;
    Mesh m_mesh;
    bool m_isBuild = false;
	SceneWorld* m_pCurrentSceneWorld = nullptr;

	uint16_t m_boneVBH = UINT16_MAX;
	uint16_t m_boneIBH = UINT16_MAX;
	bool hasBuilt = false;
    int frame_index;
    float inertialize_blending_halflife;
    array1d<Vec3> curr_bone_positions;
    array1d<Vec3> curr_bone_velocities;
    array1d<quat> curr_bone_rotations;
    array1d<Vec3> curr_bone_angular_velocities;
    array1d<bool> curr_bone_contacts;

    array1d<Vec3> trns_bone_positions;
    array1d<Vec3> trns_bone_velocities;
    array1d<quat> trns_bone_rotations;
    array1d<Vec3> trns_bone_angular_velocities;
    array1d<bool> trns_bone_contacts;

    array1d<Vec3> bone_positions;
    array1d<Vec3> bone_velocities;
    array1d<quat> bone_rotations;
    array1d<Vec3> bone_angular_velocities;

    array1d<Vec3> bone_offset_positions;
    array1d<Vec3> bone_offset_velocities;
    array1d<quat> bone_offset_rotations;
    array1d<Vec3> bone_offset_angular_velocities;

    array1d<Vec3> global_bone_positions;
    array1d<Vec3> global_bone_velocities;
    array1d<quat> global_bone_rotations;
    array1d<Vec3> global_bone_angular_velocities;
    array1d<bool> global_bone_computed;

    Vec3 transition_src_position;
    quat transition_src_rotation;
    Vec3 transition_dst_position;
    quat transition_dst_rotation;

    float search_time = 0.1f;
    float search_timer = search_time;
    float force_search_timer = search_time;

    Vec3 desired_velocity;
    Vec3 desired_velocity_change_curr;
    Vec3 desired_velocity_change_prev;
    float desired_velocity_change_threshold = 50.0;

    quat desired_rotation;
    Vec3 desired_rotation_change_curr;
    Vec3 desired_rotation_change_prev;
    float desired_rotation_change_threshold = 50.0;

    float desired_gait = 0.0f;
    float desired_gait_velocity = 0.0f;

    Vec3 simulation_position;
    Vec3 simulation_velocity;
    Vec3 simulation_acceleration;
    quat simulation_rotation;
    Vec3 simulation_angular_velocity;

    float simulation_velocity_halflife = 0.27f;
    float simulation_rotation_halflife = 0.27f;

    // All speeds in m/s
    float simulation_run_fwrd_speed = 4.0f;
    float simulation_run_side_speed = 3.0f;
    float simulation_run_back_speed = 2.5f;

    float simulation_walk_fwrd_speed = 1.75f;
    float simulation_walk_side_speed = 1.5f;
    float simulation_walk_back_speed = 1.25f;

    array1d<Vec3> trajectory_desired_velocities;
    array1d<quat> trajectory_desired_rotations;
    array1d<Vec3> trajectory_positions;
    array1d<Vec3> trajectory_velocities;
    array1d<Vec3> trajectory_accelerations;
    array1d<quat> trajectory_rotations;
    array1d<Vec3> trajectory_angular_velocities;

    // Synchronization

    bool synchronization_enabled = false;
    float synchronization_data_factor = 1.0f;

    // Adjustment

    bool adjustment_enabled = true;
    bool adjustment_by_velocity_enabled = true;
    float adjustment_position_halflife = 0.1f;
    float adjustment_rotation_halflife = 0.2f;
    float adjustment_position_max_ratio = 0.5f;
    float adjustment_rotation_max_ratio = 0.5f;

    // Clamping

    bool clamping_enabled = true;
    float clamping_max_distance = 0.15f;
    float clamping_max_angle = 0.5f * PIf;

    // IK

    bool ik_enabled = true;
    float ik_max_length_buffer = 0.015f;
    float ik_foot_height = 0.02f;
    float ik_toe_length = 0.15f;
    float ik_unlock_radius = 0.2f;
    float ik_blending_halflife = 0.1f;

    // Contact and Foot Locking data

    array1d<int> contact_bones;
    array1d<bool> contact_states;
    array1d<bool> contact_locks;
    array1d<Vec3> contact_positions;
    array1d<Vec3> contact_velocities;
    array1d<Vec3> contact_points;
    array1d<Vec3> contact_targets;
    array1d<Vec3> contact_offset_positions;
    array1d<Vec3> contact_offset_velocities;

    array1d<Vec3> adjusted_bone_positions;
    array1d<quat> adjusted_bone_rotations;

    database db;
};

}