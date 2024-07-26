#pragma once

#include "common.h"
#include "Vec.h"
#include "quat.h"
#include "array.h"

#include <assert.h>
#include <float.h>
#include <stdio.h>
#include <math.h>
#include "character.h"
#include "spring.h"

//--------------------------------------
namespace
{

enum
{
    BOUND_SM_SIZE = 16,
    BOUND_LR_SIZE = 64,
};

struct database
{
    array2d<Vec3> bone_positions;
    array2d<Vec3> bone_velocities;
    array2d<quat> bone_rotations;
    array2d<Vec3> bone_angular_velocities;
    array1d<int> bone_parents;

    array1d<int> range_starts;
    array1d<int> range_stops;

    array2d<float> features;
    array1d<float> features_offset;
    array1d<float> features_scale;

    array2d<bool> contact_states;

    array2d<float> bound_sm_min;
    array2d<float> bound_sm_max;
    array2d<float> bound_lr_min;
    array2d<float> bound_lr_max;

    int nframes() const { return bone_positions.rows; }
    int nbones() const { return bone_positions.cols; }
    int nranges() const { return range_starts.size; }
    int nfeatures() const { return features.cols; }
    int ncontacts() const { return contact_states.cols; }
};

void database_load(database& db, const char* filename)
{
    FILE* f = fopen(filename, "rb");
    assert(f != NULL);

    array2d_read(db.bone_positions, f);
    array2d_read(db.bone_velocities, f);
    array2d_read(db.bone_rotations, f);
    array2d_read(db.bone_angular_velocities, f);
    array1d_read(db.bone_parents, f);

    array1d_read(db.range_starts, f);
    array1d_read(db.range_stops, f);

    array2d_read(db.contact_states, f);

    fclose(f);
}

void database_save_matching_features(const database& db, const char* filename)
{
    FILE* f = fopen(filename, "wb");
    assert(f != NULL);

    array2d_write(db.features, f);
    array1d_write(db.features_offset, f);
    array1d_write(db.features_scale, f);

    fclose(f);
}

// When we add an offset to a frame in the database there is a chance
// it will go out of the relevant range so here we can clamp it to 
// the last frame of that range.
int database_trajectory_index_clamp(database& db, int frame, int offset)
{
    for (int i = 0; i < db.nranges(); i++)
    {
        if (frame >= db.range_starts(i) && frame < db.range_stops(i))
        {
            return clamp(frame + offset, db.range_starts(i), db.range_stops(i) - 1);
        }
    }

    assert(false);
    return -1;
}

//--------------------------------------

void normalize_feature(
    slice2d<float> features,
    slice1d<float> features_offset,
    slice1d<float> features_scale,
    const int offset,
    const int size,
    const float weight = 1.0f)
{
    // First compute what is essentially the mean 
    // value for each feature dimension
    for (int j = 0; j < size; j++)
    {
        features_offset(offset + j) = 0.0f;
    }

    for (int i = 0; i < features.rows; i++)
    {
        for (int j = 0; j < size; j++)
        {
            features_offset(offset + j) += features(i, offset + j) / features.rows;
        }
    }

    // Now compute the variance of each feature dimension
    array1d<float> vars(size);
    vars.zero();

    for (int i = 0; i < features.rows; i++)
    {
        for (int j = 0; j < size; j++)
        {
            vars(j) += squaref(features(i, offset + j) - features_offset(offset + j)) / features.rows;
        }
    }

    // We compute the overall std of the feature as the average
    // std across all dimensions
    float std = 0.0f;
    for (int j = 0; j < size; j++)
    {
        std += sqrtf(vars(j)) / size;
    }

    // Features with no variation can have zero std which is
    // almost always a bug.
    assert(std > 0.0);

    // The scale of a feature is just the std divided by the weight
    for (int j = 0; j < size; j++)
    {
        features_scale(offset + j) = std / weight;
    }

    // Using the offset and scale we can then normalize the features
    for (int i = 0; i < features.rows; i++)
    {
        for (int j = 0; j < size; j++)
        {
            features(i, offset + j) = (features(i, offset + j) - features_offset(offset + j)) / features_scale(offset + j);
        }
    }
}

void denormalize_features(
    slice1d<float> features,
    const slice1d<float> features_offset,
    const slice1d<float> features_scale)
{
    for (int i = 0; i < features.size; i++)
    {
        features(i) = (features(i) * features_scale(i)) + features_offset(i);
    }
}

//--------------------------------------

// Here I am using a simple recursive version of forward kinematics
void forward_kinematics(
    Vec3& bone_position,
    quat& bone_rotation,
    const slice1d<Vec3> bone_positions,
    const slice1d<quat> bone_rotations,
    const slice1d<int> bone_parents,
    const int bone)
{
    if (bone_parents(bone) != -1)
    {
        Vec3 parent_position;
        quat parent_rotation;

        forward_kinematics(
            parent_position,
            parent_rotation,
            bone_positions,
            bone_rotations,
            bone_parents,
            bone_parents(bone));

        bone_position = quat_mul_Vec3(parent_rotation, bone_positions(bone)) + parent_position;
        bone_rotation = quat_mul(parent_rotation, bone_rotations(bone));
    }
    else
    {
        bone_position = bone_positions(bone);
        bone_rotation = bone_rotations(bone);
    }
}

// Forward kinematics but also compute the velocities
void forward_kinematics_velocity(
    Vec3& bone_position,
    Vec3& bone_velocity,
    quat& bone_rotation,
    Vec3& bone_angular_velocity,
    const slice1d<Vec3> bone_positions,
    const slice1d<Vec3> bone_velocities,
    const slice1d<quat> bone_rotations,
    const slice1d<Vec3> bone_angular_velocities,
    const slice1d<int> bone_parents,
    const int bone)
{
    //
    if (bone_parents(bone) != -1)
    {
        Vec3 parent_position;
        Vec3 parent_velocity;
        quat parent_rotation;
        Vec3 parent_angular_velocity;

        forward_kinematics_velocity(
            parent_position,
            parent_velocity,
            parent_rotation,
            parent_angular_velocity,
            bone_positions,
            bone_velocities,
            bone_rotations,
            bone_angular_velocities,
            bone_parents,
            bone_parents(bone));

        bone_position = quat_mul_Vec3(parent_rotation, bone_positions(bone)) + parent_position;
        bone_velocity =
            parent_velocity +
            quat_mul_Vec3(parent_rotation, bone_velocities(bone)) +
            cross(parent_angular_velocity, quat_mul_Vec3(parent_rotation, bone_positions(bone)));
        bone_rotation = quat_mul(parent_rotation, bone_rotations(bone));
        bone_angular_velocity = quat_mul_Vec3(parent_rotation, bone_angular_velocities(bone)) + parent_angular_velocity;
    }
    else
    {
        bone_position = bone_positions(bone);
        bone_velocity = bone_velocities(bone);
        bone_rotation = bone_rotations(bone);
        bone_angular_velocity = bone_angular_velocities(bone);
    }
}

// Compute forward kinematics for all joints
void forward_kinematics_full(
    slice1d<Vec3> global_bone_positions,
    slice1d<quat> global_bone_rotations,
    const slice1d<Vec3> local_bone_positions,
    const slice1d<quat> local_bone_rotations,
    const slice1d<int> bone_parents)
{
    for (int i = 0; i < bone_parents.size; i++)
    {
        // Assumes bones are always sorted from root onwards
        assert(bone_parents(i) < i);

        if (bone_parents(i) == -1)
        {
            global_bone_positions(i) = local_bone_positions(i);
            global_bone_rotations(i) = local_bone_rotations(i);
        }
        else
        {
            Vec3 parent_position = global_bone_positions(bone_parents(i));
            quat parent_rotation = global_bone_rotations(bone_parents(i));
            global_bone_positions(i) = quat_mul_Vec3(parent_rotation, local_bone_positions(i)) + parent_position;
            global_bone_rotations(i) = quat_mul(parent_rotation, local_bone_rotations(i));
        }
    }
}

// Compute forward kinematics of just some joints using a
// mask to indicate which joints are already computed
void forward_kinematics_partial(
    slice1d<Vec3> global_bone_positions,
    slice1d<quat> global_bone_rotations,
    slice1d<bool> global_bone_computed,
    const slice1d<Vec3> local_bone_positions,
    const slice1d<quat> local_bone_rotations,
    const slice1d<int> bone_parents,
    int bone)
{
    if (bone_parents(bone) == -1)
    {
        global_bone_positions(bone) = local_bone_positions(bone);
        global_bone_rotations(bone) = local_bone_rotations(bone);
        global_bone_computed(bone) = true;
        return;
    }

    if (!global_bone_computed(bone_parents(bone)))
    {
        forward_kinematics_partial(
            global_bone_positions,
            global_bone_rotations,
            global_bone_computed,
            local_bone_positions,
            local_bone_rotations,
            bone_parents,
            bone_parents(bone));
    }

    Vec3 parent_position = global_bone_positions(bone_parents(bone));
    quat parent_rotation = global_bone_rotations(bone_parents(bone));
    global_bone_positions(bone) = quat_mul_Vec3(parent_rotation, local_bone_positions(bone)) + parent_position;
    global_bone_rotations(bone) = quat_mul(parent_rotation, local_bone_rotations(bone));
    global_bone_computed(bone) = true;
}

// Same but including velocity
void forward_kinematics_velocity_partial(
    slice1d<Vec3> global_bone_positions,
    slice1d<Vec3> global_bone_velocities,
    slice1d<quat> global_bone_rotations,
    slice1d<Vec3> global_bone_angular_velocities,
    slice1d<bool> global_bone_computed,
    const slice1d<Vec3> local_bone_positions,
    const slice1d<Vec3> local_bone_velocities,
    const slice1d<quat> local_bone_rotations,
    const slice1d<Vec3> local_bone_angular_velocities,
    const slice1d<int> bone_parents,
    int bone)
{
    if (bone_parents(bone) == -1)
    {
        global_bone_positions(bone) = local_bone_positions(bone);
        global_bone_velocities(bone) = local_bone_velocities(bone);
        global_bone_rotations(bone) = local_bone_rotations(bone);
        global_bone_angular_velocities(bone) = local_bone_angular_velocities(bone);
        global_bone_computed(bone) = true;
        return;
    }

    if (!global_bone_computed(bone_parents(bone)))
    {
        forward_kinematics_velocity_partial(
            global_bone_positions,
            global_bone_velocities,
            global_bone_rotations,
            global_bone_angular_velocities,
            global_bone_computed,
            local_bone_positions,
            local_bone_velocities,
            local_bone_rotations,
            local_bone_angular_velocities,
            bone_parents,
            bone_parents(bone));
    }

    Vec3 parent_position = global_bone_positions(bone_parents(bone));
    Vec3 parent_velocity = global_bone_velocities(bone_parents(bone));
    quat parent_rotation = global_bone_rotations(bone_parents(bone));
    Vec3 parent_angular_velocity = global_bone_angular_velocities(bone_parents(bone));

    global_bone_positions(bone) = quat_mul_Vec3(parent_rotation, local_bone_positions(bone)) + parent_position;
    global_bone_velocities(bone) =
        parent_velocity +
        quat_mul_Vec3(parent_rotation, local_bone_velocities(bone)) +
        cross(parent_angular_velocity, quat_mul_Vec3(parent_rotation, local_bone_positions(bone)));
    global_bone_rotations(bone) = quat_mul(parent_rotation, local_bone_rotations(bone));
    global_bone_angular_velocities(bone) = quat_mul_Vec3(parent_rotation, local_bone_angular_velocities(bone)) + parent_angular_velocity;
    global_bone_computed(bone) = true;
}

//--------------------------------------

// Compute a feature for the position of a bone relative to the simulation/root bone
void compute_bone_position_feature(database& db, int& offset, int bone, float weight = 1.0f)
{
    for (int i = 0; i < db.nframes(); i++)
    {
        Vec3 bone_position;
        quat bone_rotation;

        forward_kinematics(
            bone_position,
            bone_rotation,
            db.bone_positions(i),
            db.bone_rotations(i),
            db.bone_parents,
            bone);

        bone_position = quat_mul_Vec3(quat_inv(db.bone_rotations(i, 0)), bone_position - db.bone_positions(i, 0));

        db.features(i, offset + 0) = bone_position.x;
        db.features(i, offset + 1) = bone_position.y;
        db.features(i, offset + 2) = bone_position.z;
    }

    normalize_feature(db.features, db.features_offset, db.features_scale, offset, 3, weight);

    offset += 3;
}

// Similar but for a bone's velocity
void compute_bone_velocity_feature(database& db, int& offset, int bone, float weight = 1.0f)
{
    for (int i = 0; i < db.nframes(); i++)
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
            db.bone_positions(i),
            db.bone_velocities(i),
            db.bone_rotations(i),
            db.bone_angular_velocities(i),
            db.bone_parents,
            bone);

        bone_velocity = quat_mul_Vec3(quat_inv(db.bone_rotations(i, 0)), bone_velocity);

        db.features(i, offset + 0) = bone_velocity.x;
        db.features(i, offset + 1) = bone_velocity.y;
        db.features(i, offset + 2) = bone_velocity.z;
    }

    normalize_feature(db.features, db.features_offset, db.features_scale, offset, 3, weight);

    offset += 3;
}

// Compute the trajectory at 20, 40, and 60 frames in the future
void compute_trajectory_position_feature(database& db, int& offset, float weight = 1.0f)
{
    for (int i = 0; i < db.nframes(); i++)
    {
        int t0 = database_trajectory_index_clamp(db, i, 20);
        int t1 = database_trajectory_index_clamp(db, i, 40);
        int t2 = database_trajectory_index_clamp(db, i, 60);

        Vec3 trajectory_pos0 = quat_mul_Vec3(quat_inv(db.bone_rotations(i, 0)), db.bone_positions(t0, 0) - db.bone_positions(i, 0));
        Vec3 trajectory_pos1 = quat_mul_Vec3(quat_inv(db.bone_rotations(i, 0)), db.bone_positions(t1, 0) - db.bone_positions(i, 0));
        Vec3 trajectory_pos2 = quat_mul_Vec3(quat_inv(db.bone_rotations(i, 0)), db.bone_positions(t2, 0) - db.bone_positions(i, 0));

        db.features(i, offset + 0) = trajectory_pos0.x;
        db.features(i, offset + 1) = trajectory_pos0.z;
        db.features(i, offset + 2) = trajectory_pos1.x;
        db.features(i, offset + 3) = trajectory_pos1.z;
        db.features(i, offset + 4) = trajectory_pos2.x;
        db.features(i, offset + 5) = trajectory_pos2.z;
    }

    normalize_feature(db.features, db.features_offset, db.features_scale, offset, 6, weight);

    offset += 6;
}

// Same for direction
void compute_trajectory_direction_feature(database& db, int& offset, float weight = 1.0f)
{
    for (int i = 0; i < db.nframes(); i++)
    {
        int t0 = database_trajectory_index_clamp(db, i, 20);
        int t1 = database_trajectory_index_clamp(db, i, 40);
        int t2 = database_trajectory_index_clamp(db, i, 60);

        Vec3 trajectory_dir0 = quat_mul_Vec3(quat_inv(db.bone_rotations(i, 0)), quat_mul_Vec3(db.bone_rotations(t0, 0), Vec3(0, 0, 1)));
        Vec3 trajectory_dir1 = quat_mul_Vec3(quat_inv(db.bone_rotations(i, 0)), quat_mul_Vec3(db.bone_rotations(t1, 0), Vec3(0, 0, 1)));
        Vec3 trajectory_dir2 = quat_mul_Vec3(quat_inv(db.bone_rotations(i, 0)), quat_mul_Vec3(db.bone_rotations(t2, 0), Vec3(0, 0, 1)));

        db.features(i, offset + 0) = trajectory_dir0.x;
        db.features(i, offset + 1) = trajectory_dir0.z;
        db.features(i, offset + 2) = trajectory_dir1.x;
        db.features(i, offset + 3) = trajectory_dir1.z;
        db.features(i, offset + 4) = trajectory_dir2.x;
        db.features(i, offset + 5) = trajectory_dir2.z;
    }

    normalize_feature(db.features, db.features_offset, db.features_scale, offset, 6, weight);

    offset += 6;
}

// Build the Motion Matching search acceleration structure. Here we
// just use axis aligned bounding boxes regularly spaced at BOUND_SM_SIZE
// and BOUND_LR_SIZE frames
void database_build_bounds(database& db)
{
    int nbound_sm = ((db.nframes() + BOUND_SM_SIZE - 1) / BOUND_SM_SIZE);
    int nbound_lr = ((db.nframes() + BOUND_LR_SIZE - 1) / BOUND_LR_SIZE);

    db.bound_sm_min.resize(nbound_sm, db.nfeatures());
    db.bound_sm_max.resize(nbound_sm, db.nfeatures());
    db.bound_lr_min.resize(nbound_lr, db.nfeatures());
    db.bound_lr_max.resize(nbound_lr, db.nfeatures());

    db.bound_sm_min.set(+FLT_MAX);
    db.bound_sm_max.set(-FLT_MAX);
    db.bound_lr_min.set(+FLT_MAX);
    db.bound_lr_max.set(-FLT_MAX);

    for (int i = 0; i < db.nframes(); i++)
    {
        int i_sm = i / BOUND_SM_SIZE;
        int i_lr = i / BOUND_LR_SIZE;

        for (int j = 0; j < db.nfeatures(); j++)
        {
            db.bound_sm_min(i_sm, j) = minf(db.bound_sm_min(i_sm, j), db.features(i, j));
            db.bound_sm_max(i_sm, j) = maxf(db.bound_sm_max(i_sm, j), db.features(i, j));
            db.bound_lr_min(i_lr, j) = minf(db.bound_lr_min(i_lr, j), db.features(i, j));
            db.bound_lr_max(i_lr, j) = maxf(db.bound_lr_max(i_lr, j), db.features(i, j));
        }
    }
}

// Build all motion matching features and acceleration structure
void database_build_matching_features(
    database& db,
    const float feature_weight_foot_position,
    const float feature_weight_foot_velocity,
    const float feature_weight_hip_velocity,
    const float feature_weight_trajectory_positions,
    const float feature_weight_trajectory_directions)
{
    int nfeatures =
        3 + // Left Foot Position
        3 + // Right Foot Position 
        3 + // Left Foot Velocity
        3 + // Right Foot Velocity
        3 + // Hip Velocity
        6 + // Trajectory Positions 2D
        6; // Trajectory Directions 2D

    db.features.resize(db.nframes(), nfeatures);
    db.features_offset.resize(nfeatures);
    db.features_scale.resize(nfeatures);

    int offset = 0;
    compute_bone_position_feature(db, offset, Bone_LeftFoot, feature_weight_foot_position);
    compute_bone_position_feature(db, offset, Bone_RightFoot, feature_weight_foot_position);
    compute_bone_velocity_feature(db, offset, Bone_LeftFoot, feature_weight_foot_velocity);
    compute_bone_velocity_feature(db, offset, Bone_RightFoot, feature_weight_foot_velocity);
    compute_bone_velocity_feature(db, offset, Bone_Hips, feature_weight_hip_velocity);
    compute_trajectory_position_feature(db, offset, feature_weight_trajectory_positions);
    compute_trajectory_direction_feature(db, offset, feature_weight_trajectory_directions);

    assert(offset == nfeatures);

    database_build_bounds(db);
}

// Motion Matching search function essentially consists
// of comparing every feature Vector in the database, 
// against the query feature Vector, first checking the 
// query distance to the axis aligned bounding boxes used 
// for the acceleration structure.
void motion_matching_search(
    int& best_index,
    float& best_cost,
    const slice1d<int> range_starts,
    const slice1d<int> range_stops,
    const slice2d<float> features,
    const slice1d<float> features_offset,
    const slice1d<float> features_scale,
    const slice2d<float> bound_sm_min,
    const slice2d<float> bound_sm_max,
    const slice2d<float> bound_lr_min,
    const slice2d<float> bound_lr_max,
    const slice1d<float> query_normalized,
    const float transition_cost,
    const int ignore_range_end,
    const int ignore_surrounding)
{
    int nfeatures = query_normalized.size;
    int nranges = range_starts.size;

    int curr_index = best_index;

    // Find cost for current frame
    if (best_index != -1)
    {
        best_cost = 0.0;
        for (int i = 0; i < nfeatures; i++)
        {
            best_cost += squaref(query_normalized(i) - features(best_index, i));
        }
    }

    float curr_cost = 0.0f;

    // Search rest of database
    for (int r = 0; r < nranges; r++)
    {
        // Exclude end of ranges from search    
        int i = range_starts(r);
        int range_end = range_stops(r) - ignore_range_end;

        while (i < range_end)
        {
            // Find index of current and next large box
            int i_lr = i / BOUND_LR_SIZE;
            int i_lr_next = (i_lr + 1) * BOUND_LR_SIZE;

            // Find distance to box
            curr_cost = transition_cost;
            for (int j = 0; j < nfeatures; j++)
            {
                curr_cost += squaref(query_normalized(j) - clampf(query_normalized(j),
                    bound_lr_min(i_lr, j), bound_lr_max(i_lr, j)));

                if (curr_cost >= best_cost)
                {
                    break;
                }
            }

            // If distance is greater than current best jump to next box
            if (curr_cost >= best_cost)
            {
                i = i_lr_next;
                continue;
            }

            // Check against small box
            while (i < i_lr_next && i < range_end)
            {
                // Find index of current and next small box
                int i_sm = i / BOUND_SM_SIZE;
                int i_sm_next = (i_sm + 1) * BOUND_SM_SIZE;

                // Find distance to box
                curr_cost = transition_cost;
                for (int j = 0; j < nfeatures; j++)
                {
                    curr_cost += squaref(query_normalized(j) - clampf(query_normalized(j),
                        bound_sm_min(i_sm, j), bound_sm_max(i_sm, j)));

                    if (curr_cost >= best_cost)
                    {
                        break;
                    }
                }

                // If distance is greater than current best jump to next box
                if (curr_cost >= best_cost)
                {
                    i = i_sm_next;
                    continue;
                }

                // Search inside small box
                while (i < i_sm_next && i < range_end)
                {
                    // Skip surrounding frames
                    if (curr_index != -1 && abs(i - curr_index) < ignore_surrounding)
                    {
                        i++;
                        continue;
                    }

                    // Check against each frame inside small box
                    curr_cost = transition_cost;
                    for (int j = 0; j < nfeatures; j++)
                    {
                        curr_cost += squaref(query_normalized(j) - features(i, j));
                        if (curr_cost >= best_cost)
                        {
                            break;
                        }
                    }

                    // If cost is lower than current best then update best
                    if (curr_cost < best_cost)
                    {
                        best_index = i;
                        best_cost = curr_cost;
                    }

                    i++;
                }
            }
        }
    }
}

// Search database
void database_search(
    int& best_index,
    float& best_cost,
    const database& db,
    const slice1d<float> query,
    const float transition_cost = 0.0f,
    const int ignore_range_end = 20,
    const int ignore_surrounding = 20)
{
    // Normalize Query
    array1d<float> query_normalized(db.nfeatures());
    for (int i = 0; i < db.nfeatures(); i++)
    {
        query_normalized(i) = (query(i) - db.features_offset(i)) / db.features_scale(i);
    }

    // Search
    motion_matching_search(
        best_index,
        best_cost,
        db.range_starts,
        db.range_stops,
        db.features,
        db.features_offset,
        db.features_scale,
        db.bound_sm_min,
        db.bound_sm_max,
        db.bound_lr_min,
        db.bound_lr_max,
        query_normalized,
        transition_cost,
        ignore_range_end,
        ignore_surrounding);
}

void inertialize_pose_reset(
    slice1d<Vec3> bone_offset_positions,
    slice1d<Vec3> bone_offset_velocities,
    slice1d<quat> bone_offset_rotations,
    slice1d<Vec3> bone_offset_angular_velocities,
    Vec3& transition_src_position,
    quat& transition_src_rotation,
    Vec3& transition_dst_position,
    quat& transition_dst_rotation,
    const Vec3 root_position,
    const quat root_rotation)
{
    bone_offset_positions.zero();
    bone_offset_velocities.zero();
    bone_offset_rotations.set(quat());
    bone_offset_angular_velocities.zero();

    transition_src_position = root_position;
    transition_src_rotation = root_rotation;
    transition_dst_position = Vec3();
    transition_dst_rotation = quat();
}

void contact_reset(
    bool& contact_state,
    bool& contact_lock,
    Vec3& contact_position,
    Vec3& contact_velocity,
    Vec3& contact_point,
    Vec3& contact_target,
    Vec3& contact_offset_position,
    Vec3& contact_offset_velocity,
    const Vec3 input_contact_position,
    const Vec3 input_contact_velocity,
    const bool input_contact_state)
{
    contact_state = false;
    contact_lock = false;
    contact_position = input_contact_position;
    contact_velocity = input_contact_velocity;
    contact_point = input_contact_position;
    contact_target = input_contact_position;
    contact_offset_position = Vec3();
    contact_offset_velocity = Vec3();
}

// This function updates the inertializer states. Here 
// it outputs the smoothed animation (input plus offset) 
// as well as updating the offsets themselves. It takes 
// as input the current playing animation as well as the 
// root transition locations, a halflife, and a dt
void inertialize_pose_update(
    slice1d<Vec3> bone_positions,
    slice1d<Vec3> bone_velocities,
    slice1d<quat> bone_rotations,
    slice1d<Vec3> bone_angular_velocities,
    slice1d<Vec3> bone_offset_positions,
    slice1d<Vec3> bone_offset_velocities,
    slice1d<quat> bone_offset_rotations,
    slice1d<Vec3> bone_offset_angular_velocities,
    const slice1d<Vec3> bone_input_positions,
    const slice1d<Vec3> bone_input_velocities,
    const slice1d<quat> bone_input_rotations,
    const slice1d<Vec3> bone_input_angular_velocities,
    const Vec3 transition_src_position,
    const quat transition_src_rotation,
    const Vec3 transition_dst_position,
    const quat transition_dst_rotation,
    const float halflife,
    const float dt)
{
    // First we find the next root position, velocity, rotation
    // and rotational velocity in the world space by transforming 
    // the input animation from it's animation space into the 
    // space of the currently playing animation.
    Vec3 world_space_position = quat_mul_Vec3(transition_dst_rotation,
        quat_inv_mul_Vec3(transition_src_rotation,
            bone_input_positions(0) - transition_src_position)) + transition_dst_position;

    Vec3 world_space_velocity = quat_mul_Vec3(transition_dst_rotation,
        quat_inv_mul_Vec3(transition_src_rotation, bone_input_velocities(0)));

    // Normalize here because quat inv mul can sometimes produce 
    // unstable returns when the two rotations are very close.
    quat world_space_rotation = quat_normalize(quat_mul(transition_dst_rotation,
        quat_inv_mul(transition_src_rotation, bone_input_rotations(0))));

    Vec3 world_space_angular_velocity = quat_mul_Vec3(transition_dst_rotation,
        quat_inv_mul_Vec3(transition_src_rotation, bone_input_angular_velocities(0)));

    // Then we update these two inertializers with these new world space inputs
    inertialize_update(
        bone_positions(0),
        bone_velocities(0),
        bone_offset_positions(0),
        bone_offset_velocities(0),
        world_space_position,
        world_space_velocity,
        halflife,
        dt);

    inertialize_update(
        bone_rotations(0),
        bone_angular_velocities(0),
        bone_offset_rotations(0),
        bone_offset_angular_velocities(0),
        world_space_rotation,
        world_space_angular_velocity,
        halflife,
        dt);

    // Then we update the inertializers for the rest of the bones
    for (int i = 1; i < bone_positions.size; i++)
    {
        inertialize_update(
            bone_positions(i),
            bone_velocities(i),
            bone_offset_positions(i),
            bone_offset_velocities(i),
            bone_input_positions(i),
            bone_input_velocities(i),
            halflife,
            dt);

        inertialize_update(
            bone_rotations(i),
            bone_angular_velocities(i),
            bone_offset_rotations(i),
            bone_offset_angular_velocities(i),
            bone_input_rotations(i),
            bone_input_angular_velocities(i),
            halflife,
            dt);
    }
}
}
