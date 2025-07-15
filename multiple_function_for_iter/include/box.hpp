/* box.hpp
 * Copyright (C) 2021 SS47816
 * Implementation of the Box struct
 **/

#pragma once

#include <Eigen/Geometry>

struct Box {
   public:
    int id;
    Eigen::Vector3f position;
    Eigen::Vector3f dimension;
    Eigen::Quaternionf quaternion;
    bool is_classified_as_car;  // Flag to mark if this box is a car
    bool is_classified_as_motor;

    Box() : id(-1),
            position(Eigen::Vector3f::Zero()),
            dimension(Eigen::Vector3f::Zero()),
            quaternion(Eigen::Quaternionf::Identity()),
            is_classified_as_car(false) {}

    Box(int box_id, Eigen::Vector3f pos, Eigen::Vector3f dim)
        : id(box_id),
          position(pos),
          dimension(dim),
          quaternion(Eigen::Quaternionf::Identity()),  // Default for AABB
          is_classified_as_car(false) {}

    Box(int box_id, Eigen::Vector3f pos, Eigen::Vector3f dim,
        Eigen::Quaternionf quat)
        : id(box_id),
          position(pos),
          dimension(dim),
          quaternion(quat),
          is_classified_as_car(false) {}
};
