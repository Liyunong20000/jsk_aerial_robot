#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>

class DownwardCamera
{
public:
    /**
     * @brief Configures a camera mounted pointing downwards.
     * * @param offset_B_C  Vector (x,y,z) from Body Center (IMU) to Camera Lens in Body Frame.
     * @param z_rotation_deg Rotation around the Camera's Optical Z-axis (in degrees).
     * 0.0 deg means: Top of Image points to Body Forward (+X).
     * 90.0 deg means: Top of Image points to Body Right (-Y).
     */
    DownwardCamera(const Eigen::Vector3d& offset_B_C, double z_rotation_deg)
    {
        // 1. Define the "Base" Downward orientation (Standard Optical Frame w.r.t Body Frame)
        //    Standard Optical: Z=Forward (into scene), X=Right, Y=Down
        //    Standard Body: X=Forward, Y=Left, Z=Up
        //    
        //    Alignment for 0 degrees (Top of image = Body Forward):
        //    - Cam Z (Optical Axis) points to Body -Z (Down)
        //    - Cam Y (Image Down)   points to Body -X (Back)
        //    - Cam X (Image Right)  points to Body -Y (Right)
        
        Eigen::Matrix3d R_base;
        R_base.col(0) <<  0, -1,  0; // Cam X axis in Body Frame
        R_base.col(1) << -1,  0,  0; // Cam Y axis in Body Frame
        R_base.col(2) <<  0,  0, -1; // Cam Z axis in Body Frame

        // 2. Apply user defined rotation around the Z-axis (Optical Axis)
        double yaw_rad = z_rotation_deg * M_PI / 180.0;
        Eigen::AngleAxisd user_rot(yaw_rad, Eigen::Vector3d::UnitZ());

        // Final Rotation R_B_C = R_base * R_user
        // (We rotate the intrinsic frame around its own Z)
        Eigen::Matrix3d R_final = R_base * user_rot.toRotationMatrix();

        // 3. Construct Affine Transform T_B_C (Camera in Body)
        T_B_C_ = Eigen::Affine3d::Identity();
        T_B_C_.linear() = R_final;
        T_B_C_.translation() = offset_B_C;
        
        // 4. Precompute Inverse T_C_B (Body in Camera)
        T_C_B_ = T_B_C_.inverse();
    }

    /**
     * @brief Get the transformation from Camera Frame to Body Frame (Extrinsics)
     * Use this to transform a point from Camera to Body: P_body = T_B_C * P_cam
     */
    const Eigen::Affine3d& getExtrinsics_T_B_C() const {
        return T_B_C_;
    }

    /**
     * @brief Get the transformation from Body Frame to Camera Frame (Inverse Extrinsics)
     */
    const Eigen::Affine3d& getInverseExtrinsics_T_C_B() const {
        return T_C_B_;
    }

    /**
     * @brief Solves for the Robot Body Pose in World Frame (T_W_B) given a tag measurement.
     * * @param T_W_Tag      Known pose of the AprilTag in World Frame.
     * @param meas_T_C_Tag The measured pose of the Tag in Camera Frame (from ROS topic).
     * @return Eigen::Affine3d The calculated pose of the Body (IMU) in World Frame.
     */
    Eigen::Affine3d solveBodyPose(const Eigen::Affine3d& T_W_Tag, 
                                  const Eigen::Affine3d& meas_T_C_Tag) const
    {
        // Chain: T_W_Tag = T_W_B * T_B_C * T_C_Tag
        // We want T_W_B.
        // T_W_B = T_W_Tag * (T_B_C * T_C_Tag)^-1
        // T_W_B = T_W_Tag * T_C_Tag^-1 * T_B_C^-1
        // T_W_B = T_W_Tag * T_C_Tag^-1 * T_C_B
        
        return T_W_Tag * meas_T_C_Tag.inverse() * T_C_B_;
    }

private:
    Eigen::Affine3d T_B_C_; // Camera pose in Body Frame
    Eigen::Affine3d T_C_B_; // Body pose in Camera Frame
};
