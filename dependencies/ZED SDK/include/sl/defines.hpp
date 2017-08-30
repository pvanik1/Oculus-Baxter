/*
* SOFTWARE LICENSE
* BY USING YOUR ZED CAMERA YOU AGREE TO THIS SOFTWARE LICENSE. BEFORE SETTING IT UP,
* PLEASE READ THIS SOFTWARE LICENSE CAREFULLY. IF YOU DO NOT ACCEPT THIS
* SOFTWARE LICENSE, DO NOT USE YOUR ZED CAMERA. RETURN IT TO UNUSED TO
* STEREOLABS FOR A REFUND. Contact STEREOLABS at contact@stereolabs.com
*
* 1. Definitions
*
* "Authorized Accessory" means a STEREOLABS branded ZED, and a STEREOLABS
* licensed, third party branded, ZED hardware accessory whose packaging
* bears the official "Licensed for ZED" logo. The ZED Camera is an Authorized
*  Accessory solely for purpose of this Software license.
* "Software" means the Software Development Kit, pre-installed in the ZED
* USB flash drive included in the ZED packaging, and including any
* updates STEREOLABS may make available from time to time.
* "Unauthorized Accessories" means all hardware accessories other than
* an Authorized Accessory.
* "Unauthorized Software" means any software not distributed by STEREOLABS.
* "You" means the user of a ZED Camera.
*
* 2. License
*
* a. The Software is licensed to You, not sold. You are licensed to use the
* Software only as pre-installed in Your ZED USB flash drive, and updated by
* STEREOLABS from time to time. You may not copy or reverse engineer the Software.
*
* b. As conditions to this Software license, You agree that:
*       i. You will use Your Software with ZED Camera only and not with any
* other device (including). You will not use Unauthorized Accessories.
* They may not work or may stop working permanently after a Software update.
*       ii. You will not use or install any Unauthorized Software.
* If You do, Your ZED Camera may stop working permanently at that time
* or after a later Software update.
*       iii. You will not attempt to defeat or circumvent any Software
* technical limitation, security, or anti-piracy system. If You do,
* Your ZED Camera may stop working permanently at that time or after a
* later Software update.
*       iv. STEREOLABS may use technical measures, including Software
* updates, to limit use of the Software to the ZED Camera, to prevent
* use of Unauthorized Accessories, and to protect the technical limitations,
* security and anti-piracy systems in the ZED Camera.
*       v. STEREOLABS may update the Software from time to time without
* further notice to You, for example, to update any technical limitation,
* security, or anti-piracy system.
*
* 3. Warranty
* The Software is covered by the Limited Warranty for Your ZED Camera,
* and STEREOLABS gives no other guarantee, warranty, or condition for
* the Software. No one else may give any guarantee, warranty, or condition
* on STEREOLABS's behalf.
*
* 4. EXCLUSION OF CERTAIN DAMAGES
* STEREOLABS IS NOT RESPONSIBLE FOR ANY INDIRECT, INCIDENTAL, SPECIAL, OR
* CONSEQUENTIAL DAMAGES; ANY LOSS OF DATA, PRIVACY, CONFIDENTIALITY, OR
* PROFITS; OR ANY INABILITY TO USE THE SOFTWARE. THESE EXCLUSIONS APPLY
* EVEN IF STEREOLABS HAS BEEN ADVISED OF THE POSSIBILITY OF THESE DAMAGES,
* AND EVEN IF ANY REMEDY FAILS OF ITS ESSENTIAL PURPOSE.
*
* 5. Choice of Law
* French law governs the interpretation of this Software license and any
* claim that STEREOLABS has breached it, regardless of conflict of
* law principles.
*
*/

#ifndef __DEFINES_HPP__
#define __DEFINES_HPP__

#include <cstdint>
#include <cstring>
#include <iostream>
#include <vector>
#include <cmath>

#ifdef _WIN32
#include <Windows.h>
#include <direct.h>
#else /* _WIN32 */
#include <limits>
#include <unistd.h>
#endif /* _WIN32 */

#if defined WIN32
#if defined(SL_SDK_COMPIL)
#define SL_SDK_EXPORT __declspec(dllexport)
#else
#define SL_SDK_EXPORT
#endif
#elif __GNUC__
#define SL_SDK_EXPORT __attribute__((visibility("default")))
#if defined(__arm__) || defined(__aarch64__)
#define _SL_JETSON_
#endif
#endif

// SDK VERSION NUMBER
const int ZED_SDK_MAJOR_VERSION = 2;
const int ZED_SDK_MINOR_VERSION = 1;
const int ZED_SDK_PATCH_VERSION = 2;

namespace sl {

    ///@{
    ///  @name Unavailable Values
    /**
    Defines an unavailable depth value that is above the depth Max value.
    */
    static const float TOO_FAR = INFINITY;
    /**
    Defines an unavailable depth value that is below the depth Min value.
    */
    static const float TOO_CLOSE = -INFINITY;
    /**
    Defines an unavailable depth value that is on an occluded image area.
    */
    static const float OCCLUSION_VALUE = NAN;
    ///@}

    //macro to detect wrong data measure
#define isValidMeasure(v) (std::isfinite(v))

    /// \defgroup Enumerations Public enumerations

    /**
    \enum RESOLUTION
    \ingroup Enumerations
    \brief Represents the available resolution defined in sl::cameraResolution.
    \note Since v1.0, RESOLUTION_VGA mode has been updated to WVGA (from 640*480 to 672*376) and requires a firmware update to function (>= 1142). Firmware can be updated in the ZED Explorer.
    \warning NVIDIA Jetson X1 only supports RESOLUTION_HD1080@15, RESOLUTION_HD720@30/15, and RESOLUTION_VGA@60/30/15.
    */
    enum RESOLUTION {
        RESOLUTION_HD2K, /**< 2208*1242, available framerates: 15 fps.*/
        RESOLUTION_HD1080, /**< 1920*1080, available framerates: 15, 30 fps.*/
        RESOLUTION_HD720, /**< 1280*720, available framerates: 15, 30, 60 fps.*/
        RESOLUTION_VGA, /**< 672*376, available framerates: 15, 30, 60, 100 fps.*/
        RESOLUTION_LAST
    };

    /**
    \enum CAMERA_SETTINGS
    \ingroup Enumerations
    \brief List available camera settings for the ZED camera (contrast, hue, saturation, gain...).
    \brief Each enum defines one of those settings.
    */
    enum CAMERA_SETTINGS {
        CAMERA_SETTINGS_BRIGHTNESS, /**< Defines the brightness control. Affected value should be between 0 and 8.*/
        CAMERA_SETTINGS_CONTRAST, /**< Defines the contrast control. Affected value should be between 0 and 8.*/
        CAMERA_SETTINGS_HUE, /**< Defines the hue control. Affected value should be between 0 and 11.*/
        CAMERA_SETTINGS_SATURATION, /**< Defines the saturation control. Affected value should be between 0 and 8.*/
        CAMERA_SETTINGS_GAIN, /**< Defines the gain control. Affected value should be between 0 and 100 for manual control. If ZED_EXPOSURE is set to -1, the gain is in auto mode too.*/
        CAMERA_SETTINGS_EXPOSURE, /**< Defines the exposure control. A -1 value enable the AutoExposure/AutoGain control,as the boolean parameter (default) does. Affected value should be between 0 and 100 for manual control.*/
        CAMERA_SETTINGS_WHITEBALANCE, /**< Defines the color temperature control. Affected value should be between 2800 and 6500 with a step of 100. A value of -1 set the AWB ( auto white balance), as the boolean parameter (default) does.*/
        CAMERA_SETTINGS_AUTO_WHITEBALANCE, /**< Defines the status of white balance (automatic or manual). A value of 0 disable the AWB, while 1 activate it.*/
        CAMERA_SETTINGS_LAST
    };

    /**
    \enum SELF_CALIBRATION_STATE
    \ingroup Enumerations
    \brief Status for self calibration. Since v0.9.3, self-calibration is done in background and start in the sl::Camera::open or Reset function.
    \brief You can follow the current status for the self-calibration any time once ZED object has been construct.
    */
    enum SELF_CALIBRATION_STATE {
        SELF_CALIBRATION_STATE_NOT_STARTED, /**< Self calibration has not run yet (no sl::Camera::open or sl::Camera::resetSelfCalibration called).*/
        SELF_CALIBRATION_STATE_RUNNING, /**< Self calibration is currently running.*/
        SELF_CALIBRATION_STATE_FAILED, /**< Self calibration has finished running but did not manage to get accurate values. Old parameters are taken instead.*/
        SELF_CALIBRATION_STATE_SUCCESS, /**< Self calibration has finished running and did manage to get accurate values. New parameters are set.*/
        SELF_CALIBRATION_STATE_LAST
    };

    /**
    \enum DEPTH_MODE
    \ingroup Enumerations
    \brief List available depth computation modes.
    */
    enum DEPTH_MODE {
        DEPTH_MODE_NONE, /**< This mode does not compute any depth map. Only rectified stereo images will be available.*/
        DEPTH_MODE_PERFORMANCE, /**< Fastest mode for depth computation.*/
        DEPTH_MODE_MEDIUM, /**< Balanced quality mode. Depth map is robust in any environment and requires medium resources for computation.*/
        DEPTH_MODE_QUALITY, /**< Best quality mode. Requires more compute power.*/
        DEPTH_MODE_LAST
    };

    /**
    \enum SENSING_MODE
    \ingroup Enumerations
    \brief List available depth sensing modes.
    */
    enum SENSING_MODE {
        SENSING_MODE_STANDARD, /**< This mode outputs ZED standard depth map that preserves edges and depth accuracy.
                               * Applications example: Obstacle detection, Automated navigation, People detection, 3D reconstruction.*/
        SENSING_MODE_FILL, /**< This mode outputs a smooth and fully dense depth map.
                           * Applications example: AR/VR, Mixed-reality capture, Image post-processing.*/
        SENSING_MODE_LAST
    };

    /**
    \enum UNIT
    \ingroup Enumerations
    \brief List available unit for measures.
    */
    enum UNIT {
        UNIT_MILLIMETER, /**< International System, 1/1000 METER. */
        UNIT_CENTIMETER, /**< International System, 1/100 METER. */
        UNIT_METER, /**< International System, 1 METER */
        UNIT_INCH, /**< Imperial Unit, 1/12 FOOT */
        UNIT_FOOT, /**< Imperial Unit, 1 FOOT */
        UNIT_LAST
    };

    /**
    \enum COORDINATE_SYSTEM
    \ingroup Enumerations
    \brief List available coordinates systems for positional tracking and 3D measures.
    */
    enum COORDINATE_SYSTEM {
        COORDINATE_SYSTEM_IMAGE, /**< Standard coordinates system in computer vision. Used in OpenCV : see here : http://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html */
        COORDINATE_SYSTEM_LEFT_HANDED_Y_UP, /**< Left-Handed with Y up and Z forward. Used in Unity with DirectX. */
        COORDINATE_SYSTEM_RIGHT_HANDED_Y_UP, /**< Right-Handed with Y pointing up and Z backward. Used in OpenGL. */
        COORDINATE_SYSTEM_RIGHT_HANDED_Z_UP, /**< Right-Handed with Z pointing up and Y forward. Used in 3DSMax. */
        COORDINATE_SYSTEM_LEFT_HANDED_Z_UP, /**< Left-Handed with Z axis pointing up and X forward. Used in Unreal Engine. */
        COORDINATE_SYSTEM_LAST
    };

    /**
    \enum MEASURE
    \ingroup Enumerations
    \brief List retrievable measures.
    */
    enum MEASURE {
        MEASURE_DISPARITY, /**< Disparity map, sl::MAT_TYPE_32F_C1.*/
        MEASURE_DEPTH, /**< Depth map, sl::MAT_TYPE_32F_C1.*/
        MEASURE_CONFIDENCE, /**< Certainty/confidence of the disparity map, sl::MAT_TYPE_32F_C1.*/
        MEASURE_XYZ, /**< Point cloud, sl::MAT_TYPE_32F_C4, channel 4 is empty.*/
        MEASURE_XYZRGBA, /**< Colored point cloud,  sl::MAT_TYPE_32F_C4, channel 4 contains color in R-G-B-A order.*/
        MEASURE_XYZBGRA, /**< Colored point cloud,  sl::MAT_TYPE_32F_C4, channel 4 contains color in B-G-R-A order.*/
        MEASURE_XYZARGB, /**< Colored point cloud,  sl::MAT_TYPE_32F_C4, channel 4 contains color in A-R-G-B order.*/
        MEASURE_XYZABGR, /**< Colored point cloud,  sl::MAT_TYPE_32F_C4, channel 4 contains color in A-B-G-R order.*/
        MEASURE_NORMALS, /**< Normals vector,  sl::MAT_TYPE_32F_C4, channel 4 is empty (set to 0)*/
        MEASURE_DISPARITY_RIGHT, /**< Disparity map for right sensor, sl::MAT_TYPE_32F_C1.*/
        MEASURE_DEPTH_RIGHT, /**< Depth map for right sensor, sl::MAT_TYPE_32F_C1.*/
        MEASURE_XYZ_RIGHT, /**< Point cloud for right sensor, sl::MAT_TYPE_32F_C1, channel 4 is empty.*/
        MEASURE_XYZRGBA_RIGHT, /**< Colored point cloud for right sensor, sl::MAT_TYPE_32F_C4, channel 4 contains color in R-G-B-A order.*/
        MEASURE_XYZBGRA_RIGHT, /**< Colored point cloud for right sensor, sl::MAT_TYPE_32F_C4, channel 4 contains color in B-G-R-A order.*/
        MEASURE_XYZARGB_RIGHT, /**< Colored point cloud for right sensor, sl::MAT_TYPE_32F_C4, channel 4 contains color in A-R-G-B order.*/
        MEASURE_XYZABGR_RIGHT, /**< Colored point cloud for right sensor, sl::MAT_TYPE_32F_C4, channel 4 contains color in A-B-G-R order.*/
        MEASURE_NORMALS_RIGHT, /**< Normals vector for right view, sl::MAT_TYPE_32F_C4, channel 4 is empty (set to 0)*/
        MEASURE_LAST
    };

    /**
    \enum VIEW
    \ingroup Enumerations
    \brief List available views.
    */
    enum VIEW {
        VIEW_LEFT, /**< Left RGBA image, sl::MAT_TYPE_8U_C4. */
        VIEW_RIGHT, /**< Right RGBA image, sl::MAT_TYPE_8U_C4. */
        VIEW_LEFT_GRAY, /**< Left GRAY image, sl::MAT_TYPE_8U_C1. */
        VIEW_RIGHT_GRAY, /**< Right GRAY image, sl::MAT_TYPE_8U_C1. */
        VIEW_LEFT_UNRECTIFIED, /**< Left RGBA unrectified image, sl::MAT_TYPE_8U_C4. */
        VIEW_RIGHT_UNRECTIFIED,/**< Right RGBA unrectified image, sl::MAT_TYPE_8U_C4. */
        VIEW_LEFT_UNRECTIFIED_GRAY, /**< Left GRAY unrectified image, sl::MAT_TYPE_8U_C1. */
        VIEW_RIGHT_UNRECTIFIED_GRAY,/**< Right GRAY unrectified image, sl::MAT_TYPE_8U_C1. */
        VIEW_SIDE_BY_SIDE, /**< Left and right image (the image width is therefore doubled). RGBA image, sl::MAT_TYPE_8U_C4. */
        VIEW_DEPTH, /**< Color rendering of the depth, sl::MAT_TYPE_8U_C4. */
        VIEW_CONFIDENCE, /**< Color rendering of the depth confidence, sl::MAT_TYPE_8U_C4. */
        VIEW_NORMALS, /**< Color rendering of the normals, sl::MAT_TYPE_8U_C4. */
        VIEW_DEPTH_RIGHT, /**< Color rendering of the right depth mapped on right sensor, sl::MAT_TYPE_8U_C4. */
        VIEW_NORMALS_RIGHT, /**< Color rendering of the normals mapped on right sensor, sl::MAT_TYPE_8U_C4. */
        VIEW_LAST
    };

    /**
    \enum DEPTH_FORMAT
    \ingroup Enumerations
    \brief List available file formats for saving depth maps.
    */
    enum DEPTH_FORMAT {
        DEPTH_FORMAT_PNG, /**< PNG image format in 16bits. 32bits depth is mapped to 16bits color image to preserve the consistency of the data range.*/
        DEPTH_FORMAT_PFM, /**< stream of bytes, graphic image file format.*/
        DEPTH_FORMAT_PGM, /**< gray-scale image format.*/
        DEPTH_FORMAT_LAST
    };

    /**
    \enum POINT_CLOUD_FORMAT
    \ingroup Enumerations
    \brief List available file formats for saving point clouds. Stores the spatial coordinates (x,y,z) of each pixel and optionally its RGB color.
    */
    enum POINT_CLOUD_FORMAT {
        POINT_CLOUD_FORMAT_XYZ_ASCII, /**< Generic point cloud file format, without color information.*/
        POINT_CLOUD_FORMAT_PCD_ASCII, /**< Point Cloud Data file, with color information.*/
        POINT_CLOUD_FORMAT_PLY_ASCII, /**< PoLYgon file format, with color information.*/
        POINT_CLOUD_FORMAT_VTK_ASCII, /**< Visualization ToolKit file, without color information.*/
        POINT_CLOUD_FORMAT_LAST
    };

    /**
    \enum TRACKING_STATE
    \ingroup Enumerations
    \brief List the different states of positional tracking.
    */
    enum TRACKING_STATE {
        TRACKING_STATE_SEARCHING, /**< The camera is searching for a previously known position to locate itself.*/
        TRACKING_STATE_OK, /**< Positional tracking is working normally.*/
        TRACKING_STATE_OFF, /**< Positional tracking is not enabled.*/
        TRACKING_STATE_FPS_TOO_LOW, /**< Effective FPS is too low to give proper results for motion tracking. Consider using PERFORMANCES parameters (DEPTH_MODE_PERFORMANCE, low camera resolution (VGA,HD720))*/
        TRACKING_STATE_LAST
    };

    /**
    \enum AREA_EXPORT_STATE
    \ingroup Enumerations
    \brief List the different states of spatial memory area export.
    */
    enum AREA_EXPORT_STATE {
        AREA_EXPORT_STATE_SUCCESS, /**< The spatial memory file has been successfully created.*/
        AREA_EXPORT_STATE_RUNNING, /**< The spatial memory is currently written.*/
        AREA_EXPORT_STATE_NOT_STARTED, /**< The spatial memory file exportation has not been called.*/
        AREA_EXPORT_STATE_FILE_EMPTY, /**< The spatial memory contains no data, the file is empty.*/
        AREA_EXPORT_STATE_FILE_ERROR, /**< The spatial memory file has not been written because of a wrong file name.*/
        AREA_EXPORT_STATE_SPATIAL_MEMORY_DISABLED, /**< The spatial memory learning is disable, no file can be created.*/
        AREA_EXPORT_STATE_LAST
    };

    /**
    \enum REFERENCE_FRAME
    \ingroup Enumerations
    \brief Define which type of position matrix is used to store camera path and pose.
    */
    enum REFERENCE_FRAME {
        REFERENCE_FRAME_WORLD, /**< The transform of sl::Pose will contains the motion with reference to the world frame (previously called PATH).*/
        REFERENCE_FRAME_CAMERA, /**< The transform of sl::Pose will contains the motion with reference to the previous camera frame (previously called POSE).*/
        REFERENCE_FRAME_LAST
    };

    /**
    \enum SPATIAL_MAPPING_STATE
    \ingroup Enumerations
    \brief Gives the spatial mapping state.
    */
    enum SPATIAL_MAPPING_STATE {
        SPATIAL_MAPPING_STATE_INITIALIZING, /**< The spatial mapping is initializing.*/
        SPATIAL_MAPPING_STATE_OK, /**< The depth and tracking data were correctly integrated in the fusion algorithm.*/
        SPATIAL_MAPPING_STATE_NOT_ENOUGH_MEMORY, /**< The maximum memory dedicated to the scanning has been reach, the mesh will no longer be updated.*/
        SPATIAL_MAPPING_STATE_NOT_ENABLED, /**< Camera::enableSpatialMapping() wasn't called (or the scanning was stopped and not relaunched).*/
        SPATIAL_MAPPING_STATE_FPS_TOO_LOW, /**< Effective FPS is too low to give proper results for spatial mapping. Consider using PERFORMANCES parameters (DEPTH_MODE_PERFORMANCE, low camera resolution (VGA,HD720), spatial mapping low resolution)*/
        SPATIAL_MAPPING_STATE_LAST
    };

    /**
    \enum SVO_COMPRESSION_MODE
    \ingroup Enumerations
    \brief List available compression modes for SVO recording.
    \brief sl::SVO_COMPRESSION_MODE_LOSSLESS is an improvement of previous lossless compression (used in ZED Explorer), even if size may be bigger, compression time is much faster.
    */
    enum SVO_COMPRESSION_MODE {
        SVO_COMPRESSION_MODE_RAW, /**< RAW images, no compression.*/
        SVO_COMPRESSION_MODE_LOSSLESS, /**< new Lossless, with PNG/ZSTD based compression : avg size = 42% of RAW).*/
        SVO_COMPRESSION_MODE_LOSSY, /**< new Lossy, with JPEG based compression : avg size = 22% of RAW).*/
        SVO_COMPRESSION_MODE_LAST
    };

    /**
    \struct RecordingState
    \brief Recording structure that contains information about SVO.
    */
    struct RecordingState {
        bool status; /**< status of current frame. May be true for success or false if frame could not be written in the SVO file.*/
        double current_compression_time; /**< compression time for the current frame in ms.*/
        double current_compression_ratio; /**< compression ratio (% of raw size) for the current frame.*/
        double average_compression_time; /**< average compression time in ms since beginning of recording.*/
        double average_compression_ratio; /**< compression ratio (% of raw size) since beginning of recording.*/
    };

    ///@{
    ///  @name ZED Camera Resolution
    /**
    Available video modes for the ZED camera.
    */
    static const std::vector<std::pair<int, int>> cameraResolution = {
        std::make_pair(2208, 1242), /**< sl::RESOLUTION_HD2K */
        std::make_pair(1920, 1080), /**< sl::RESOLUTION_HD1080 */
        std::make_pair(1280, 720), /**< sl::RESOLUTION_HD720 */
        std::make_pair(672, 376) /**< sl::RESOLUTION_VGA */
    };
    ///@}

    ///////////////////////////////////////////////////////////////////////////////////////////////////////

    ///@{
    ///  @name Enumeration conversion
    /*!
    \ingroup Functions
    \brief Converts the given RESOLUTION into a string
    \param res : a specific RESOLUTION
    \return The corresponding string
    */
    static inline std::string resolution2str(RESOLUTION res) {
        std::string output;
        switch (res) {
            case RESOLUTION::RESOLUTION_HD2K:
            output = "HD2K";
            break;
            case RESOLUTION::RESOLUTION_HD1080:
            output = "HD1080";
            break;
            case RESOLUTION::RESOLUTION_HD720:
            output = "HD720";
            break;
            case RESOLUTION::RESOLUTION_VGA:
            output = "VGA";
            break;
            case RESOLUTION::RESOLUTION_LAST:
            output = "Unknown";
            break;
            default:
            output = "Unknown";
            break;
        }
        return output;
    }

    /*!
    \ingroup Functions
    \brief Converts the given SELF_CALIBRATION_STATE into a string
    \param state : a specific SELF_CALIBRATION_STATE
    \return The corresponding string
    */
    static inline std::string statusCode2str(SELF_CALIBRATION_STATE state) {
        std::string output;
        switch (state) {
            case SELF_CALIBRATION_STATE::SELF_CALIBRATION_STATE_NOT_STARTED:
            output = "Self calibration:  Not Started";
            break;
            case SELF_CALIBRATION_STATE::SELF_CALIBRATION_STATE_RUNNING:
            output = "Self calibration:  Running";
            break;
            case SELF_CALIBRATION_STATE::SELF_CALIBRATION_STATE_FAILED:
            output = "Self calibration:  Failed";
            break;
            case SELF_CALIBRATION_STATE::SELF_CALIBRATION_STATE_SUCCESS:
            output = "Self calibration:  Success";
            break;
            case SELF_CALIBRATION_STATE::SELF_CALIBRATION_STATE_LAST:
            output = "Unknown";
            break;
            default:
            output = "Unknown";
            break;
        }
        return output;
    }

    /*!
    \ingroup Functions
    \brief Converts the given string into a DEPTH_MODE
    \param mode : a specific depth
    \return The corresponding DEPTH_MODE
    */
    static inline DEPTH_MODE str2mode(std::string mode) {
        DEPTH_MODE output = DEPTH_MODE_PERFORMANCE;
        if (!mode.compare("None"))
            output = DEPTH_MODE_NONE;
        if (!mode.compare("Performance"))
            output = DEPTH_MODE_PERFORMANCE;
        if (!mode.compare("Medium"))
            output = DEPTH_MODE_MEDIUM;
        if (!mode.compare("Quality"))
            output = DEPTH_MODE_QUALITY;
        return output;
    }

    /*!
    \ingroup Functions
    \brief Converts the given DEPTH_MODE into a string
    \param mode : a specific DEPTH_MODE
    \return The corresponding string
    */
    static inline std::string depthMode2str(DEPTH_MODE mode) {
        std::string output;
        switch (mode) {
            case DEPTH_MODE::DEPTH_MODE_NONE:
            output = "None";
            break;
            case DEPTH_MODE::DEPTH_MODE_PERFORMANCE:
            output = "Performance";
            break;
            case DEPTH_MODE::DEPTH_MODE_MEDIUM:
            output = "Medium";
            break;
            case DEPTH_MODE::DEPTH_MODE_QUALITY:
            output = "Quality";
            break;
            case DEPTH_MODE::DEPTH_MODE_LAST:
            output = "Unknown";
            break;
            default:
            output = "Unknown";
            break;
        }
        return output;
    }

    /*!
    \ingroup Functions
    \brief Converts the given SENSING_MODE into a string
    \param mode : a specific SENSING_MODE
    \return The corresponding string
    */
    static inline std::string sensingMode2str(SENSING_MODE mode) {
        std::string output;
        switch (mode) {
            case SENSING_MODE::SENSING_MODE_STANDARD:
            output = "Standard";
            break;
            case SENSING_MODE::SENSING_MODE_FILL:
            output = "Fill";
            break;
            case SENSING_MODE::SENSING_MODE_LAST:
            output = "Unknown";
            break;
            default:
            output = "Unknown";
            break;
        }
        return output;
    }

    /*!
    \ingroup Functions
    \brief Converts the given UNIT into a string
    \param unit : a specific UNIT
    \return The corresponding string
    */
    static inline std::string unit2str(UNIT unit) {
        std::string output;
        switch (unit) {
            case UNIT::UNIT_MILLIMETER:
            output = "Millimeter";
            break;
            case UNIT::UNIT_CENTIMETER:
            output = "Centimeter";
            break;
            case UNIT::UNIT_METER:
            output = "Meter";
            break;
            case UNIT::UNIT_INCH:
            output = "Inch";
            break;
            case UNIT::UNIT_FOOT:
            output = "Feet";
            break;
            case UNIT::UNIT_LAST:
            output = "Unknown";
            break;
            default:
            output = "Unknown";
            break;
        }
        return output;
    }

    /*!
    \ingroup Functions
    \brief Converts the given string into a UNIT
    \param unit : a specific unit string
    \return The corresponding UNIT
    */
    static inline UNIT str2unit(std::string unit) {
        UNIT output = UNIT_MILLIMETER;
        if (!unit.compare("Millimeter"))
            output = UNIT_MILLIMETER;
        if (!unit.compare("Centimeter"))
            output = UNIT_CENTIMETER;
        if (!unit.compare("Meter"))
            output = UNIT_METER;
        if (!unit.compare("Inch"))
            output = UNIT_INCH;
        if (!unit.compare("Feet"))
            output = UNIT_FOOT;
        return output;
    }

    /*!
    \ingroup Functions
    \brief Converts the given TRACKING_STATE into a string
    \param state : a specific TRACKING_STATE
    \return The corresponding string
    */
    static inline std::string trackingState2str(TRACKING_STATE state) {
        std::string output;
        switch (state) {
            case TRACKING_STATE_SEARCHING:
            output = "Tracking state: Searching";
            break;
            case TRACKING_STATE_OK:
            output = "Tracking state: OK";
            break;
            case TRACKING_STATE_OFF:
            output = "Tracking state: OFF";
            break;
            case TRACKING_STATE_FPS_TOO_LOW:
            output = "Tracking state: FPS too low";
            break;
            case TRACKING_STATE_LAST:
            output = "Unknown";
            break;
            default:
            output = "Unknown";
            break;
        }
        return output;
    }

    /*!
    \ingroup Functions
    \brief Converts the given SPATIAL_MAPPING_STATE into a string
    \param state : a specific SPATIAL_MAPPING_STATE
    \return The corresponding string
    */
    static inline std::string spatialMappingState2str(SPATIAL_MAPPING_STATE state) {
        std::string output;
        switch (state) {
            case SPATIAL_MAPPING_STATE_INITIALIZING:
            output = "Spatial Mapping state: Initializing";
            break;
            case SPATIAL_MAPPING_STATE_OK:
            output = "Spatial Mapping state: OK";
            break;
            case SPATIAL_MAPPING_STATE_NOT_ENOUGH_MEMORY:
            output = "Spatial Mapping state: Not Enough Memory";
            break;
            case SPATIAL_MAPPING_STATE_NOT_ENABLED:
            output = "Spatial Mapping state: Not Enabled";
            break;
            case SPATIAL_MAPPING_STATE_FPS_TOO_LOW:
            output = "Spatial Mapping state: FPS too low";
            break;
            case SPATIAL_MAPPING_STATE_LAST:
            output = "Unknown";
            break;
            default:
            output = "Unknown";
            break;
        }
        return output;
    }
    ///@}
};

#endif /*__DEFINES_HPP__*/
