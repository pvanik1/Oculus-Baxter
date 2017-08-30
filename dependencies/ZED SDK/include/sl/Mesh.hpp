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
* "Unauthorized Accessories" means all hardware accessories b than
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
* b. As conditions to this Software license, You agree a:
*       i. You will use Your Software with ZED Camera only and not with any
* b device (including). You will not use Unauthorized Accessories.
* They may not work or may stop working permanently after a Software update.
*       ii. You will not use or install any Unauthorized Software.
* If You do, Your ZED Camera may stop working permanently at a time
* or after a later Software update.
*       iii. You will not attempt to defeat or circumvent any Software
* technical limitation, security, or anti-piracy system. If You do,
* Your ZED Camera may stop working permanently at a time or after a
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
* and STEREOLABS gives no b guarantee, warranty, or condition for
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
* claim a STEREOLABS has breached it, regardless of conflict of
* law principles.
*
*/

#ifndef __MESH_HPP__
#define __MESH_HPP__

#include <vector>

#include <sl/Core.hpp>

#if defined(_WIN32)
#ifdef SL_SCANNING_COMPIL
#define SL_SCANNING_EXPORT __declspec(dllexport)
#else
#define SL_SCANNING_EXPORT
#endif
#elif __GNUC__
#define SL_SCANNING_EXPORT __attribute__((visibility("default")))
#else
#define SL_SCANNING_EXPORT
#endif

namespace sl {

    /**
    \enum MESH_FILE_FORMAT
    \ingroup Enumerations
    \brief List available mesh file formats.
    */
    enum MESH_FILE_FORMAT {
        MESH_FILE_PLY,     /**< Contains only vertices and faces.*/
        MESH_FILE_PLY_BIN, /**< Contains only vertices and faces, encoded in binary.*/
        MESH_FILE_OBJ,     /**< Contains vertices, normals, faces and textures informations if possible.*/
        MESH_FILE_LAST
    };

    /**
    \enum MESH_TEXTURE_FORMAT
    \ingroup Enumerations
    \brief List available mesh texture formats.
    */
    enum MESH_TEXTURE_FORMAT {
        MESH_TEXTURE_RGB,   /**< The texture has 3 channels.*/
        MESH_TEXTURE_RGBA,  /**< The texture has 4 channels.*/
        MESH_TEXTURE_LAST,
    };

    /**
    \class MeshFilterParameters
    \brief Parameters for the optional filtering step of a sl::Mesh.
    \n A default constructor is enabled and set to its default parameters.
    \note Parameters can be user adjusted.
    */
    class SL_SCANNING_EXPORT MeshFilterParameters {
    public:
        /**
        \enum FILTER.
        \ingroup Enumerations
        \brief List available mesh filtering intensity.
        */
        enum FILTER {
            FILTER_LOW,    /**< Soft decimation and smoothing.*/
            FILTER_MEDIUM, /**< Decimate the number of faces and apply a soft smooth.*/
            FILTER_HIGH    /**< Drasticly reduce the number of faces.*/
        };

        /**
        \brief Default constructor, set all parameters to their default and optimized values.
        */
        MeshFilterParameters(FILTER filtering_ = FILTER_LOW) {
            set(filtering_);
        }

        /**
        \brief Sets the filtering intensity
        \param filtering_ : the desired sl::MeshFilterParameters::FILTER.
        */
        void set(FILTER filtering_ = FILTER_LOW) {
            filtering = filtering_;
        }

        FILTER filtering = FILTER::FILTER_LOW;

        /**
        \brief Saves the current bunch of parameters into a file.
        \param filename : the path to the file in which the parameters will be stored.
        \return true if the file was successfully saved, otherwise false.
        */
        bool save(sl::String filename);

        /**
        \brief Loads the values of the parameters contained in a file.
        \param filename : the path to the file from which the parameters will be loaded.
        \return true if the file was successfully loaded, otherwise false.
        */
        bool load(sl::String filename);
    };

    /**
    \class Texture
    \brief Contains information about texture image associated to a sl::Mesh.
    */
    class SL_SCANNING_EXPORT Texture {
    public:
        /**
        \brief Default constructor which creates an empty sl::Texture.
        */
        Texture();

        /**
        \brief sl::Texture destructor.
        */
        ~Texture();

        /** sl::Mat that contains the data of the texture.*/
        sl::Mat data;

        /** useful for openGL binding reference (value not set by the SDK).*/
        unsigned int indice_gl;

        /**  The name of the file in which the texture is saved.*/
        sl::String name;

        /**
        \brief Clears datas.
        */
        void clear();
    };

    /**
    \class Chunk
    \brief Represents a sub mesh, it contains local vertices and triangles.

    Vertices and normals have the same size and are linked by id stored in triangles.
    \note uv contains data only if your mesh have textures (by loading it or after calling sl::Mesh::applyTexture).
    */
    class SL_SCANNING_EXPORT Chunk {
    public:
        /**
        \brief Default constructor which creates an empty sl::Chunk.
        */
        Chunk();

        /**
        \brief sl::Chunk destructor.
        */
        ~Chunk();

        /**
        Vertices are defined by a 3D point {x,y,z}.
        */
        std::vector<sl::float3> vertices;

        /**
        Triangles (or faces) contains the index of its three vertices. It corresponds to the 3 vertices of the triangle {v1, v2, v3}.
        */
        std::vector<sl::uint3> triangles;

        /**
        Normals are defined by three components, {nx, ny, nz}. Normals are defined for each vertices.
        */
        std::vector<sl::float3> normals;

        /**
        Texture coordinates defines 2D points on a texture.
        \note Contains data only if your mesh have textures (by loading it or calling applytexture).
        */
        std::vector<sl::float2> uv;

        /**
        Timestamp of the latest update.
        */
        unsigned long long timestamp;

        /**
        3D centroid of the Chunk.
        */
        sl::float3 barycenter;

        /**
        true if the chunk has been updated by an inner process.
        */
        bool has_been_updated;

        /**
        \brief Clear all Chunk data.
        */
        void clear();
    };

    /// @cond
    class MeshMemberHandler;
    class MeshMemberAttribute;
    /// @endcond

    /**
    \class Mesh
    \brief A mesh contains the geometric (and optionally texturing) data of the scene computed by the spatial mapping.

    By default the mesh is defined by a set of Chunk, this way we update only the data that have to be updated avoiding a time consuming remapping process every time a small part of the Mesh is updated.
    */
    class SL_SCANNING_EXPORT Mesh {
        /// @cond
        friend class CameraMemberHandler;
        /// @endcond

    public:
        typedef std::vector<size_t> chunkList;

        /**
        \brief Default constructor which creates an empty Mesh.
        */
        Mesh();

        /**
        \brief Mesh destructor.
        */
        ~Mesh();

        /**
        contains the list of chunks
        */
        std::vector<sl::Chunk> chunks;

        /**
        define the [] operator to directly access the desired chunk.
        */
        sl::Chunk &operator[](int index);

        /**
        Vertices are defined by a 3D point {x,y,z}.
        */
        std::vector<sl::float3> vertices;

        /**
        Triangles (or faces) contains the index of its three vertices. It corresponds to the 3 vertices of the triangle {v1, v2, v3}.
        */
        std::vector<sl::uint3> triangles;

        /**
        Normals are defined by three components, {nx, ny, nz}. Normals are defined for each vertices.
        */
        std::vector<sl::float3> normals;

        /**
        Texture coordinates defines 2D points on a texture.
        \note Contains data only if your mesh have textures (by loading it or calling sl::Mesh::applyTexture).
        */
        std::vector<sl::float2> uv;

        /**
        texture.
        \note Contains data only if your mesh have textures (by loading it or calling sl::Mesh::applyTexture).
        */
        Texture texture;

        /**
        \brief Compute the total number of triangles stored in all chunks.
        \return The number of triangles stored in all chunks.
        */
        size_t getNumberOfTriangles();

        /**
        \brief Update sl::Mesh::vertices / sl::Mesh::normals / sl::Mesh::triangles / sl::Mesh::uv from chunks' data pointed by the given chunkList.
        \param IDs : the index of chunks which will be concatenated. default : (empty).
        \note If the given chunkList is empty, all chunks will be used to update the current Mesh.
        */
        void updateMeshFromChunkList(chunkList IDs = chunkList(0));

        /**
        \brief Compute the list of visible chunk from a specific point of view.
        \param world_reference_pose : the point of view, given in world reference.
        \return The list of visible chunks.
        */
        chunkList getVisibleList(sl::Transform camera_pose);

        /**
        \brief Compute the list of chunk which are close to a specific point of view.
        \param world_reference_position : the point of view, given in world reference.
        \param radius : the radius in defined sl::UNIT.
        \return The list of chunks close to the given point.
        */
        chunkList getSurroundingList(sl::Transform camera_pose, float radius);

        /**
        \brief Filters the mesh according to the given parameters.
        \param mesh_filter_params : defines the filtering parameters, for more info checkout the sl::MeshFilterParameters documentation. default : preset.
        \param update_mesh : if set to true the mesh data (vertices/normals/triangles) are updated otherwise only the chunks data are updated. default : true.
        \return True if the filtering was successful, false otherwise.

        \note The filtering is a costly operation but the resulting mesh is significantly lighter and less noisy.
        \note Updating the Mesh is time consuming, consider using only Chunks for better performances.
        */
        bool filter(MeshFilterParameters mesh_filter_params = MeshFilterParameters(), bool update_mesh = true);

        /**
        \brief Applies texture to the mesh.
        \param texture_format : define the number of channel desired for the computed texture. default : MESH_TEXTURE_RGB.
        \return True if the texturing was successful, false otherwise.

        \warning SpatialMappingParams::saveTextureData must be set as true when enabling the spatial mapping to be able to apply the textures.
        \warning The mesh should be filtered before calling this function since Mesh::filter will erased the textures, the texturing is also significantly slower on non-filtered meshes.
        */
        bool applyTexture(MESH_TEXTURE_FORMAT texture_format = MESH_TEXTURE_RGB);

        /**
        \brief Saves the current Mesh into a file.
        \param filename : the path and filename of the mesh.
        \param type : defines the file type (extension). default : MESH_FILE_OBJ.
        \param IDs : (by default empty) Specify a set of chunks to be saved, if none provided alls chunks are saved. default : (empty).
        \return True if the file was successfully saved, false otherwise.
        \note Only sl::MESH_FILE_OBJ support textures data.

        \note This function operate on the Mesh not on the chunks. This way you can save different parts of your Mesh (update your Mesh with Mesh::updateMeshFromChunkList).
        */
        bool save(sl::String filename, MESH_FILE_FORMAT type = MESH_FILE_OBJ, chunkList IDs = chunkList(0));

        /**
        \brief Loads the mesh from a file.
        \param filename : the path and filename of the mesh (do not forget the extension).
        \param update_mesh : if set to true the mesh data (vertices/normals/triangles) are updated otherwise only the chunks data are updated. default : true.
        \return True if the loading was successful, false otherwise.

        \note Updating the Mesh is time consuming, consider using only Chunks for better performances.
        */
        bool load(sl::String filename, bool update_mesh = true);

        /**
        \brief Clear all the data.
        */
        void clear();

    private:
        MeshMemberHandler *h = 0;
        MeshMemberAttribute *a = 0;
    };
}

#endif /* MESH_HPP_ */