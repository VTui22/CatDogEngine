#include "MotionMatchingComponent.h"

#include "Math/MeshGenerator.h"
#include "Rendering/Utility/VertexLayoutUtility.h"
#include "Scene/Mesh.h"
#include "Scene/VertexFormat.h"

namespace engine
{

void MotionMatchingComponent::Reset()
{

}

void MotionMatchingComponent::Build()
{
    
    m_CDMesh.Init(static_cast<uint32_t>(m_character.positions.size));
    for (uint32_t i = 0U; i < m_character.positions.size; ++i)
    {
        cd::Vec3f vertexPosition = cd::Vec3f(m_character.positions.data[i].x, m_character.positions.data[i].y, m_character.positions.data[i].z);
        m_CDMesh.SetVertexPosition(i, vertexPosition);

        cd::Vec3f vertexNormal = cd::Vec3f(m_character.normals.data[i].x, m_character.normals.data[i].y, m_character.normals.data[i].z);
        m_CDMesh.SetVertexNormal(i, vertexNormal);
    }
}
}