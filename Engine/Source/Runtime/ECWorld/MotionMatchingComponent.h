#pragma once

#include "Core/StringCrc.h"
#include "Math/Box.hpp"

#include <vector>
#include "MotionMatching/character.h"
#include <Utilities/MeshUtils.hpp>

namespace engine
{

class MotionMatchingComponent final
{
public:
	static constexpr StringCrc GetClassName()
	{
		constexpr StringCrc className("MotionMatchingComponent");
		return className;
	}

public:
	MotionMatchingComponent() = default;
	MotionMatchingComponent(const MotionMatchingComponent&) = default;
	MotionMatchingComponent& operator=(const MotionMatchingComponent&) = default;
	MotionMatchingComponent(MotionMatchingComponent&&) = default;
	MotionMatchingComponent& operator=(MotionMatchingComponent&&) = default;
	~MotionMatchingComponent() = default;


	character GetCharacter() { return m_character; }
	Mesh& GetMesh() { return m_mesh; }
	void SetMesh(Mesh& mesh) { m_mesh = cd::MoveTemp(mesh); }
	void SetCDMesh(cd::Mesh& mesh) { m_CDMesh = cd::MoveTemp(mesh); }

	void Reset();
	void Build();


private:
	character m_character;
	Mesh m_mesh;
	cd::Mesh m_CDMesh;
};

}